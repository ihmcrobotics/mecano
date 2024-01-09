package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.*;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.*;

/**
 * This calculator can be used to evaluate the joint efforts due to gravity and external wrenches,
 * as well as to evaluate the gradient of these efforts.
 * <p>
 * The gradient can be used to predict the change in joint effort that the change in joint
 * configuration can induce. In turn, this can be used to formulate a minimization problem that
 * allows to reduce joint efforts due to gravity and external wrenches by driving the joint to an
 * optimal configuration.
 * </p>
 * <p>
 * The gradient can be used to predict the joint efforts given a variation in joint
 * configuration:<br>
 * <tt>&tau;(q + &delta;q) = &tau;(q) + &nabla;&tau;(q) &delta;q</tt><br>
 * where:
 * <ul>
 * <li>q is the N vector of joint configurations,
 * <li>&delta;q is the N vector of variation in joint configurations,
 * <li>&tau;(q) is the N vector of joint efforts given the joint configuration q,
 * <li>&nabla;&tau;(q) is the N-by-N matrix representing the gradient of joint efforts for the given
 * joint configuration.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class MultiBodyGravityGradientCalculator
{
   /**
    * Defines the multi-body system to use with this calculator.
    */
   private final MultiBodySystemReadOnly input;
   /**
    * Output of this algorithm: joint efforts due to gravity and external wrenches for all the joint to
    * consider.
    */
   private final DMatrixRMaj tauMatrix;
   /**
    * Output of this algorithm: gradient of the {@link #tauMatrix}.
    */
   private final DMatrixRMaj tauGradientMatrix;
   /**
    * The root of the internal recursive algorithm.
    */
   private final AlgorithmStep initialStep;
   /**
    * Map to quickly retrieve information for each rigid-body.
    */
   private final Map<RigidBodyReadOnly, AlgorithmStep> rigidBodyToAlgorithmStepMap = new LinkedHashMap<>();

   /**
    * The gravitational acceleration expressed in the inertial frame, typically equal to
    * {@code (0, 0, -9.81)}.
    */
   private final FrameVector3D gravitationalAcceleration = new FrameVector3D();
   /**
    * Dirty bit used to indicate if the output of this algorithm needs to be refreshed next time it is
    * accessed.
    */
   private boolean dirtyFlag = true;

   /**
    * Creates a calculator for that considers all the descendants of the given {@code rootBody}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param rootBody the supporting body of the subtree to be evaluated by this calculator. Not
    *                 modified.
    */
   public MultiBodyGravityGradientCalculator(RigidBodyReadOnly rootBody)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody));
   }

   /**
    * @param input
    */
   public MultiBodyGravityGradientCalculator(MultiBodySystemReadOnly input)
   {
      this.input = input;

      tauGradientMatrix = new DMatrixRMaj(input.getNumberOfDoFs(), input.getNumberOfDoFs());
      tauMatrix = new DMatrixRMaj(input.getNumberOfDoFs(), 1);

      initialStep = new AlgorithmStep(input.getRootBody(), null, null);
      rigidBodyToAlgorithmStepMap.put(input.getRootBody(), initialStep);
      buildMultiBodyTree(initialStep, input.getJointsToIgnore());
      initialStep.includeIgnoredSubtreeInertia();

      gravitationalAcceleration.setToZero(input.getInertialFrame());
   }

   private List<AlgorithmStep> buildMultiBodyTree(AlgorithmStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<AlgorithmStep> algorithmSteps = new ArrayList<>();
      algorithmSteps.add(parent);

      List<JointReadOnly> childrenJoints = MultiBodySystemTools.sortLoopClosureInChildrenJoints(parent.rigidBody);

      for (JointReadOnly childJoint : childrenJoints)
      {
         if (jointsToIgnore.contains(childJoint))
            continue;

         if (childJoint.isLoopClosure())
         {
            /*
             * We simply skip any loop closure joint which will leave their columns in the matrix set to zero,
             * which is what we want.
             */
            continue;
         }

         RigidBodyReadOnly childBody = childJoint.getSuccessor();

         if (childBody != null && !rigidBodyToAlgorithmStepMap.containsKey(childBody))
         {
            int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
            AlgorithmStep child = new AlgorithmStep(childBody, parent, jointIndices);
            rigidBodyToAlgorithmStepMap.put(childBody, child);
            algorithmSteps.addAll(buildMultiBodyTree(child, jointsToIgnore));
         }
      }

      return algorithmSteps;
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration, it is usually equal to
    *                {@code (0, 0, -9.81)}.
    */
   public void setGravitionalAcceleration(FrameTuple3DReadOnly gravity)
   {
      gravitationalAcceleration.set(gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration, it is usually equal to
    *                {@code (0, 0, -9.81)}.
    */
   public void setGravitionalAcceleration(Tuple3DReadOnly gravity)
   {
      gravitationalAcceleration.setIncludingFrame(input.getInertialFrame(), gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration along the z-axis, it is usually equal to
    *                {@code -9.81}.
    */
   public void setGravitionalAcceleration(double gravity)
   {
      setGravitionalAcceleration(0.0, 0.0, gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravityX the gravitational linear acceleration along the x-axis, it is usually equal to
    *                 {@code 0}.
    * @param gravityY the gravitational linear acceleration along the y-axis, it is usually equal to
    *                 {@code 0}.
    * @param gravityZ the gravitational linear acceleration along the z-axis, it is usually equal to
    *                 {@code -9.81}.
    */
   public void setGravitionalAcceleration(double gravityX, double gravityY, double gravityZ)
   {
      gravitationalAcceleration.setIncludingFrame(input.getInertialFrame(), gravityX, gravityY, gravityZ);
   }

   /**
    * Resets all the external wrenches that were added to the rigid-bodies.
    */
   public void setExternalWrenchesToZero()
   {
      initialStep.setExternalWrenchToZeroRecursive();
   }

   /**
    * Gets the internal reference to the external wrench associated with the given rigidBody.
    * <p>
    * Modify the return wrench to configure the wrench to be applied on this rigid-body.
    * </p>
    * <p>
    * It is assumed to be independent from the system configuration while expressed in inertial frame.
    * However, note that the wrench is actually expressed in local frame instead of inertial frame to
    * keep the moment to a numerically reasonable value.
    * </p>
    *
    * @param rigidBody the query. Not modified.
    * @return the wrench associated to the query.
    */
   public FixedFrameWrenchBasics getExternalWrench(RigidBodyReadOnly rigidBody)
   {
      return rigidBodyToAlgorithmStepMap.get(rigidBody).externalWrench;
   }

   /**
    * Sets external wrench to apply to the given {@code rigidBody}.
    * <p>
    * It is assumed to be independent from the system configuration while expressed in inertial frame.
    * However, note that the wrench is actually expressed in local frame instead of inertial frame to
    * keep the moment to a numerically reasonable value.
    * </p>
    *
    * @param rigidBody      the rigid-body to which the wrench is to applied. Not modified.
    * @param externalWrench the external wrench to apply to the rigid-body.
    */
   public void setExternalWrench(RigidBodyReadOnly rigidBody, WrenchReadOnly externalWrench)
   {
      getExternalWrench(rigidBody).setMatchingFrame(externalWrench);
   }

   /**
    * Marks the internal data as out-dated, it will be re-evaluated next time {@link #getTauMatrix()}
    * or {@link #getTauGradientMatrix()} is called.
    */
   public void reset()
   {
      dirtyFlag = true;
   }

   private void update()
   {
      if (!dirtyFlag)
         return;

      dirtyFlag = false;
      initialStep.passOne();
      initialStep.passTwo();
   }

   /**
    * Gets the definition of the multi-body system that was used to create this calculator.
    *
    * @return this calculator input.
    */
   public MultiBodySystemReadOnly getInput()
   {
      return input;
   }

   /**
    * Gets the N-by-1 matrix containing the joint efforts due to gravity and external wrenches applied
    * to the multi-body system. (N is the number of degrees of freedom of the system).
    *
    * @return this calculator output: the joint efforts.
    */
   public DMatrixRMaj getTauMatrix()
   {
      update();
      return tauMatrix;
   }

   /**
    * Gets the N-by-N matrix representing the gradient of the joint efforts {@link #getTauMatrix()}. (N
    * is the number of degrees of freedom of the system).
    * <p>
    * The gradient can be used to predict the joint efforts given a variation in joint
    * configuration:<br>
    * <tt>&tau;(q + &delta;q) = &tau;(q) + &nabla;&tau;(q) &delta;q</tt><br>
    * where:
    * <ul>
    * <li>q is the N vector of joint configurations,
    * <li>&delta;q is the N vector of variation in joint configurations,
    * <li>&tau;(q) is the N vector of joint efforts given the joint configuration q,
    * <li>&nabla;&tau;(q) is the N-by-N matrix representing the gradient of joint efforts for the given
    * joint configuration.
    * </ul>
    * </p>
    *
    * @return the gradient of the joint efforts.
    */
   public DMatrixRMaj getTauGradientMatrix()
   {
      update();
      return tauGradientMatrix;
   }

   /**
    * Intermediate variable used to compute {@link AlgorithmStep#subTreeExternalSpatialForce}.
    */
   private final SpatialForce childExternalSpatialForce = new SpatialForce();
   /**
    * Intermediate variable used to compute {@link AlgorithmStep#subTreeCoM}.
    */
   private final FramePoint3D childCoM = new FramePoint3D();

   /**
    * This class represents a single step for this algorithm.
    */
   private class AlgorithmStep
   {
      /**
       * The rigid-body for which this step is for.
       */
      private final RigidBodyReadOnly rigidBody;
      /**
       * Body inertia: usually equal to {@code rigidBody.getInertial()}. However, if at least one child of
       * {@code rigidBody} is ignored, it is equal to this rigid-body inertia and the subtree inertia
       * attached to the ignored joint.
       */
      private final SpatialInertia bodyInertia;
      /**
       * The corresponding matrix indices for each of this step's joint degree of freedom.
       */
      private final int[] jointIndices;
      /**
       * The algorithm step for the parent of this step's rigid-body.
       */
      private final AlgorithmStep parent;
      /**
       * The algorithm steps for the children of this step's rigid-body.
       */
      private final List<AlgorithmStep> children = new ArrayList<>();

      /**
       * User input: external wrench to be applied to this body.
       * <p>
       * It is assumed to be independent from the system configuration while expressed in inertial frame.
       * The wrench is actually expressed in local frame instead of inertial frame to keep the moment to a
       * numerically reasonable value.
       * </p>
       */
      private final FixedFrameWrenchBasics externalWrench;
      /**
       * Marker used to keep track of whether there is an external wrench being applied to this step's
       * rigid-body.
       */
      private boolean hasExternalWrench = false;
      /**
       * Marker used to keep track of whether there's at least one external wrench being applied to any of
       * the descendants to this step's, including this step.
       */
      private boolean hasSubTreeExternalWrench = false;

      /**
       * The total mass of the subtree starting at this step.
       */
      private double subTreeMass;
      /**
       * The total center of mass of the subtree starting at this step.
       */
      private final FramePoint3D subTreeCoM = new FramePoint3D();
      /**
       * The sum of external wrenches applied to the subtree starting at this step.
       */
      private final SpatialForce subTreeExternalSpatialForce = new SpatialForce();
      /**
       * The resulting force due to gravity on the subtree at the sutree's center of mass expressed in
       * local coordinates of this step's joint.
       */
      private final FrameVector3D gravityForceAtCoM = new FrameVector3D();

      public AlgorithmStep(RigidBodyReadOnly rigidBody, AlgorithmStep parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;

         if (isRoot())
         {
            bodyInertia = null;
            externalWrench = null;
         }
         else
         {
            parent.children.add(this);
            bodyInertia = new SpatialInertia(rigidBody.getInertia());
            externalWrench = new Wrench(getBodyFixedFrame(), getBodyFixedFrame());
         }
      }

      public void includeIgnoredSubtreeInertia()
      {
         if (!isRoot() && children.size() != rigidBody.getChildrenJoints().size())
         {
            for (JointReadOnly childJoint : rigidBody.getChildrenJoints())
            {
               if (input.getJointsToIgnore().contains(childJoint))
               {
                  SpatialInertia subtreeIneria = MultiBodySystemTools.computeSubtreeInertia(childJoint);
                  subtreeIneria.changeFrame(rigidBody.getBodyFixedFrame());
                  bodyInertia.add(subtreeIneria);
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).includeIgnoredSubtreeInertia();
      }

      public void setExternalWrenchToZeroRecursive()
      {
         if (!isRoot())
            externalWrench.setToZero();

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            children.get(childIndex).setExternalWrenchToZeroRecursive();
         }
      }

      private final Twist unitTwist_i = new Twist();

      /**
       * From leaves to root, updates the forces needed to calculate the effort matrix and its gradient.
       */
      public void passOne()
      {
         for (int i = 0; i < children.size(); i++)
            children.get(i).passOne();

         // Update the subtree mass and external wrenches
         subTreeMass = bodyInertia == null ? 0.0 : bodyInertia.getMass();

         for (int i = 0; i < children.size(); i++)
         {
            subTreeMass += children.get(i).subTreeMass;
         }

         if (!isRoot())
         {
            hasExternalWrench = externalWrench.getLinearPartX() != 0.0 || externalWrench.getLinearPartY() != 0.0 || externalWrench.getLinearPartZ() != 0.0
                                || externalWrench.getAngularPartX() != 0.0 || externalWrench.getAngularPartY() != 0.0
                                || externalWrench.getAngularPartZ() != 0.0;
            hasSubTreeExternalWrench = hasExternalWrench;
            subTreeExternalSpatialForce.setIncludingFrame(externalWrench);
            subTreeExternalSpatialForce.changeFrame(getFrameAfterJoint());

            for (int i = 0; i < children.size(); i++)
            {
               if (children.get(i).hasSubTreeExternalWrench)
               {
                  childExternalSpatialForce.setIncludingFrame(children.get(i).subTreeExternalSpatialForce);
                  childExternalSpatialForce.changeFrame(getFrameAfterJoint());
                  subTreeExternalSpatialForce.add(childExternalSpatialForce);
                  hasSubTreeExternalWrench = true;
               }
            }
         }

         // Update the force due gravity in local coordinates.
         ReferenceFrame frameToUse = isRoot() ? getBodyFixedFrame() : getFrameAfterJoint();
         gravityForceAtCoM.setIncludingFrame(gravitationalAcceleration);
         gravityForceAtCoM.scale(subTreeMass);
         gravityForceAtCoM.changeFrame(frameToUse);

         // Update the center of mass position of the subtree in local coordinates.
         if (bodyInertia == null)
         {
            subTreeCoM.setToZero(getBodyFixedFrame());
         }
         else
         {
            subTreeCoM.setIncludingFrame(bodyInertia.getCenterOfMassOffset());
            subTreeCoM.changeFrame(frameToUse);
            subTreeCoM.scale(bodyInertia.getMass());
         }

         for (int i = 0; i < children.size(); i++)
         {
            childCoM.setIncludingFrame(children.get(i).subTreeCoM);
            childCoM.changeFrame(frameToUse);
            subTreeCoM.scaleAdd(children.get(i).subTreeMass, childCoM, subTreeCoM);
         }

         subTreeCoM.scale(1.0 / subTreeMass);
      }

      /**
       * From leaves to root, compute the elements of the gradient matrix.
       */
      public void passTwo()
      {
         for (int i = 0; i < children.size(); i++)
            children.get(i).passTwo();

         if (isRoot())
            return;

         for (int i = 0; i < getNumberOfDoFs(); i++)
         {
            int index_i = jointIndices[i];

            TwistReadOnly unitTwist_i = getUnitTwist(i);
            tauMatrix.set(index_i, 0, computeTauElement(unitTwist_i));
            double gradient_ii = computeGravityGradientElement(unitTwist_i, unitTwist_i);
            tauGradientMatrix.set(index_i, index_i, gradient_ii);

            for (int j = 0; j < i; j++)
            {
               int index_j = jointIndices[j];
               TwistReadOnly unitTwist_j = getUnitTwist(j);

               double n_gravity_ij = computeGravityGradientElement(unitTwist_i, unitTwist_j);
               double n_gravity_ji = computeGravityGradientElement(unitTwist_j, unitTwist_i);
               double n_extWrench_ij = computeSubTreeExtWrenchGradientElement(unitTwist_i, unitTwist_j, this);
               tauGradientMatrix.set(index_i, index_j, n_gravity_ij + n_extWrench_ij);
               tauGradientMatrix.set(index_j, index_i, n_gravity_ji - n_extWrench_ij);
            }
         }

         AlgorithmStep ancestor = parent;

         while (!ancestor.isRoot())
         {
            for (int j = 0; j < getNumberOfDoFs(); j++)
            {
               int index_j = jointIndices[j];

               TwistReadOnly unitTwist_j = getUnitTwist(j);

               for (int i = 0; i < ancestor.getNumberOfDoFs(); i++)
               {
                  int index_i = ancestor.jointIndices[i];
                  unitTwist_i.setIncludingFrame(ancestor.getUnitTwist(i));
                  unitTwist_i.changeFrame(getFrameAfterJoint());

                  double gradient_ji = computeGravityGradientElement(unitTwist_j, unitTwist_i);
                  double gradient_ij = gradient_ji;
                  gradient_ji += computeSubTreeExtWrenchGradientElement(unitTwist_j, unitTwist_i, this);
                  tauGradientMatrix.set(index_i, index_j, gradient_ij);
                  tauGradientMatrix.set(index_j, index_i, gradient_ji);
               }
            }

            ancestor = ancestor.parent;
         }
      }

      private double computeTauElement(TwistReadOnly unitTwist)
      {
         FrameVector3DReadOnly velocity = unitTwist.getLinearPart();
         FrameVector3DReadOnly omega = unitTwist.getAngularPart();

         // f = f_g
         double fx = -gravityForceAtCoM.getX();
         double fy = -gravityForceAtCoM.getY();
         double fz = -gravityForceAtCoM.getZ();

         // t = x_CoM x f + t_ext
         double tx = subTreeCoM.getY() * fz - subTreeCoM.getZ() * fy - subTreeExternalSpatialForce.getAngularPartX();
         double ty = subTreeCoM.getZ() * fx - subTreeCoM.getX() * fz - subTreeExternalSpatialForce.getAngularPartY();
         double tz = subTreeCoM.getX() * fy - subTreeCoM.getY() * fx - subTreeExternalSpatialForce.getAngularPartZ();

         // f -= f_ext
         fx -= subTreeExternalSpatialForce.getLinearPartX();
         fy -= subTreeExternalSpatialForce.getLinearPartY();
         fz -= subTreeExternalSpatialForce.getLinearPartZ();

         return TupleTools.dot(tx, ty, tz, omega) + TupleTools.dot(fx, fy, fz, velocity);
      }

      private double computeGravityGradientElement(TwistReadOnly unitTwist_i, TwistReadOnly unitTwist_j)
      {
         FrameVector3DReadOnly velocity_i = unitTwist_i.getLinearPart();
         FrameVector3DReadOnly omega_i = unitTwist_i.getAngularPart();
         FrameVector3DReadOnly omega_j = unitTwist_j.getAngularPart();

         // f = f_g
         double fx = -gravityForceAtCoM.getX();
         double fy = -gravityForceAtCoM.getY();
         double fz = -gravityForceAtCoM.getZ();

         // f x w_j
         double fxDot = fy * omega_j.getZ() - fz * omega_j.getY();
         double fyDot = fz * omega_j.getX() - fx * omega_j.getZ();
         double fzDot = fx * omega_j.getY() - fy * omega_j.getX();

         // x_CoM x (f x w_j)
         double txDot = subTreeCoM.getY() * fzDot - subTreeCoM.getZ() * fyDot;
         double tyDot = subTreeCoM.getZ() * fxDot - subTreeCoM.getX() * fzDot;
         double tzDot = subTreeCoM.getX() * fyDot - subTreeCoM.getY() * fxDot;

         // (x_CoM x (f x w_j)) . w_i + (f x w_j) . v_i
         return TupleTools.dot(txDot, tyDot, tzDot, omega_i) + TupleTools.dot(fxDot, fyDot, fzDot, velocity_i);
      }

      private double computeSubTreeExtWrenchGradientElement(TwistReadOnly unitTwist_i, TwistReadOnly unitTwist_j, AlgorithmStep start_k)
      {
         if (!start_k.hasSubTreeExternalWrench)
            return 0.0; // There's no external wrench down the subtree.

         double gradientElement = 0.0;

         if (start_k.hasExternalWrench)
            gradientElement = computeSingleExtWrenchGradientElement(unitTwist_i, unitTwist_j, start_k);

         for (int i = 0; i < start_k.children.size(); i++)
         {
            AlgorithmStep child = start_k.children.get(i);
            gradientElement += computeSubTreeExtWrenchGradientElement(unitTwist_i, unitTwist_j, child);
         }
         return gradientElement;
      }

      private final SpatialVector descendantForce = new SpatialVector();
      private final FramePoint3D pointOfApplication = new FramePoint3D();

      private double computeSingleExtWrenchGradientElement(TwistReadOnly unitTwist_i, TwistReadOnly unitTwist_j, AlgorithmStep descendant_k)
      {
         // We only want the force/torque to be rotated, so we use a SpatialVector instead of SpatialForce/Wrench.
         descendantForce.setIncludingFrame(descendant_k.externalWrench);
         descendantForce.changeFrame(getFrameAfterJoint());
         pointOfApplication.setToZero(descendant_k.getBodyFixedFrame());
         pointOfApplication.changeFrame(getFrameAfterJoint());

         // f_ext_k
         double fx = -descendantForce.getLinearPartX();
         double fy = -descendantForce.getLinearPartY();
         double fz = -descendantForce.getLinearPartZ();
         // t_ext_k
         double tx = -descendantForce.getAngularPartX();
         double ty = -descendantForce.getAngularPartY();
         double tz = -descendantForce.getAngularPartZ();

         FrameVector3DReadOnly velocity_i = unitTwist_i.getLinearPart();
         FrameVector3DReadOnly velocity_j = unitTwist_j.getLinearPart();
         FrameVector3DReadOnly omega_i = unitTwist_i.getAngularPart();
         FrameVector3DReadOnly omega_j = unitTwist_j.getAngularPart();

         // d_ext_k
         double dx = pointOfApplication.getX();
         double dy = pointOfApplication.getY();
         double dz = pointOfApplication.getZ();

         // f_ext_k x w_j
         double fxDot_j = fy * omega_j.getZ() - fz * omega_j.getY();
         double fyDot_j = fz * omega_j.getX() - fx * omega_j.getZ();
         double fzDot_j = fx * omega_j.getY() - fy * omega_j.getX();
         // f_ext_k x w_i
         double fxDot_i = fy * omega_i.getZ() - fz * omega_i.getY();
         double fyDot_i = fz * omega_i.getX() - fx * omega_i.getZ();
         double fzDot_i = fx * omega_i.getY() - fy * omega_i.getX();

         // d_ext_k x (f_ext_k x w_j) + t_ext_k
         double txDot_j = dy * fzDot_j - dz * fyDot_j + ty * omega_j.getZ() - tz * omega_j.getY();
         double tyDot_j = dz * fxDot_j - dx * fzDot_j + tz * omega_j.getX() - tx * omega_j.getZ();
         double tzDot_j = dx * fyDot_j - dy * fxDot_j + tx * omega_j.getY() - ty * omega_j.getX();
         // d_ext_k x (f_ext_k x w_i)
         double txDot_i = dy * fzDot_i - dz * fyDot_i;
         double tyDot_i = dz * fxDot_i - dx * fzDot_i;
         double tzDot_i = dx * fyDot_i - dy * fxDot_i;

         //   (d_ext_k x (f_ext_k x w_j) + t_ext_k) . w_i 
         // + (f_ext_k x w_j) . v_i
         // - (d_ext_k x (f_ext_k x w_i)) . w_j
         // - (f_ext_k x w_i) . v_j
         double gradient_ijk = TupleTools.dot(txDot_j, tyDot_j, tzDot_j, omega_i);
         gradient_ijk += TupleTools.dot(fxDot_j, fyDot_j, fzDot_j, velocity_i);
         gradient_ijk -= TupleTools.dot(txDot_i, tyDot_i, tzDot_i, omega_j);
         gradient_ijk -= TupleTools.dot(fxDot_i, fyDot_i, fzDot_i, velocity_j);
         return gradient_ijk;
      }

      public boolean isRoot()
      {
         return parent == null;
      }

      public JointReadOnly getJoint()
      {
         return rigidBody.getParentJoint();
      }

      public int getNumberOfDoFs()
      {
         return getJoint().getDegreesOfFreedom();
      }

      public MovingReferenceFrame getFrameAfterJoint()
      {
         return getJoint().getFrameAfterJoint();
      }

      public TwistReadOnly getUnitTwist(int dofIndex)
      {
         return getJoint().getUnitTwists().get(dofIndex);
      }

      public MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }
   }
}