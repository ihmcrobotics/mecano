package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class MultiBodyGravityGradientCalculator
{
   private final MultiBodySystemReadOnly input;
   private final DMatrixRMaj gravityMatrix;
   private final DMatrixRMaj gravityGradientMatrix;
   private final AlgorithmStep initialStep;
   private final FrameVector3D gravitationalAcceleration = new FrameVector3D();

   private boolean dirtyFlag = true;

   public MultiBodyGravityGradientCalculator(MultiBodySystemReadOnly input)
   {
      this.input = input;

      gravityGradientMatrix = new DMatrixRMaj(input.getNumberOfDoFs(), input.getNumberOfDoFs());
      gravityMatrix = new DMatrixRMaj(input.getNumberOfDoFs(), 1);

      initialStep = new AlgorithmStep(input.getRootBody(), null, null);
      buildMultiBodyTree(initialStep, input.getJointsToIgnore());
      initialStep.includeIgnoredSubtreeInertia();

      gravitationalAcceleration.setToZero(input.getInertialFrame());
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

   private List<AlgorithmStep> buildMultiBodyTree(AlgorithmStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<AlgorithmStep> algorithmSteps = new ArrayList<>();
      algorithmSteps.add(parent);

      List<JointReadOnly> childrenJoints = new ArrayList<>(parent.rigidBody.getChildrenJoints());

      if (childrenJoints.size() > 1)
      { // Reorganize the joints in the children to ensure that loop closures are treated last.
         List<JointReadOnly> loopClosureAncestors = new ArrayList<>();

         for (int i = 0; i < childrenJoints.size();)
         {
            if (MultiBodySystemTools.doesSubtreeContainLoopClosure(childrenJoints.get(i).getSuccessor()))
               loopClosureAncestors.add(childrenJoints.remove(i));
            else
               i++;
         }

         childrenJoints.addAll(loopClosureAncestors);
      }

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
         if (childBody != null)
         {
            int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
            AlgorithmStep child = new AlgorithmStep(childBody, parent, jointIndices);
            algorithmSteps.addAll(buildMultiBodyTree(child, jointsToIgnore));
         }
      }

      return algorithmSteps;
   }

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

   public DMatrixRMaj getGravityMatrix()
   {
      update();
      return gravityMatrix;
   }

   public DMatrixRMaj getGravityGradientMatrix()
   {
      update();
      return gravityGradientMatrix;
   }

   private class AlgorithmStep
   {
      private final RigidBodyReadOnly rigidBody;
      private final SpatialInertia bodyInertia;
      private final AlgorithmStep parent;
      private final int[] jointIndices;
      private final List<AlgorithmStep> children = new ArrayList<>();

      private double subTreeMass;
      private final FramePoint3D centerOfMass = new FramePoint3D();

      private final FramePoint3D childCoM = new FramePoint3D();
      private final FrameVector3D gravity = new FrameVector3D();

      public AlgorithmStep(RigidBodyReadOnly rigidBody, AlgorithmStep parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;

         if (isRoot())
         {
            bodyInertia = null;
         }
         else
         {
            parent.children.add(this);
            bodyInertia = new SpatialInertia(rigidBody.getInertia());
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

      private final Twist unitTwist_i = new Twist();

      public void passOne()
      {
         for (int i = 0; i < children.size(); i++)
            children.get(i).passOne();

         // Update the subtree mass
         subTreeMass = bodyInertia == null ? 0.0 : bodyInertia.getMass();
         for (int i = 0; i < children.size(); i++)
            subTreeMass += children.get(i).subTreeMass;

         ReferenceFrame frameToUse = isRoot() ? getBodyFixedFrame() : getFrameAfterJoint();
         gravity.setIncludingFrame(gravitationalAcceleration);
         gravity.changeFrame(frameToUse);

         // The centerOfMassTimesMass can be used to obtain the overall center of mass position.
         if (bodyInertia == null)
         {
            centerOfMass.setToZero(getBodyFixedFrame());
         }
         else
         {
            centerOfMass.setIncludingFrame(bodyInertia.getCenterOfMassOffset());
            centerOfMass.changeFrame(frameToUse);
            centerOfMass.scale(bodyInertia.getMass());
         }

         for (int i = 0; i < children.size(); i++)
         {
            childCoM.setIncludingFrame(children.get(i).centerOfMass);
            childCoM.changeFrame(frameToUse);
            centerOfMass.scaleAdd(children.get(i).subTreeMass, childCoM, centerOfMass);
         }

         centerOfMass.scale(1.0 / subTreeMass);
      }

      /** From leaves to root, compute the elements of the gradient matrix. */
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
            gravityMatrix.set(index_i, 0, computeGravityElement(unitTwist_i));
            gravityGradientMatrix.set(index_i, index_i, computeGravityGradientElement(unitTwist_i, unitTwist_i));

            for (int j = 0; j < i; j++)
            {
               int index_j = jointIndices[j];
               TwistReadOnly unitTwist_j = getUnitTwist(j);

               gravityGradientMatrix.set(index_i, index_j, computeGravityGradientElement(unitTwist_i, unitTwist_j));
               gravityGradientMatrix.set(index_j, index_i, computeGravityGradientElement(unitTwist_j, unitTwist_i));
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
                  gravityGradientMatrix.set(index_i, index_j, gradient_ji);
                  gravityGradientMatrix.set(index_j, index_i, gradient_ji);
               }
            }

            ancestor = ancestor.parent;
         }
      }

      private double computeGravityElement(TwistReadOnly unitTwist)
      {
         FrameVector3DReadOnly velocity = unitTwist.getLinearPart();
         FrameVector3DReadOnly omega = unitTwist.getAngularPart();

         // g x x_CoM
         double tx = gravity.getY() * centerOfMass.getZ() - gravity.getZ() * centerOfMass.getY();
         double ty = gravity.getZ() * centerOfMass.getX() - gravity.getX() * centerOfMass.getZ();
         double tz = gravity.getX() * centerOfMass.getY() - gravity.getY() * centerOfMass.getX();

         return subTreeMass * (TupleTools.dot(tx, ty, tz, omega) - gravity.dot((Vector3DReadOnly) velocity));
      }

      private double computeGravityGradientElement(TwistReadOnly unitTwist_i, TwistReadOnly unitTwist_j)
      {
         FrameVector3DReadOnly velocity_i = unitTwist_i.getLinearPart();
         FrameVector3DReadOnly omega_i = unitTwist_i.getAngularPart();
         FrameVector3DReadOnly omega_j = unitTwist_j.getAngularPart();

         // w_j x g
         double fxDot = omega_j.getY() * gravity.getZ() - omega_j.getZ() * gravity.getY();
         double fyDot = omega_j.getZ() * gravity.getX() - omega_j.getX() * gravity.getZ();
         double fzDot = omega_j.getX() * gravity.getY() - omega_j.getY() * gravity.getX();
         // (w_j x g) x x_CoM
         double txDot = fyDot * centerOfMass.getZ() - fzDot * centerOfMass.getY();
         double tyDot = fzDot * centerOfMass.getX() - fxDot * centerOfMass.getZ();
         double tzDot = fxDot * centerOfMass.getY() - fyDot * centerOfMass.getX();

         // m_subtree * (((w_j x g) x x_CoM) . w_i - (w_j x g) . v_i)
         return subTreeMass * (-TupleTools.dot(txDot, tyDot, tzDot, omega_i) + TupleTools.dot(fxDot, fyDot, fzDot, velocity_i));
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