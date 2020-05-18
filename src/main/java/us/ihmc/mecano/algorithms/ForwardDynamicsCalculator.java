package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.ejml.alg.dense.misc.UnrolledInverseFromMinor;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Computed joint accelerations based on joint efforts.
 * <p>
 * This calculator is based on the articulated-body inertia algorithm as described in Featherstone -
 * Rigid Body Dynamics Algorithms (2008): <a href=
 * "https://books.google.com/books?id=GJRGBQAAQBAJ&lpg=PR5&ots=XoFXvnJZLH&dq=rigid%20body%20dynamics%20algorithms&lr&pg=PR1#v=onepage&q=rigid%20body%20dynamics%20algorithms&f=false">link</a>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class ForwardDynamicsCalculator
{
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;

   /** The root of the internal recursive algorithm. */
   private final ArticulatedBodyRecursionStep initialRecursionStep;
   /** Map to quickly retrieve information for each rigid-body. */
   private final Map<RigidBodyReadOnly, ArticulatedBodyRecursionStep> rigidBodyToRecursionStepMap = new LinkedHashMap<>();

   /** The input of this algorithm: the effort matrix for all the joints to consider. */
   private final DenseMatrix64F jointTauMatrix;
   /** The output of this algorithm: the acceleration matrix for all the joints to consider. */
   private final DenseMatrix64F jointAccelerationMatrix;

   /**
    * Extension of this algorithm into an acceleration provider that can be used instead of a
    * {@link SpatialAccelerationCalculator}.
    */
   private final RigidBodyAccelerationProvider accelerationProvider;

   /**
    * Extension of this algorithm into an acceleration provider that can be used instead of a
    * {@link SpatialAccelerationCalculator}.
    */
   private final RigidBodyAccelerationProvider zeroVelocityAccelerationProvider;

   /**
    * Creates a calculator for computing the joint accelerations for all the descendants of the given
    * {@code rootBody}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param rootBody the supporting body of the subtree to be evaluated by this calculator. Not
    *                 modified.
    */
   public ForwardDynamicsCalculator(RigidBodyReadOnly rootBody)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody));
   }

   /**
    * Creates a calculator for computing the joint accelerations for a system defined by the given
    * {@code input}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param input the definition of the system to be evaluated by this calculator.
    */
   public ForwardDynamicsCalculator(MultiBodySystemReadOnly input)
   {
      this(input, true);
   }

   /**
    * Creates a calculator for computing the joint accelerations for a system defined by the given
    * {@code input}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param input                          the definition of the system to be evaluated by this
    *                                       calculator.
    * @param considerIgnoredSubtreesInertia whether the inertia of the ignored part(s) of the given
    *                                       multi-body system should be considered. When {@code true},
    *                                       this provides more accurate joint accelerations as they
    *                                       account for instance for the gravity acting on the ignored
    *                                       rigid-bodies, i.e. bodies which have an ancestor joint that
    *                                       is ignored as specified in the given {@code input}. When
    *                                       {@code false}, the resulting joint accelerations may be
    *                                       less accurate and this calculator may gain slight
    *                                       performance improvement.
    */
   public ForwardDynamicsCalculator(MultiBodySystemReadOnly input, boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;

      RigidBodyReadOnly rootBody = input.getRootBody();
      initialRecursionStep = new ArticulatedBodyRecursionStep(rootBody, null, null);
      rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
      buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore());
      if (considerIgnoredSubtreesInertia)
         initialRecursionStep.includeIgnoredSubtreeInertia();

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      jointTauMatrix = new DenseMatrix64F(nDoFs, 1);
      jointAccelerationMatrix = new DenseMatrix64F(nDoFs, 1);

      Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> accelerationFunction = body ->
      {
         ArticulatedBodyRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);
         if (recursionStep == null)
            return null;
         // The algorithm computes the acceleration expressed in the parent joint frame.
         // To prevent unnecessary computation, let's only change the frame when needed.
         recursionStep.rigidBodyAcceleration.changeFrame(body.getBodyFixedFrame());
         return recursionStep.rigidBodyAcceleration;
      };
      accelerationProvider = RigidBodyAccelerationProvider.toRigidBodyAccelerationProvider(accelerationFunction, input.getInertialFrame());

      Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> zeroVelocityAccelerationFunction = body ->
      {
         ArticulatedBodyRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);
         if (recursionStep == null)
            return null;
         // The algorithm computes the acceleration expressed in the parent joint frame.
         // To prevent unnecessary computation, let's only change the frame when needed.
         recursionStep.rigidBodyZeroVelocityAcceleration.changeFrame(body.getBodyFixedFrame());
         return recursionStep.rigidBodyZeroVelocityAcceleration;
      };
      zeroVelocityAccelerationProvider = RigidBodyAccelerationProvider.toRigidBodyAccelerationProvider(zeroVelocityAccelerationFunction,
                                                                                                       input.getInertialFrame(),
                                                                                                       false,
                                                                                                       true);
   }

   private void buildMultiBodyTree(ArticulatedBodyRecursionStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      for (JointReadOnly childJoint : parent.rigidBody.getChildrenJoints())
      {
         if (jointsToIgnore.contains(childJoint))
            continue;

         RigidBodyReadOnly childBody = childJoint.getSuccessor();
         if (childBody != null)
         {
            int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
            ArticulatedBodyRecursionStep child = new ArticulatedBodyRecursionStep(childBody, parent, jointIndices);
            rigidBodyToRecursionStepMap.put(childBody, child);
            buildMultiBodyTree(child, jointsToIgnore);
         }
      }
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
      gravity.checkReferenceFrameMatch(input.getInertialFrame());
      setGravitionalAcceleration((Tuple3DReadOnly) gravity);
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
      SpatialAcceleration rootAcceleration = initialRecursionStep.rigidBodyAcceleration;
      rootAcceleration.setToZero();
      rootAcceleration.getLinearPart().setAndNegate(gravity);
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
      SpatialAcceleration rootAcceleration = initialRecursionStep.rigidBodyAcceleration;
      rootAcceleration.setToZero();
      rootAcceleration.getLinearPart().set(gravityX, gravityY, gravityZ);
      rootAcceleration.negate();
   }

   /**
    * Changes the spatial acceleration of the root. Even though the root is assumed to be non-moving,
    * the {@code rootAcceleration} is usually set to the opposite of the gravitational acceleration,
    * such that the effect of the gravity is naturally propagated to the entire system.
    *
    * @param newRootAcceleration the new spatial acceleration of the root.
    * @throws ReferenceFrameMismatchException if any of the reference frames of
    *                                         {@code newRootAcceleration} does not match this
    *                                         calculator's root spatial acceleration's frames.
    */
   public void setRootAcceleration(SpatialAccelerationReadOnly newRootAcceleration)
   {
      initialRecursionStep.rigidBodyAcceleration.set(newRootAcceleration);
   }

   /**
    * Resets all the external wrenches that were added to the rigid-bodies.
    */
   public void setExternalWrenchesToZero()
   {
      initialRecursionStep.setExternalWrenchToZeroRecursive();
   }

   /**
    * Gets the internal reference to the external wrench associated with the given rigidBody.
    * <p>
    * Modify the return wrench to configure the wrench to be applied on this rigid-body.
    * </p>
    *
    * @param rigidBody the query. Not modified.
    * @return the wrench associated to the query.
    */
   public FixedFrameWrenchBasics getExternalWrench(RigidBodyReadOnly rigidBody)
   {
      ArticulatedBodyRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(rigidBody);
      return recursionStep == null ? null : recursionStep.externalWrench;
   }

   /**
    * Sets external wrench to apply to the given {@code rigidBody}.
    *
    * @param rigidBody      the rigid-body to which the wrench is to applied. Not modified.
    * @param externalWrench the external wrench to apply to the rigid-body.
    */
   public void setExternalWrench(RigidBodyReadOnly rigidBody, WrenchReadOnly externalWrench)
   {
      rigidBodyToRecursionStepMap.get(rigidBody).externalWrench.setMatchingFrame(externalWrench);
   }

   /**
    * Computes the joint accelerations resulting from the joint efforts.
    * <p>
    * The desired joint efforts are extracted from the joint state. To explicitly specify the joint
    * efforts to use, see {@link #compute(DenseMatrix64F)}.
    * </p>
    */
   public void compute()
   {
      compute(null);
   }

   /**
    * Computes the joint accelerations resulting from the given joint efforts.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    *
    * @param jointTauMatrix the matrix containing the joint efforts to use. Not modified.
    */
   public void compute(DenseMatrix64F jointTauMatrix)
   {
      if (jointTauMatrix != null)
      {
         this.jointTauMatrix.set(jointTauMatrix);
      }
      else
      {
         List<? extends JointReadOnly> indexedJointsInOrder = input.getJointMatrixIndexProvider().getIndexedJointsInOrder();
         MultiBodySystemTools.extractJointsState(indexedJointsInOrder, JointStateType.EFFORT, this.jointTauMatrix);
      }

      initialRecursionStep.passOne();
      initialRecursionStep.passTwo();
      initialRecursionStep.passThree();
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
    * Gets the computed joint accelerations.
    *
    * @return this calculator output: the joint accelerations.
    */
   public DenseMatrix64F getJointAccelerationMatrix()
   {
      return jointAccelerationMatrix;
   }

   /**
    * Gets the computed N-by-1 acceleration vector for the given {@code joint}, where N is the number
    * of degrees of freedom the joint has.
    *
    * @param joint the joint to get the acceleration of. Not modified.
    * @return the computed joint acceleration matrix.
    */
   public DenseMatrix64F getComputedJointAcceleration(JointReadOnly joint)
   {
      ArticulatedBodyRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());

      if (recursionStep == null)
         return null;
      else
         return recursionStep.qdd;
   }

   /**
    * Gets the internal recursion step for the root body.
    *
    * @return the root body recursion step.
    */
   ArticulatedBodyRecursionStep getInitialRecursionStep()
   {
      return initialRecursionStep;
   }

   /**
    * Writes the computed joint accelerations into the given {@code joints}.
    * <p>
    * Any joint that is not considered by this calculator remains unchanged.
    * </p>
    *
    * @param joints the array of joints to write the acceleration into. Modified.
    */
   public void writeComputedJointAccelerations(JointBasics[] joints)
   {
      for (JointBasics joint : joints)
         writeComputedJointAcceleration(joint);
   }

   /**
    * Writes the computed joint accelerations into the given {@code joints}.
    * <p>
    * Any joint that is not considered by this calculator remains unchanged.
    * </p>
    *
    * @param joints the list of joints to write the acceleration into. Modified.
    */
   public void writeComputedJointAccelerations(List<? extends JointBasics> joints)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
         writeComputedJointAcceleration(joints.get(jointIndex));
   }

   /**
    * Writes the computed acceleration into the given {@code joint}.
    * <p>
    * Any joint that is not considered by this calculator remains unchanged.
    * </p>
    *
    * @param joint the joint to retrieve the acceleration of and to store it. Modified.
    * @return whether the calculator handles the given joint or not.
    */
   public boolean writeComputedJointAcceleration(JointBasics joint)
   {
      ArticulatedBodyRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());

      if (recursionStep == null)
         return false;

      joint.setJointAcceleration(0, recursionStep.qdd);
      return true;
   }

   /**
    * Gets the rigid-body acceleration provider that uses accelerations computed in this calculator.
    *
    * @return the acceleration provider backed by this calculator.
    */
   public RigidBodyAccelerationProvider getAccelerationProvider()
   {
      return accelerationProvider;
   }

   /**
    * Gets the rigid-body acceleration provider that uses accelerations computed in this calculator.
    *
    * @param considerVelocities whether the provider should consider bias accelerations, i.e.
    *                           centrifugal and Coriolis accelerations, resulting from joint
    *                           velocities.
    * @return the acceleration provider backed by this calculator.
    */
   public RigidBodyAccelerationProvider getAccelerationProvider(boolean considerVelocities)
   {
      return considerVelocities ? accelerationProvider : zeroVelocityAccelerationProvider;
   }

   /**
    * Represents a single recursion step for any of the three passes of the articulated-body algorithm
    * as introduced in R. Featherstone, <i>Rigid Body Dynamics Algorithms</i>.
    *
    * @author Sylvain Bertrand
    */
   class ArticulatedBodyRecursionStep
   {
      /**
       * The rigid-body for which this recursion is.
       */
      final RigidBodyReadOnly rigidBody;
      /**
       * Body inertia: usually equal to {@code rigidBody.getInertial()}. However, if at least one child of
       * {@code rigidBody} is ignored, it is equal to this rigid-body inertia and the subtree inertia
       * attached to the ignored joint.
       */
      final SpatialInertia bodyInertia;
      /**
       * User input: external wrench to be applied to this body.
       */
      final Wrench externalWrench;
      /**
       * Coriolis acceleration.
       */
      final SpatialAcceleration biasAcceleration;
      /**
       * Bias wrench containing external and Coriolis forces applied to this body.
       */
      final Wrench biasWrench;

      /**
       * Intermediate result for faster transformation from body-fixed frame to joint frame.
       */
      final SpatialInertia spatialInertia;
      /**
       * Articulated-body inertia for this joint.
       */
      final ArticulatedBodyInertia articulatedInertia;
      /**
       * Apparent bias wrench to this joint.
       */
      final SpatialForce articulatedBiasWrench;
      /**
       * Pre-transformed articulated-body inertia for the parent.
       */
      final ArticulatedBodyInertia articulatedInertiaForParent;
      /**
       * Pre-transformed bias wrench for the parent.
       */
      final SpatialForce articulatedBiasWrenchForParent;
      /**
       * Spatial acceleration of this rigid-body.
       */
      final SpatialAcceleration rigidBodyAcceleration = new SpatialAcceleration();
      /**
       * Spatial acceleration of this rigid-body ignoring joint velocities.
       */
      final SpatialAcceleration rigidBodyZeroVelocityAcceleration = new SpatialAcceleration();
      /**
       * <tt>IA</tt> is the 6-by-6 articulated-body inertia for this body.
       */
      final DenseMatrix64F IA;
      /**
       * <tt>S</tt> is the 6-by-N matrix representing the motion subspace of the parent joint, where N is
       * the number of DoFs of the joint.
       */
      final DenseMatrix64F S;

      /**
       * Intermediate result to save operations:
       *
       * <pre>
       * U = I<sup>A</sup> S
       * </pre>
       *
       * where <tt>I<sup>A</sup></tt> is this handle's articulated-body inertia, and <tt>S</tt> is the
       * parent joint motion subspace.
       */
      final DenseMatrix64F U;
      /**
       * Intermediate result to save operations:
       *
       * <pre>
       * D = S<sup>T</sup> U
       *   = S<sup>T</sup> I<sup>A</sup> S
       * </pre>
       *
       * where <tt>I<sup>A</sup></tt> is this handle's articulated-body inertia, and <tt>S</tt> is the
       * parent joint motion subspace.
       */
      final DenseMatrix64F D;
      /**
       * Intermediate result to save operations:
       *
       * <pre>
       * D<sup>-1</sup> = ( S<sup>T</sup> U )<sup>-1</sup>
       *  <sub>  </sub> = ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup>
       * </pre>
       *
       * where <tt>I<sup>A</sup></tt> is this handle's articulated-body inertia, and <tt>S</tt> is the
       * parent joint motion subspace.
       */
      final DenseMatrix64F Dinv;
      /**
       * Intermediate result to save operations:
       *
       * <pre>
       * U D<sup>-1</sup> = U ( S<sup>T</sup> U )<sup>-1</sup>
       *    <sub>  </sub> = I<sup>A</sup> S ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup>
       * </pre>
       *
       * where <tt>I<sup>A</sup></tt> is this handle's articulated-body inertia, and <tt>S</tt> is the
       * parent joint motion subspace.
       */
      final DenseMatrix64F U_Dinv;
      /**
       * Intermediate result:
       *
       * <pre>
       * U D<sup>-1</sup> U<sup>T</sup> = U ( S<sup>T</sup> U )<sup>-1</sup> U<sup>T</sup>
       *      <sub>   </sub> = I<sup>A</sup> S ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup> S<sup>T</sup> I<sup>A</sup>
       * </pre>
       *
       * where <tt>I<sup>A</sup></tt> is this handle's articulated-body inertia, and <tt>S</tt> is the
       * parent joint motion subspace.
       */
      final DenseMatrix64F U_Dinv_UT;
      /**
       * This is the apparent articulated rigid-body inertia for the parent:
       *
       * <pre>
       * I<sup>a</sup> = I<sup>A</sup> - U D<sup>-1</sup> U<sup>T</sup>
       *  <sup> </sup> = I<sup>A</sup> - U ( S<sup>T</sup> U )<sup>-1</sup> U<sup>T</sup>
       *  <sup> </sup> = I<sup>A</sup> - I<sup>A</sup> S ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup> S<sup>T</sup> I<sup>A</sup>
       * </pre>
       *
       * where <tt>I<sup>A</sup></tt> is this handle's articulated-body inertia, and <tt>S</tt> is the
       * parent joint motion subspace.
       */
      final DenseMatrix64F Ia;
      /**
       * This is the N-by-1 vector representing the joint effort, where N is equal to the number of DoFs
       * that the joint has.
       */
      final DenseMatrix64F tau;
      /**
       * This is some bias force for this joint:
       *
       * <pre>
       * p<sup>A</sup> = p + &sum;<sub>&forall;child</sub> p<sup>a</sup>
       * </pre>
       */
      final DenseMatrix64F pA;
      /**
       * Intermediate result to save computation:
       *
       * <pre>
       * u = &tau; - S<sup>T</sup> p<sup>A</sup>
       * </pre>
       *
       * where <tt>&tau;</tt> is the N-by-1 vector representing the joint effort, N being the number of
       * DoFs for this joint, <tt>S</tt> is the joint motion subspace, and <tt>p<sup>A</sup></tt> some
       * bias forces exerted on this joint.
       */
      final DenseMatrix64F u;
      /**
       * Bias acceleration for this joint:
       *
       * <pre>
       * c = v &times; ( S qDot )
       * </pre>
       *
       * where <tt>v</tt> is the twist of this body, <tt>S</tt> the joint motion subspace, and
       * <tt>qDot</tt> the N-by-1 vector for this joint velocity, with N being the number of DoFs for this
       * joint.
       */
      final DenseMatrix64F c;
      /**
       * The apparent bias forces of this joint for the parent:
       *
       * <pre>
       * p<sup>a</sup> = p<sup>A</sup> + I<sup>a</sup> c + U D<sup>-1</sup> u
       *  <sup> </sup> = p<sup>A</sup> + I<sup>a</sup> c + I<sup>A</sup> S ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup> ( &tau; - S<sup>T</sup> p<sup>A</sup> )
       * </pre>
       *
       * where <tt>p<sup>A</sup></tt> are the bias forces acting on this joint, <tt>I<sup>a</sup></tt> is
       * the apparent articulated-body inertia for the parent, <tt>S</tt> is the joint motion subspace,
       * <tt>I<sup>A</sup></tt> this handle's articulated-body inertia, <tt>&tau;</tt> this joint effort,
       * <tt>c</tt> the bias acceleration for this joint.
       */
      final DenseMatrix64F pa;
      /**
       * Intermediate result to save computation:
       *
       * <pre>
       * a' = a<sub>parent</sub> + c
       * </pre>
       */
      final DenseMatrix64F aPrime;
      /**
       * This body acceleration:
       *
       * <pre>
       * a = a' + S qDDot
       *   = a<sub>parent</sub> + c + S qDDot
       * </pre>
       */
      final DenseMatrix64F a;
      /**
       * Intermediate result for garbage-free operation.
       */
      final DenseMatrix64F qdd_intermediate;
      /**
       * <b>This the output of this algorithm: the joint acceleration:</b>
       *
       * <pre>
       * qDDot = D<sup>-1</sup> ( u - U<sup>T</sup> a' )
       *       = D<sup>-1</sup> ( &tau; - U<sup>T</sup> ( a<sub>parent</sub> + c + S qDDot ) - S<sup>T</sup> p<sup>A</sup> )
       *       = ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup> ( &tau; - ( I<sup>A</sup> S )<sup>T</sup> ( a<sub>parent</sub> + c + S qDDot ) - S<sup>T</sup> p<sup>A</sup> )
       *       = ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup> ( &tau; - S<sup>T</sup> I<sup>A</sup> ( a<sub>parent</sub> + c + S qDDot ) - S<sup>T</sup> p<sup>A</sup> )
       * </pre>
       */
      final DenseMatrix64F qdd;
      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid-body.
       */
      final ArticulatedBodyRecursionStep parent;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      final List<ArticulatedBodyRecursionStep> children = new ArrayList<>();
      /**
       * Solver for inverting <tt>D</tt>. Only needed for 6-DoF joints.
       */
      final LinearSolver<DenseMatrix64F> inverseSolver;
      /**
       * Intermediate variable to save computation. Transform from {@code this.getFrameAfterJoint()} to
       * {@code parent.getFrameAfterJoint()}.
       */
      final RigidBodyTransform transformToParentJointFrame;
      /**
       * Joint indices for storing {@code qdd} in the main matrix {@code jointAccelerationMatrix}.
       */
      final int[] jointIndices;

      private ArticulatedBodyRecursionStep(RigidBodyReadOnly rigidBody, ArticulatedBodyRecursionStep parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;

         externalWrench = new Wrench(getBodyFixedFrame(), getBodyFixedFrame());

         if (parent == null)
         {
            bodyInertia = null;
            biasAcceleration = null;
            biasWrench = null;
            spatialInertia = null;
            articulatedInertia = null;
            articulatedBiasWrench = null;
            articulatedInertiaForParent = null;
            articulatedBiasWrenchForParent = null;
            rigidBodyAcceleration.setToZero(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());
            rigidBodyZeroVelocityAcceleration.setToZero(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());

            IA = null;
            S = null;
            U = null;
            D = null;
            Dinv = null;
            U_Dinv = null;
            U_Dinv_UT = null;
            tau = null;
            pA = null;
            u = null;
            c = null;
            pa = null;
            Ia = null;
            qdd = null;
            qdd_intermediate = null;
            aPrime = null;
            a = null;
            inverseSolver = null;
            transformToParentJointFrame = null;
         }
         else
         {
            parent.children.add(this);
            int nDoFs = getJoint().getDegreesOfFreedom();

            bodyInertia = new SpatialInertia(rigidBody.getInertia());
            biasAcceleration = new SpatialAcceleration();
            biasWrench = new Wrench();
            spatialInertia = new SpatialInertia();
            articulatedInertia = new ArticulatedBodyInertia();
            articulatedBiasWrench = new SpatialForce();
            articulatedInertiaForParent = parent.isRoot() ? null : new ArticulatedBodyInertia();
            articulatedBiasWrenchForParent = parent.isRoot() ? null : new SpatialForce();

            IA = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, SpatialVectorReadOnly.SIZE);
            S = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, nDoFs);
            U = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, nDoFs);
            D = new DenseMatrix64F(nDoFs, nDoFs);
            Dinv = new DenseMatrix64F(nDoFs, nDoFs);
            U_Dinv = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, nDoFs);
            U_Dinv_UT = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, SpatialVectorReadOnly.SIZE);
            tau = new DenseMatrix64F(nDoFs, 1);
            pA = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
            u = new DenseMatrix64F(nDoFs, 1);
            c = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
            pa = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
            Ia = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, SpatialVectorReadOnly.SIZE);
            qdd = new DenseMatrix64F(nDoFs, 1);
            qdd_intermediate = new DenseMatrix64F(nDoFs, 1);
            aPrime = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
            a = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
            inverseSolver = nDoFs == 6 ? LinearSolverFactory.symmPosDef(6) : null;
            transformToParentJointFrame = new RigidBodyTransform();
            getJoint().getMotionSubspace(S);
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
                  subtreeIneria.changeFrame(getBodyFixedFrame());
                  bodyInertia.add(subtreeIneria);
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).includeIgnoredSubtreeInertia();
      }

      /**
       * The first pass consists in calculating the bias wrench resulting from external and Coriolis
       * forces, and the bias acceleration resulting from the Coriolis acceleration.
       */
      public void passOne()
      {
         if (isRoot())
         {
            // Need to set the zero-velocity acceleration to account for gravity.
            rigidBodyZeroVelocityAcceleration.setIncludingFrame(rigidBodyAcceleration);
         }
         else
         {
            MovingReferenceFrame frameAfterJoint = getFrameAfterJoint();
            MovingReferenceFrame frameBeforeJoint = getJoint().getFrameBeforeJoint();
            if (parent.isRoot())
               frameAfterJoint.getTransformToDesiredFrame(transformToParentJointFrame, parent.getBodyFixedFrame());
            else
               frameAfterJoint.getTransformToDesiredFrame(transformToParentJointFrame, parent.getFrameAfterJoint());

            bodyInertia.computeDynamicWrench(null, getBodyTwist(), biasWrench);
            biasWrench.sub(externalWrench);
            biasWrench.changeFrame(frameAfterJoint);

            biasAcceleration.setToZero(frameAfterJoint, input.getInertialFrame(), frameBeforeJoint);
            biasAcceleration.changeFrame(frameAfterJoint, getJoint().getJointTwist(), frameAfterJoint.getTwistOfFrame());
            biasAcceleration.get(c);
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).passOne();
      }

      /**
       * The second two calculates the articulated-body inertia and the bias wrench that includes
       * Coriolis, external, and joint forces.
       * <p>
       * This pass also computes several intermediate variables to reduce the number of calculations.
       * </p>
       */
      public void passTwo()
      {
         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).passTwo();

         if (isRoot())
            return;

         MovingReferenceFrame frameAfterJoint = getFrameAfterJoint();

         spatialInertia.setIncludingFrame(bodyInertia);
         spatialInertia.changeFrame(frameAfterJoint);
         articulatedInertia.setIncludingFrame(spatialInertia);

         articulatedBiasWrench.setIncludingFrame(biasWrench);
         articulatedBiasWrench.changeFrame(frameAfterJoint);

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            ArticulatedBodyRecursionStep child = children.get(childIndex);
            child.articulatedInertiaForParent.applyTransform(child.transformToParentJointFrame);
            child.articulatedBiasWrenchForParent.applyTransform(child.transformToParentJointFrame);
            child.articulatedInertiaForParent.setReferenceFrame(frameAfterJoint);
            child.articulatedBiasWrenchForParent.setReferenceFrame(frameAfterJoint);

            articulatedInertia.add(child.articulatedInertiaForParent);
            articulatedBiasWrench.add(child.articulatedBiasWrenchForParent);
         }

         // Computing intermediate variables used in later calculation
         articulatedInertia.get(IA);
         CommonOps.mult(IA, S, U);
         CommonOps.multTransA(S, U, D);

         int nDoFs = getJoint().getDegreesOfFreedom();
         if (nDoFs == 1)
         {
            Dinv.set(0, 1.0 / D.get(0));
         }
         else if (nDoFs == 0)
         {
            Dinv.reshape(0, 0);
         }
         else if (nDoFs <= 5)
         {
            UnrolledInverseFromMinor.inv(D, Dinv);
         }
         else
         {
            inverseSolver.setA(D);
            inverseSolver.invert(Dinv);
         }

         // Computing u_i = tau_i - S_i^T * p_i^A
         for (int dofIndex = 0; dofIndex < nDoFs; dofIndex++)
         {
            tau.set(dofIndex, 0, jointTauMatrix.get(jointIndices[dofIndex], 0));
         }

         articulatedBiasWrench.get(pA);
         CommonOps.multTransA(-1.0, S, pA, u);
         CommonOps.addEquals(u, tau);

         if (!parent.isRoot())
         {
            CommonOps.mult(U, Dinv, U_Dinv);
            CommonOps.multTransB(U_Dinv, U, U_Dinv_UT);

            // Computing I_i^a = I_i^A - U_i * D_i^-1 * U_i^T
            articulatedInertiaForParent.setIncludingFrame(articulatedInertia);
            articulatedInertiaForParent.sub(U_Dinv_UT);
            articulatedInertiaForParent.get(Ia);

            // Computing p_i^a = p_i^A + I_i^a * c_i + U_i * D_i^-1 * u_i
            articulatedBiasWrench.get(pa);
            CommonOps.multAdd(Ia, c, pa);
            CommonOps.multAdd(U_Dinv, u, pa);
            articulatedBiasWrenchForParent.setIncludingFrame(frameAfterJoint, pa);
         }
      }

      /**
       * The third and last pass calculates the joint acceleration and body spatial acceleration.
       */
      public void passThree()
      {
         if (!isRoot())
         {
            rigidBodyZeroVelocityAcceleration.setIncludingFrame(parent.rigidBodyZeroVelocityAcceleration);
            rigidBodyZeroVelocityAcceleration.applyInverseTransform(transformToParentJointFrame);
            rigidBodyZeroVelocityAcceleration.setBodyFrame(getBodyFixedFrame());
            rigidBodyZeroVelocityAcceleration.setBaseFrame(input.getInertialFrame());
            rigidBodyZeroVelocityAcceleration.setReferenceFrame(getFrameAfterJoint());

            // Computing a'_i = a_{lambda(i)} + c_i
            rigidBodyAcceleration.setIncludingFrame(parent.rigidBodyAcceleration);
            rigidBodyAcceleration.applyInverseTransform(transformToParentJointFrame);
            rigidBodyAcceleration.setReferenceFrame(getFrameAfterJoint());
            rigidBodyAcceleration.add((SpatialVectorReadOnly) biasAcceleration);
            rigidBodyAcceleration.get(aPrime);

            // Computing qdd_i = D_i^-1 * ( u_i - U_i^T * a'_i )
            CommonOps.multTransA(-1.0, U, aPrime, qdd_intermediate);
            CommonOps.addEquals(qdd_intermediate, u);
            CommonOps.mult(Dinv, qdd_intermediate, qdd);

            // Computing a_i = a'_i + S_i * qdd_i
            CommonOps.mult(S, qdd, a);

            rigidBodyZeroVelocityAcceleration.add(a);

            CommonOps.addEquals(a, aPrime);
            rigidBodyAcceleration.setIncludingFrame(getBodyFixedFrame(), input.getInertialFrame(), getFrameAfterJoint(), a);

            for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
            {
               jointAccelerationMatrix.set(jointIndices[dofIndex], 0, qdd.get(dofIndex, 0));
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).passThree();
      }

      public void setExternalWrenchToZeroRecursive()
      {
         if (externalWrench != null)
            externalWrench.setToZero();

         for (int i = 0; i < children.size(); i++)
            children.get(i).setExternalWrenchToZeroRecursive();
      }

      public boolean isRoot()
      {
         return parent == null;
      }

      public MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }

      public MovingReferenceFrame getFrameAfterJoint()
      {
         return getJoint().getFrameAfterJoint();
      }

      public JointReadOnly getJoint()
      {
         return rigidBody.getParentJoint();
      }

      public TwistReadOnly getBodyTwist()
      {
         return getBodyFixedFrame().getTwistOfFrame();
      }

      @Override
      public String toString()
      {
         return "RigidBody: " + rigidBody + ", parent: " + parent.rigidBody + ", children: " + Arrays.asList(children.stream().map(c -> c.rigidBody).toArray());
      }
   }
}
