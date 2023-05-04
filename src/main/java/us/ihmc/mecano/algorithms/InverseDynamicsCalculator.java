package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.*;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Computes joint efforts based on joint accelerations.
 * <p>
 * This calculator uses a recursive Newton-Euler algorithm, as described in Featherstone - Rigid
 * Body Dynamics Algorithms (2008): <a href=
 * "https://books.google.com/books?id=GJRGBQAAQBAJ&lpg=PR5&ots=XoFXvnJZLH&dq=rigid%20body%20dynamics%20algorithms&lr&pg=PR1#v=onepage&q=rigid%20body%20dynamics%20algorithms&f=false">link</a>
 * </p>
 * <p>
 * In the presence of kinematic loops, this implementation will set the effort for the joint at loop
 * closure to zero. The efforts of the joints that compose the loop can then be calculated
 * externally knowing that the secondary branch, i.e. the branch of the loop that ends with the loop
 * closure joint, does not account for the effort due to the subtree following the loop.
 * </p>
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public class InverseDynamicsCalculator
{
   private static final boolean DEFAULT_CONSIDER_ACCELERATIONS = true;
   private static final boolean DEFAULT_CONSIDER_CORIOLIS = true;

   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;
   /** The root of the internal recursive algorithm. */
   final RecursionStep initialRecursionStep;
   /** Map to quickly retrieve information for each rigid-body. */
   final Map<RigidBodyReadOnly, RecursionStep> rigidBodyToRecursionStepMap = new LinkedHashMap<>();
   /** Map to quickly retrieve information for each joint. */
   private final Map<JointReadOnly, RecursionStepBasics> jointToRecursionStepMap = new LinkedHashMap<>();

   /** The input of this algorithm: the acceleration matrix for all the joints to consider. */
   private final DMatrixRMaj allJointAccelerationMatrix;
   /** The output of this algorithm: the effort matrix for all the joints to consider. */
   private final DMatrixRMaj allJointTauMatrix;

   /** Whether the effort resulting from the joint accelerations should be considered. */
   private boolean considerJointAccelerations = DEFAULT_CONSIDER_ACCELERATIONS;
   /** Whether the effort resulting from the Coriolis and centrifugal forces should be considered. */
   private boolean considerCoriolisAndCentrifugalForces = DEFAULT_CONSIDER_CORIOLIS;
   /**
    * Extension of this algorithm into an acceleration provider that be used instead of a
    * {@link SpatialAccelerationCalculator}.
    */
   private final RigidBodyAccelerationProvider accelerationProvider;

   /**
    * Creates a calculator for computing the joint efforts for all the descendants of the given
    * {@code rootBody}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param rootBody                             the supporting body of the subtree to be evaluated by
    *                                             this calculator. Not modified.
    * @param considerCoriolisAndCentrifugalForces whether the effort resulting from the Coriolis and
    *                                             centrifugal forces should be considered.
    * @param considerJointAccelerations           whether the effort resulting from the joint
    *                                             accelerations should be considered.
    * @deprecated Use the following code snippet instead:
    *
    *             <pre>
    *             InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(rootBody);
    *             calculator.setConsiderCoriolisAndCentrifugalForces(considerCoriolisAndCentrifugalForces);
    *             calculator.setConsiderJointAccelerations(considerJointAccelerations);
    *             </pre>
    */
   @Deprecated
   public InverseDynamicsCalculator(RigidBodyReadOnly rootBody, boolean considerCoriolisAndCentrifugalForces, boolean considerJointAccelerations)
   {
      this(rootBody);
      setConsiderCoriolisAndCentrifugalForces(considerCoriolisAndCentrifugalForces);
      setConsiderJointAccelerations(considerJointAccelerations);
   }

   /**
    * Creates a calculator for computing the joint efforts for system defined by the given
    * {@code input}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param input                                the definition of the system to be evaluated by this
    *                                             calculator.
    * @param considerCoriolisAndCentrifugalForces whether the effort resulting from the Coriolis and
    *                                             centrifugal forces should be considered.
    * @param considerJointAccelerations           whether the effort resulting from the joint
    *                                             accelerations should be considered.
    * @deprecated Use the following code snippet instead:
    *
    *             <pre>
    *             InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(input);
    *             calculator.setConsiderCoriolisAndCentrifugalForces(considerCoriolisAndCentrifugalForces);
    *             calculator.setConsiderJointAccelerations(considerJointAccelerations);
    *             </pre>
    */
   @Deprecated
   public InverseDynamicsCalculator(MultiBodySystemReadOnly input, boolean considerCoriolisAndCentrifugalForces, boolean considerJointAccelerations)
   {
      this(input);
      setConsiderCoriolisAndCentrifugalForces(considerCoriolisAndCentrifugalForces);
      setConsiderJointAccelerations(considerJointAccelerations);
   }

   /**
    * Creates a calculator for computing the joint efforts for system defined by the given
    * {@code input}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param input                                the definition of the system to be evaluated by this
    *                                             calculator.
    * @param considerCoriolisAndCentrifugalForces whether the effort resulting from the Coriolis and
    *                                             centrifugal forces should be considered.
    * @param considerJointAccelerations           whether the effort resulting from the joint
    *                                             accelerations should be considered.
    * @param considerIgnoredSubtreesInertia       whether the inertia of the ignored part(s) of the
    *                                             given multi-body system should be considered. When
    *                                             {@code true}, this provides more accurate joint
    *                                             torques as they compensate for instance for the
    *                                             gravity acting on the ignored rigid-bodies, i.e.
    *                                             bodies which have an ancestor joint that is ignored
    *                                             as specified in the given {@code input}. When
    *                                             {@code false}, the resulting joint torques may be
    *                                             less accurate and this calculator may gain slight
    *                                             performance improvement.
    * @deprecated Use the following code snippet instead:
    *
    *             <pre>
    *             InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(input, considerIgnoredSubtreesInertia);
    *             calculator.setConsiderCoriolisAndCentrifugalForces(considerCoriolisAndCentrifugalForces);
    *             calculator.setConsiderJointAccelerations(considerJointAccelerations);
    *             </pre>
    */
   @Deprecated
   public InverseDynamicsCalculator(MultiBodySystemReadOnly input, boolean considerCoriolisAndCentrifugalForces, boolean considerJointAccelerations,
                                    boolean considerIgnoredSubtreesInertia)
   {
      this(input, considerIgnoredSubtreesInertia);
      setConsiderCoriolisAndCentrifugalForces(considerCoriolisAndCentrifugalForces);
      setConsiderJointAccelerations(considerJointAccelerations);
   }

   /**
    * Creates a calculator for computing the joint efforts for all the descendants of the given
    * {@code rootBody}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param rootBody the supporting body of the subtree to be evaluated by this calculator. Not
    *                 modified.
    */
   public InverseDynamicsCalculator(RigidBodyReadOnly rootBody)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), true);
   }

   /**
    * Creates a calculator for computing the joint efforts for system defined by the given
    * {@code input}.
    * <p>
    * Do not forgot to set the gravitational acceleration so this calculator can properly account for
    * it.
    * </p>
    *
    * @param input the definition of the system to be evaluated by this calculator.
    */
   public InverseDynamicsCalculator(MultiBodySystemReadOnly input)
   {
      this(input, true);
   }

   /**
    * Creates a calculator for computing the joint efforts for system defined by the given
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
    *                                       this provides more accurate joint torques as they
    *                                       compensate for instance for the gravity acting on the
    *                                       ignored rigid-bodies, i.e. bodies which have an ancestor
    *                                       joint that is ignored as specified in the given
    *                                       {@code input}. When {@code false}, the resulting joint
    *                                       torques may be less accurate and this calculator may gain
    *                                       slight performance improvement.
    */
   public InverseDynamicsCalculator(MultiBodySystemReadOnly input, boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;

      RigidBodyReadOnly rootBody = input.getRootBody();
      initialRecursionStep = new RecursionStep(rootBody, null, null);
      rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
      buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore());

      if (considerIgnoredSubtreesInertia)
         initialRecursionStep.includeIgnoredSubtreeInertia();

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      allJointAccelerationMatrix = new DMatrixRMaj(nDoFs, 1);
      allJointTauMatrix = new DMatrixRMaj(nDoFs, 1);

      Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> accelerationFunction = body ->
      {
         RecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);
         return recursionStep == null ? null : recursionStep.rigidBodyAcceleration;
      };
      accelerationProvider = RigidBodyAccelerationProvider.toRigidBodyAccelerationProvider(accelerationFunction,
                                                                                           input.getInertialFrame(),
                                                                                           this::areCoriolisAndCentrifugalForcesConsidered,
                                                                                           this::areJointAccelerationsConsidered);
   }

   private void buildMultiBodyTree(RecursionStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
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

         RigidBodyReadOnly childBody = childJoint.getSuccessor();

         if (childBody != null)
         {
            if (rigidBodyToRecursionStepMap.containsKey(childBody))
            {
               int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
               LoopClosureRecursionStep recursion = new LoopClosureRecursionStep(childJoint, parent, rigidBodyToRecursionStepMap.get(childBody), jointIndices);
               jointToRecursionStepMap.put(childJoint, recursion);
            }
            else
            {
               int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
               RecursionStep child = new RecursionStep(childBody, parent, jointIndices);
               rigidBodyToRecursionStepMap.put(childBody, child);
               jointToRecursionStepMap.put(childJoint, child);
               buildMultiBodyTree(child, jointsToIgnore);
            }
         }
      }
   }

   /**
    * Sets whether the effort resulting from the Coriolis and centrifugal forces should be considered.
    *
    * @param considerCoriolisAndCentrifugalForces {@code true} to account for Coriolis and centrifugal
    *                                             forces, {@code false} for ignoring them. Default
    *                                             value {@value #DEFAULT_CONSIDER_CORIOLIS}.
    */
   public void setConsiderCoriolisAndCentrifugalForces(boolean considerCoriolisAndCentrifugalForces)
   {
      this.considerCoriolisAndCentrifugalForces = considerCoriolisAndCentrifugalForces;
   }

   /**
    * Sets whether the effort resulting from the joint accelerations should be considered.
    *
    * @param considerJointAccelerations {@code true} to account for joint accelerations, {@code false}
    *                                   for ignoring them. Default value
    *                                   {@value #DEFAULT_CONSIDER_ACCELERATIONS}.
    */
   public void setConsiderJointAccelerations(boolean considerJointAccelerations)
   {
      this.considerJointAccelerations = considerJointAccelerations;
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
      return rigidBodyToRecursionStepMap.get(rigidBody).externalWrench;
   }

   /**
    * Sets external wrench to apply to the given {@code rigidBody}.
    *
    * @param rigidBody      the rigid-body to which the wrench is to applied. Not modified.
    * @param externalWrench the external wrench to apply to the rigid-body.
    */
   public void setExternalWrench(RigidBodyReadOnly rigidBody, WrenchReadOnly externalWrench)
   {
      getExternalWrench(rigidBody).setMatchingFrame(externalWrench);
   }

   /**
    * Computes the joint efforts needed to achieve the desired joint accelerations.
    * <p>
    * The desired joint accelerations are extracted from the joint state. To explicitly specify the
    * joint accelerations to use, see {@link #compute(DMatrix)}.
    * </p>
    */
   public void compute()
   {
      compute(null);
   }

   /**
    * Computes the joint efforts needed to achieve the given joint accelerations.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    *
    * @param jointAccelerationMatrix the matrix containing the joint accelerations to use. Not
    *                                modified.
    */
   public void compute(DMatrix jointAccelerationMatrix)
   {
      initializeJointAccelerationMatrix(jointAccelerationMatrix);
      initialRecursionStep.passOneRecursive();
      initialRecursionStep.passTwoRecursive();
   }

   /**
    * Initialise the joint acceleration matrix containing the desired accelerations to achieve.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    *
    * @param jointAccelerationMatrix the matrix containing the joint accelerations to use. Not
    *                                modified.
    */
   void initializeJointAccelerationMatrix(DMatrix jointAccelerationMatrix)
   {
      if (jointAccelerationMatrix != null)
      {
         allJointAccelerationMatrix.set(jointAccelerationMatrix);
      }
      else
      {
         List<? extends JointReadOnly> indexedJointsInOrder = input.getJointMatrixIndexProvider().getIndexedJointsInOrder();
         MultiBodySystemTools.extractJointsState(indexedJointsInOrder, JointStateType.ACCELERATION, allJointAccelerationMatrix);
      }
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
    * Gets the spatial inertia of the chosen {@code RigidBody}.
    *
    * @return the spatial inertia of the chosen rigid body.
    */
   public SpatialInertia getBodyInertia(RigidBodyReadOnly rigidBody)
   {
      return rigidBodyToRecursionStepMap.get(rigidBody).getBodyInertia();
   }

   /**
    * Returns whether this calculator is considering the efforts resulting from joint accelerations or
    * not.
    *
    * @return {@code true} if this calculator is accounting for joint accelerations, {@code false}
    *         otherwise.
    * @see #setConsiderJointAccelerations(boolean)
    */
   public boolean areJointAccelerationsConsidered()
   {
      return considerJointAccelerations;
   }

   /**
    * Returns whether this calculator is considering the efforts resulting from Coriolis and
    * centrifugal effects or not.
    *
    * @return {@code true} if this calculator is accounting for Coriolis and centrifugal forces,
    *         {@code false} otherwise.
    * @see #setConsiderCoriolisAndCentrifugalForces(boolean)
    */
   public boolean areCoriolisAndCentrifugalForcesConsidered()
   {
      return considerCoriolisAndCentrifugalForces;
   }

   /**
    * Gets the computed joint efforts.
    *
    * @return this calculator output: the joint efforts.
    */
   public DMatrixRMaj getJointTauMatrix()
   {
      return allJointTauMatrix;
   }

   /**
    * Gets the computed wrench for the given {@code joint}.
    *
    * @param joint the query. Not modified.
    * @return the joint wrench or {@code null} if this calculator does not consider the given joint.
    */
   public WrenchReadOnly getComputedJointWrench(JointReadOnly joint)
   {
      RecursionStepBasics recursionStep = jointToRecursionStepMap.get(joint);
      if (recursionStep == null)
         return null;
      else
         return recursionStep.getJointWrench();
   }

   /**
    * Gets the computed N-by-1 effort vector for the given {@code joint}, where N is the number of
    * degrees of freedom the joint has.
    *
    * @param joint the query. Not modify.
    * @return the tau matrix.
    */
   public DMatrixRMaj getComputedJointTau(JointReadOnly joint)
   {
      RecursionStepBasics recursionStep = jointToRecursionStepMap.get(joint);

      if (recursionStep == null)
         return null;
      else
         return recursionStep.getTau();
   }

   /**
    * Writes the computed joint efforts into the given {@code joints}.
    * <p>
    * Any joint that is not considered by this calculator remains unchanged.
    * </p>
    *
    * @param joints the array of joints to write the effort into. Modified.
    */
   // TODO This is a poor method name, should probably be something like writeComputedJointEfforts or writeComputedJointTaus
   public void writeComputedJointWrenches(JointBasics[] joints)
   {
      for (JointBasics joint : joints)
         writeComputedJointWrench(joint);
   }

   /**
    * Writes the computed joint efforts into the given {@code joints}.
    * <p>
    * Any joint that is not considered by this calculator remains unchanged.
    * </p>
    *
    * @param joints the list of joints to write the effort into. Modified.
    */
   // TODO This is a poor method name, should probably be something like writeComputedJointEfforts or writeComputedJointTaus
   public void writeComputedJointWrenches(List<? extends JointBasics> joints)
   {
      for (int i = 0; i < joints.size(); i++)
         writeComputedJointWrench(joints.get(i));
   }

   /**
    * Writes the computed effort into the given {@code joint}.
    * <p>
    * If this calculator does not consider this joint, it remains unchanged.
    * </p>
    *
    * @param joint the joint to retrieve the acceleration of and to store it. Modified.
    * @return {@code true} if the joint effort was modified, {@code false} otherwise.
    */
   // TODO This is a poor method name, should probably be something like writeComputedJointEffort or writeComputedJointTau
   public boolean writeComputedJointWrench(JointBasics joint)
   {
      DMatrixRMaj jointTau = getComputedJointTau(joint);

      if (jointTau == null)
         return false;

      joint.setJointTau(0, jointTau);
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

   private interface RecursionStepBasics
   {
      void includeIgnoredSubtreeInertia();

      /**
       * Resets the external wrenches from here down the leaves recursively.
       */
      void setExternalWrenchToZeroRecursive();

      /**
       * First pass going from the root to the leaves.
       * <p>
       * Here the rigid-body accelerations are updated and the net wrenches resulting from the rigid-body
       * acceleration and velocity are computed.
       * </p>
       */
      void passOneRecursive();

      /**
       * Second pass going from leaves to the root.
       * <p>
       * The net wrenches are propagated upstream and summed up at each rigid-body to compute the joint
       * effort.
       * </p>
       */
      void passTwoRecursive();

      RecursionStep getParent();

      WrenchReadOnly getJointWrench();

      RigidBodyReadOnly getRigidBody();

      DMatrixRMaj getTau();

      default String getSimpleNameForParent()
      {
         return String.format("{%s}", getRigidBody().getName());
      }
   }

   /** Intermediate result used for garbage free operations. */
   private final SpatialForce jointForceFromChild = new SpatialForce();

   /**
    * Represents a single recursion step with all the intermediate variables needed.
    *
    * @author Sylvain Bertrand
    */
   final class RecursionStep implements RecursionStepBasics
   {
      /**
       * The rigid-body for which this recursion is.
       */
      private final RigidBodyReadOnly rigidBody;
      /**
       * The parent joint of {@link #rigidBody}.
       */
      private final JointReadOnly joint;
      /**
       * Body inertia: usually equal to {@code rigidBody.getInertial()}. However, if at least one child of
       * {@code rigidBody} is ignored, it is equal to this rigid-body inertia and the subtree inertia
       * attached to the ignored joint.
       */
      private final SpatialInertia bodyInertia;
      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid-body.
       */
      private final RecursionStep parent;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      private final List<RecursionStepBasics> children = new ArrayList<>();
      /**
       * Calculated joint wrench, before projection onto the joint motion subspace.
       */
      private final Wrench jointWrench;
      /**
       * User input: external wrench to be applied to this body.
       */
      private final FixedFrameWrenchBasics externalWrench;

      /**
       * The rigid-body spatial acceleration.
       */
      private final SpatialAcceleration rigidBodyAcceleration;
      /**
       * Intermediate variable for storing this joint acceleration.
       */
      private final SpatialAcceleration localJointAcceleration = new SpatialAcceleration();
      /**
       * Intermediate variable for storing this joint twist.
       */
      private final Twist localJointTwist = new Twist();
      /**
       * <tt>S</tt> is the 6-by-N matrix representing the motion subspace of the parent joint, where N is
       * the number of DoFs of the joint.
       */
      private final DMatrixRMaj S;
      /**
       * Joint acceleration.
       */
      private final DMatrixRMaj qdd;
      /**
       * Rigid-body spatial acceleration.
       */
      private final DMatrixRMaj a;
      /**
       * Computed joint effort.
       */
      private final DMatrixRMaj tau;
      /**
       * Computed joint wrench, before projection onto the joint motion subspace.
       */
      private final DMatrixRMaj jointWrenchMatrix;
      /**
       * Joint indices for storing {@code tau} in the main matrix {@code jointTauMatrix}.
       */
      private int[] jointIndices;
      /**
       * Intermediate variable for holding the body twist.
       */
      private TwistReadOnly bodyTwistToUse;

      public RecursionStep(RigidBodyReadOnly rigidBody, RecursionStep parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;
         rigidBodyAcceleration = new SpatialAcceleration(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());

         if (isRoot())
         {
            joint = null;
            bodyInertia = null;
            jointWrench = null;
            externalWrench = null;
            S = null;
            qdd = null;
            a = null;
            tau = null;
            jointWrenchMatrix = null;
            bodyTwistToUse = null;
         }
         else
         {
            joint = rigidBody.getParentJoint();
            parent.children.add(this);
            int nDoFs = joint.getDegreesOfFreedom();

            bodyInertia = new SpatialInertia(rigidBody.getInertia());
            jointWrench = new Wrench();
            externalWrench = new Wrench(getBodyFixedFrame(), getBodyFixedFrame());
            S = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, nDoFs);
            qdd = new DMatrixRMaj(nDoFs, 1);
            a = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            tau = new DMatrixRMaj(nDoFs, 1);
            jointWrenchMatrix = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            if (!joint.isMotionSubspaceVariable())
               joint.getMotionSubspace(S);
            bodyTwistToUse = new Twist();
         }
      }

      @Override
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

      @Override
      public void passOneRecursive()
      {
         passOne();

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            children.get(childIndex).passOneRecursive();
         }
      }

      public void passOne()
      {
         if (!isRoot())
         {
            if (joint.isMotionSubspaceVariable())
               joint.getMotionSubspace(S);

            rigidBodyAcceleration.setIncludingFrame(parent.rigidBodyAcceleration);

            if (considerCoriolisAndCentrifugalForces)
            {
               joint.getPredecessorTwist(localJointTwist);
               rigidBodyAcceleration.changeFrame(getBodyFixedFrame(), localJointTwist, parent.getBodyFixedFrame().getTwistOfFrame());
               bodyTwistToUse = getBodyFixedFrame().getTwistOfFrame();
            }
            else
            {
               rigidBodyAcceleration.changeFrame(getBodyFixedFrame());
               bodyTwistToUse = null;
            }

            if (considerJointAccelerations)
            {
               int nDoFs = joint.getDegreesOfFreedom();

               CommonOps_DDRM.extract(allJointAccelerationMatrix, jointIndices, nDoFs, qdd);
               CommonOps_DDRM.mult(S, qdd, a);
               localJointAcceleration.setIncludingFrame(joint.getFrameAfterJoint(), joint.getFrameBeforeJoint(), joint.getFrameAfterJoint(), a);
               if (joint.isMotionSubspaceVariable())
               {
                  SpatialAccelerationReadOnly jointBiasAcceleration = joint.getJointBiasAcceleration();
                  localJointAcceleration.checkReferenceFrameMatch(jointBiasAcceleration);
                  localJointAcceleration.add((SpatialVectorReadOnly) jointBiasAcceleration);
               }
               localJointAcceleration.changeFrame(getBodyFixedFrame());
               localJointAcceleration.setBodyFrame(getBodyFixedFrame());
               localJointAcceleration.setBaseFrame(parent.getBodyFixedFrame());
               rigidBodyAcceleration.add(localJointAcceleration);
            }
            else
            {
               rigidBodyAcceleration.setBodyFrame(getBodyFixedFrame());
            }
         }
      }

      @Override
      public void passTwoRecursive()
      {
         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            children.get(childIndex).passTwoRecursive();
         }

         passTwo();
      }

      public void passTwo()
      {
         if (isRoot())
            return;

         bodyInertia.computeDynamicWrench(rigidBodyAcceleration, bodyTwistToUse, jointWrench);

         jointWrench.sub(externalWrench);
         jointWrench.changeFrame(joint.getFrameAfterJoint());

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            addJointWrenchFromChild(children.get(childIndex));

         jointWrench.get(jointWrenchMatrix);
         CommonOps_DDRM.multTransA(S, jointWrenchMatrix, tau);

         for (int dofIndex = 0; dofIndex < joint.getDegreesOfFreedom(); dofIndex++)
         {
            allJointTauMatrix.set(jointIndices[dofIndex], 0, tau.get(dofIndex, 0));
         }
      }

      private void addJointWrenchFromChild(RecursionStepBasics child)
      {
         jointForceFromChild.setIncludingFrame(child.getJointWrench());
         jointForceFromChild.changeFrame(joint.getFrameAfterJoint());
         jointWrench.add(jointForceFromChild);
      }

      public SpatialInertia getBodyInertia()
      {
         return bodyInertia;
      }

      @Override
      public void setExternalWrenchToZeroRecursive()
      {
         if (!isRoot())
            externalWrench.setToZero();

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
         {
            children.get(childIndex).setExternalWrenchToZeroRecursive();
         }
      }

      @Override
      public RecursionStep getParent()
      {
         return parent;
      }

      @Override
      public RigidBodyReadOnly getRigidBody()
      {
         return rigidBody;
      }

      public MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }

      private boolean isRoot()
      {
         return parent == null;
      }

      @Override
      public Wrench getJointWrench()
      {
         return jointWrench;
      }

      @Override
      public DMatrixRMaj getTau()
      {
         return tau;
      }

      @Override
      public String toString()
      {
         String bodyName = rigidBody == null ? "null" : rigidBody.getName();
         String jointName = joint == null ? "null" : joint.getName();
         String childrenBodyNames = EuclidCoreIOTools.getCollectionString(", ", children, RecursionStepBasics::getSimpleNameForParent);
         return String.format("Body: %s, joint: %s, children: [%s]", bodyName, jointName, childrenBodyNames);
      }
   }

   /**
    * Represents a single recursion step with all the intermediate variables needed.
    *
    * @author Sylvain Bertrand
    */
   private final class LoopClosureRecursionStep implements RecursionStepBasics
   {
      private final JointReadOnly joint;
      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid-body.
       */
      private final RecursionStep parent;
      /**
       * Calculated joint wrench, before projection onto the joint motion subspace.
       */
      private final Wrench jointWrench;
      /**
       * Computed joint effort.
       */
      private final DMatrixRMaj tau;
      /**
       * Joint indices for storing {@code tau} in the main matrix {@code jointTauMatrix}.
       */
      private int[] jointIndices;
      /**
       * The next recursion after the loop closure. {@code successorRecursion} shares the same rigid-body
       * as this recursion.
       */
      private RecursionStep successorRecursion;

      public LoopClosureRecursionStep(JointReadOnly loopClosureJoint, RecursionStep parent, RecursionStep successorRecursion, int[] jointIndices)
      {
         this.parent = parent;
         if (loopClosureJoint.getSuccessor() != successorRecursion.rigidBody)
            throw new IllegalArgumentException("Rigid-body mismatch. Joint's successor: " + loopClosureJoint.getSuccessor() + ", recursion body: "
                  + successorRecursion.rigidBody);
         if (loopClosureJoint == successorRecursion.joint)
            throw new IllegalArgumentException("This recursion joint should not be equal to the successor joint, joint: " + loopClosureJoint);

         joint = loopClosureJoint;
         this.successorRecursion = successorRecursion;
         this.jointIndices = jointIndices;

         parent.children.add(this);
         int nDoFs = joint.getDegreesOfFreedom();

         jointWrench = new Wrench();
         tau = new DMatrixRMaj(nDoFs, 1);
      }

      @Override
      public void includeIgnoredSubtreeInertia()
      {
         // Do nothing, it's done in the successorRecursion
      }

      @Override
      public void passOneRecursive()
      {
         /*
          * Do nothing, assume the joint acceleration satisfies the constraint such that the acceleration
          * computed in successorRecursion is valid for this joint too.
          */
      }

      @Override
      public void passTwoRecursive()
      {
         // The effort at the joint is set to zero and has to be handled externally once the effort for the loop joints has been computed.
         jointWrench.setToZero(getRigidBody().getBodyFixedFrame(), joint.getFrameAfterJoint());
         tau.zero();

         for (int dofIndex = 0; dofIndex < joint.getDegreesOfFreedom(); dofIndex++)
         {
            allJointTauMatrix.set(jointIndices[dofIndex], 0, tau.get(dofIndex, 0));
         }
      }

      /**
       * Resets the external wrenches from here down the leaves recursively.
       */
      @Override
      public void setExternalWrenchToZeroRecursive()
      {
         // Do nothing here, the external wrench is stored in the successorRecursion
      }

      @Override
      public RecursionStep getParent()
      {
         return parent;
      }

      @Override
      public RigidBodyReadOnly getRigidBody()
      {
         return successorRecursion.rigidBody;
      }

      @Override
      public Wrench getJointWrench()
      {
         return jointWrench;
      }

      @Override
      public DMatrixRMaj getTau()
      {
         return tau;
      }

      @Override
      public String toString()
      {
         String bodyName = successorRecursion.rigidBody.getName();
         String jointName = joint.getName();
         return String.format("Body: %s, joint: %s, successor recursion: [%s]", bodyName, jointName, successorRecursion.toString());
      }
   }
}
