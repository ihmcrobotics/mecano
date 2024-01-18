package us.ihmc.mecano.algorithms;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.*;
import java.util.function.Function;

/**
 * Computed joint accelerations based on joint efforts.
 * <p>
 * This calculator is based on the articulated-body inertia algorithm as described in Featherstone -
 * Rigid Body Dynamics Algorithms (2008): <a href=
 * "https://books.google.com/books?id=GJRGBQAAQBAJ&lpg=PR5&ots=XoFXvnJZLH&dq=rigid%20body%20dynamics%20algorithms&lr&pg=PR1#v=onepage&q=rigid%20body%20dynamics%20algorithms&f=false">link</a>
 * </p>
 * <p>
 * Note on kinematic loops: this calculator does not support kinematic loops yet.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class ForwardDynamicsCalculator
{
   public enum JointSourceMode
   {
      /**
       * Default mode for a joint, its acceleration is computed in this calculator based on the joint's
       * effort.
       */
      EFFORT_SOURCE,
      /**
       * Secondary mode, the joint's acceleration is pre-determined and its effort is computed by this
       * calculator.
       */
      ACCELERATION_SOURCE;
   }

   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;

   /** The root of the internal recursive algorithm. */
   private final ArticulatedBodyRecursionStep initialRecursionStep;
   /** Map to quickly retrieve information for each rigid-body. */
   private final Map<RigidBodyReadOnly, ArticulatedBodyRecursionStep> rigidBodyToRecursionStepMap = new LinkedHashMap<>();
   /** For iterating over each rigid-body quickly. */
   private final ArticulatedBodyRecursionStep[] articulatedBodyRecursionSteps;

   private boolean isJointTauOutputDirty = true;
   private boolean isJointAccelerationOutputDirty = true;
   /**
    * The input of this algorithm: the effort matrix for all the joints to consider.
    * <p>
    * Also serves as the output for locked joint.
    * </p>
    */
   private final DMatrixRMaj jointTauOutput;
   /**
    * The output of this algorithm: the acceleration matrix for all the joints to consider.
    * <p>
    * Also serves as the input for locked joint.
    * </p>
    */
   private final DMatrixRMaj jointAccelerationOutput;

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

   private final int totalDoFs;

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
    * @throws UnsupportedOperationException if the multi-body system contains kinematic loop(s).
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
    * @throws UnsupportedOperationException if the {@code input} contains kinematic loop(s).
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
    * @throws UnsupportedOperationException if the {@code input} contains kinematic loop(s).
    */
   public ForwardDynamicsCalculator(MultiBodySystemReadOnly input, boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;

      RigidBodyReadOnly rootBody = input.getRootBody();
      initialRecursionStep = new ArticulatedBodyRecursionStep(rootBody, null, null);
      rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
      buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore());
      if (considerIgnoredSubtreesInertia)
         initialRecursionStep.updateIgnoredSubtreeInertia();
      articulatedBodyRecursionSteps = rigidBodyToRecursionStepMap.values().toArray(new ArticulatedBodyRecursionStep[rigidBodyToRecursionStepMap.size()]);

      totalDoFs = input.getNumberOfDoFs();
      jointTauOutput = new DMatrixRMaj(totalDoFs, 1);
      jointAccelerationOutput = new DMatrixRMaj(totalDoFs, 1);

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
      List<JointReadOnly> childrenJoints = MultiBodySystemTools.sortLoopClosureInChildrenJoints(parent.rigidBody);

      for (JointReadOnly childJoint : childrenJoints)
      {
         if (jointsToIgnore.contains(childJoint))
            continue;

         if (childJoint.isLoopClosure())
         {
            System.out.println(getClass().getSimpleName() + ": This calculator does not support kinematic loops. Ignoring joint: " + childJoint.getName());
            continue;
         }

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
   public void setGravitationalAcceleration(FrameTuple3DReadOnly gravity)
   {
      gravity.checkReferenceFrameMatch(input.getInertialFrame());
      setGravitationalAcceleration((Tuple3DReadOnly) gravity);
   }

   /**
    * @deprecated Use {@link #setGravitationalAcceleration(FrameTuple3DReadOnly)} instead.
    */
   @Deprecated
   public void setGravitionalAcceleration(FrameTuple3DReadOnly gravity)
   {
      setGravitationalAcceleration(gravity);
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
   public void setGravitationalAcceleration(Tuple3DReadOnly gravity)
   {
      SpatialAcceleration rootAcceleration = initialRecursionStep.rigidBodyAcceleration;
      rootAcceleration.setToZero();
      rootAcceleration.getLinearPart().setAndNegate(gravity);
   }

   /**
    * @deprecated Use {@link #setGravitationalAcceleration(Tuple3DReadOnly)} instead.
    */
   @Deprecated
   public void setGravitionalAcceleration(Tuple3DReadOnly gravity)
   {
      setGravitationalAcceleration(gravity);
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
   public void setGravitationalAcceleration(double gravity)
   {
      setGravitationalAcceleration(0.0, 0.0, gravity);
   }

   /**
    * @deprecated Use {@link #setGravitationalAcceleration(double)} instead.
    */
   @Deprecated
   public void setGravitionalAcceleration(double gravity)
   {
      setGravitationalAcceleration(gravity);
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
   public void setGravitationalAcceleration(double gravityX, double gravityY, double gravityZ)
   {
      SpatialAcceleration rootAcceleration = initialRecursionStep.rigidBodyAcceleration;
      rootAcceleration.setToZero();
      rootAcceleration.getLinearPart().set(gravityX, gravityY, gravityZ);
      rootAcceleration.negate();
   }

   /**
    * @deprecated Use {@link #setGravitationalAcceleration(double, double, double)} instead.
    */
   @Deprecated
   public void setGravitionalAcceleration(double gravityX, double gravityY, double gravityZ)
   {
      setGravitationalAcceleration(gravityX, gravityY, gravityZ);
   }

   /**
    * Changes the spatial acceleration of the root. Even though the root is assumed to be non-moving,
    * the {@code rootAcceleration} is usually set to the opposite of the gravitational acceleration,
    * such that the effect of the gravity is naturally propagated to the entire system.
    *
    * @param newRootAcceleration the new spatial acceleration of the root.
    * @throws ReferenceFrameMismatchException if any of the reference frames of
    *       {@code newRootAcceleration} does not match this
    *       calculator's root spatial acceleration's frames.
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
      for (ArticulatedBodyRecursionStep recursionStep : articulatedBodyRecursionSteps)
      {
         if (recursionStep.externalWrench != null)
            recursionStep.externalWrench.setToZero();
      }
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
    * Sets source mode for the given joint:
    * <ul>
    * <li>{@link JointSourceMode#EFFORT_SOURCE} (default): the joint effort is provided as an input to
    * this calculator, via as joint state or the given matrix when calling compute. The joint
    * acceleration is computed in this calculator and depend on the joint's effort. Note that in that
    * mode, any provided acceleration input for that joint is ignored.
    * <li>{@link JointSourceMode#ACCELERATION_SOURCE}: the joint acceleration is provided as an input
    * to this calculator, via as joint state or the given matrix when calling compute. The joint effort
    * is computed in this calculator and depend on the joint's acceleration. Note that in that mode,
    * any provided effort input for that joint is ignored.
    * </ul>
    *
    * @param joint the joint set the source mode of. Not modified.
    * @param mode  the desired mode for the joint. Default value is
    *              {@link JointSourceMode#ACCELERATION_SOURCE}.
    */
   public void setJointSourceMode(JointReadOnly joint, JointSourceMode mode)
   {
      rigidBodyToRecursionStepMap.get(joint.getSuccessor()).sourceMode = mode;
   }

   /**
    * Convenience method for setting the source mode of all the joints this calculator handles.
    * <ul>
    * <li>{@link JointSourceMode#EFFORT_SOURCE} (default): the joint effort is provided as an input to
    * this calculator, via {@link JointReadOnly#getJointWrench()} or the given matrix when calling
    * compute. The joint acceleration is computed in this calculator and depend on the joint's effort.
    * Note that in that mode, any provided acceleration input for that joint is ignored.
    * <li>{@link JointSourceMode#ACCELERATION_SOURCE}: the joint acceleration is provided as an input
    * to this calculator, via {@link JointReadOnly#getJointAcceleration()} or the given matrix when
    * calling compute. The joint effort is computed in this calculator and depend on the joint's
    * acceleration. Note that in that mode, any provided effort input for that joint is ignored.
    * </ul>
    *
    * @param jointSourceModeFunction the function used to determine a joint source mode. The function
    *                                can return {@code null} for joints which source mode should not be
    *                                changed.
    */
   public void setJointSourceModes(Function<JointReadOnly, JointSourceMode> jointSourceModeFunction)
   {
      for (ArticulatedBodyRecursionStep recursionStep : articulatedBodyRecursionSteps)
      {
         if (recursionStep.getJoint() != null)
         {
            JointSourceMode newMode = jointSourceModeFunction.apply(recursionStep.getJoint());
            if (newMode != null)
               recursionStep.sourceMode = newMode;
         }
      }
   }

   /**
    * Resets all joint to their default source mode: {@link JointSourceMode#EFFORT_SOURCE}.
    */
   public void resetJointSourceModes()
   {
      for (ArticulatedBodyRecursionStep recursionStep : articulatedBodyRecursionSteps)
      {
         recursionStep.sourceMode = JointSourceMode.EFFORT_SOURCE;
      }
   }

   /**
    * Gets the current source mode for the given joint.
    * <ul>
    * Source mode can be:
    * <li>{@link JointSourceMode#EFFORT_SOURCE} (default): the joint effort is provided as an input to
    * this calculator, via {@link JointReadOnly#getJointWrench()} or the given matrix when calling
    * compute. The joint acceleration is computed in this calculator and depend on the joint's effort.
    * Note that in that mode, any provided acceleration input for that joint is ignored.
    * <li>{@link JointSourceMode#ACCELERATION_SOURCE}: the joint acceleration is provided as an input
    * to this calculator, via {@link JointReadOnly#getJointAcceleration()} or the given matrix when
    * calling compute. The joint effort is computed in this calculator and depend on the joint's
    * acceleration. Note that in that mode, any provided effort input for that joint is ignored.
    * </ul>
    *
    * @param joint the joint to get the current source mode of.
    * @return the current joint source mode.
    */
   public JointSourceMode getJointSourceMode(JointReadOnly joint)
   {
      return rigidBodyToRecursionStepMap.get(joint.getSuccessor()).sourceMode;
   }

   /**
    * Computes the joint accelerations resulting from the joint efforts.
    * <p>
    * The desired joint efforts are extracted from the joint state. To explicitly specify the joint
    * efforts to use, see {@link #compute(DMatrix)}.
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
    * @param jointTauInput the matrix containing the joint efforts to use. Not modified.
    */
   public void compute(DMatrix jointTauInput)
   {
      compute(jointTauInput, null);
   }

   /**
    * Computes the joint accelerations resulting from the given joint efforts.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    *
    * @param jointTauInput          the matrix containing the joint efforts to use. If {@code null},
    *                               the joint state is used.. Not modified.
    * @param jointAccelerationInput the matrix containing the joint accelerations to use, only used for
    *                               joints with source mode set to
    *                               {@link JointSourceMode#ACCELERATION_SOURCE}. If {@code null}, the
    *                               joint state is used. Not modified.
    */
   public void compute(DMatrix jointTauInput, DMatrix jointAccelerationInput)
   {
      checkAllJointMatrixSize(jointTauInput);
      checkAllJointMatrixSize(jointAccelerationInput);

      isJointTauOutputDirty = true;
      isJointAccelerationOutputDirty = true;
      boolean atLeastOneLockedJoint = initialRecursionStep.passOne();
      initialRecursionStep.passTwo(jointTauInput);
      initialRecursionStep.passThree(jointAccelerationInput);
      if (atLeastOneLockedJoint)
         initialRecursionStep.passFour();
   }

   private void checkAllJointMatrixSize(DMatrix matrix)
   {
      if (matrix == null)
         return;

      if (matrix.getNumRows() != totalDoFs || matrix.getNumCols() != 1)
         throw new MatrixDimensionException(String.format("Incompatible matrix dimension, expected: [nRows: %d, nCols: %d], was: [nRows: %d, nCols: %d]",
                                                          totalDoFs,
                                                          1,
                                                          matrix.getNumRows(),
                                                          matrix.getNumCols()));
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
    * Gets the joint accelerations:
    * <ul>
    * <li>if the joint source mode is {@link JointSourceMode#EFFORT_SOURCE} (default), the acceleration
    * is computed by this calculator.
    * <li>if the joint source mode is {@link JointSourceMode#ACCELERATION_SOURCE}, the acceleration is
    * the input of this calculator.
    * </ul>
    *
    * @return the joint accelerations.
    */
   public DMatrixRMaj getJointAccelerationMatrix()
   {
      if (isJointAccelerationOutputDirty)
      {
         for (ArticulatedBodyRecursionStep articulatedBodyRecursionStep : articulatedBodyRecursionSteps)
         {
            articulatedBodyRecursionStep.getAccelerationOutput(jointAccelerationOutput);
         }
         isJointAccelerationOutputDirty = false;
      }
      return jointAccelerationOutput;
   }

   /**
    * Gets the joint efforts:
    * <ul>
    * <li>if the joint source mode is {@link JointSourceMode#EFFORT_SOURCE} (default), the effort is
    * the input of this calculator.
    * <li>if the joint source mode is {@link JointSourceMode#ACCELERATION_SOURCE}, the effort is
    * computed by this calculator.
    * </ul>
    *
    * @return the joint efforts.
    */
   public DMatrixRMaj getJointTauMatrix()
   {
      if (isJointTauOutputDirty)
      {
         for (ArticulatedBodyRecursionStep articulatedBodyRecursionStep : articulatedBodyRecursionSteps)
         {
            articulatedBodyRecursionStep.getTauOutput(jointTauOutput);
         }
         isJointTauOutputDirty = false;
      }
      return jointTauOutput;
   }

   /**
    * Gets the computed N-by-1 acceleration vector for the given {@code joint}, where N is the number
    * of degrees of freedom the joint has.
    *
    * @param joint the joint to get the acceleration of. Not modified.
    * @return the computed joint acceleration matrix.
    */
   public DMatrixRMaj getComputedJointAcceleration(JointReadOnly joint)
   {
      ArticulatedBodyRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());

      if (recursionStep == null)
         return null;
      else
         return recursionStep.qdd;
   }

   /**
    * Gets the N-by-1 force vector for the given {@code joint}, where N is the number of degrees of
    * freedom the joint has.
    * <p>
    * The joint efforts are provided in {@link #compute(DMatrix)} or extracted from the joint state and
    * are the input of this algorithm. Howver, for locked joints, this calculator instead preserve the
    * joint acceleration and calculate the joint effort.
    * </p>
    *
    * @param joint the joint to get the effort of. Not modified.
    * @return the joint effort matrix.
    */
   public DMatrixRMaj getJointTau(JointReadOnly joint)
   {
      ArticulatedBodyRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());

      if (recursionStep == null)
         return null;
      else
         return recursionStep.qdd;
   }

   /**
    * Gets the wrench exerted by the given {@code joint} on its predecessor.
    * <p>
    * This method returns the full 6-D joint wrench before projection onto the joint motion subspace. It can be used to examine the internal forces and torques
    * exerted at the joint.
    * </p>
    *
    * @param joint the joint to get the wrench of. Not modified.
    * @return the joint wrench.
    */
   public WrenchReadOnly getJointWrench(JointReadOnly joint)
   {
      ArticulatedBodyRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());

      if (recursionStep == null)
         return null;
      else
         return recursionStep.getJointWrench();
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

   /** Intermediate result used for garbage free operations. */
   private final SpatialForce jointForceFromChild = new SpatialForce();

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
       * Used for storing intermediate result when part of the subtree is ignored but the subtree inertia
       * is still to be considered.
       */
      private SpatialInertia bodyInertia;
      /**
       * Combined inertia of the subtree that is ignored but inertia is still to be considered for this
       * recursion step.
       */
      private SpatialInertia bodySubtreeInertia;
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
       *
       * <pre>
       * p<sup>A</sup> = p + &sum;<sub>&forall;child</sub> p<sup>a</sup>
       * </pre>
       */
      final SpatialForce articulatedBiasWrench;
      /**
       * Pre-transformed articulated-body inertia for the parent.
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
       * <tt>S</tt> is the 6-by-N matrix representing the motion subspace of the parent joint, where N is
       * the number of DoFs of the joint.
       */
      final DMatrixRMaj S;

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
      final DMatrixRMaj U;
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
      final DMatrixRMaj D;
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
      final DMatrixRMaj Dinv;
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
      final DMatrixRMaj U_Dinv;
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
      final DMatrixRMaj U_Dinv_UT;
      /**
       * This is the N-by-1 vector representing the joint effort, where N is equal to the number of DoFs
       * that the joint has.
       */
      final DMatrixRMaj tau;
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
      final DMatrixRMaj u;
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
      final DMatrixRMaj c;
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
      final DMatrixRMaj pa;
      /**
       * This body acceleration:
       *
       * <pre>
       * a = a' + S qDDot
       *   = a<sub>parent</sub> + c + S qDDot
       * </pre>
       */
      final DMatrixRMaj a;
      /**
       * Intermediate result for garbage-free operation.
       */
      final DMatrixRMaj qdd_intermediate;
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
      final DMatrixRMaj qdd;
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
      final LinearSolverDense<DMatrixRMaj> inverseSolver;
      /**
       * Intermediate variable to save computation. Transform from {@code this.getFrameAfterJoint()} to
       * {@code parent.getFrameAfterJoint()}.
       */
      final RigidBodyTransform transformToParentJointFrame;
      /**
       * Joint indices for storing {@code qdd} in the main matrix {@code jointAccelerationMatrix}.
       */
      final int[] jointIndices;
      /**
       * User parameter for determining the joint mode.
       */
      JointSourceMode sourceMode = JointSourceMode.EFFORT_SOURCE;
      /**
       * Calculated joint wrench, before projection onto the joint motion subspace.
       */
      private final Wrench jointWrench;
      private final DMatrixRMaj jointWrenchMatrix;
      private boolean isJointWrenchDirty = true;

      private ArticulatedBodyRecursionStep(RigidBodyReadOnly rigidBody, ArticulatedBodyRecursionStep parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;

         externalWrench = new Wrench(getBodyFixedFrame(), getBodyFixedFrame());

         if (parent == null)
         {
            biasAcceleration = null;
            biasWrench = null;
            spatialInertia = null;
            articulatedInertia = null;
            articulatedBiasWrench = null;
            articulatedInertiaForParent = null;
            articulatedBiasWrenchForParent = null;
            rigidBodyAcceleration.setToZero(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());
            rigidBodyZeroVelocityAcceleration.setToZero(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());

            S = null;
            U = null;
            D = null;
            Dinv = null;
            U_Dinv = null;
            U_Dinv_UT = null;
            tau = null;
            u = null;
            c = null;
            pa = null;
            qdd = null;
            qdd_intermediate = null;
            a = null;
            inverseSolver = null;
            transformToParentJointFrame = null;
            jointWrench = null;
            jointWrenchMatrix = null;
         }
         else
         {
            parent.children.add(this);
            int nDoFs = getJoint().getDegreesOfFreedom();

            biasAcceleration = new SpatialAcceleration();
            biasWrench = new Wrench();
            spatialInertia = new SpatialInertia();
            articulatedInertia = new ArticulatedBodyInertia();
            articulatedBiasWrench = new SpatialForce();
            articulatedInertiaForParent = parent.isRoot() ? null : new ArticulatedBodyInertia();
            articulatedBiasWrenchForParent = parent.isRoot() ? null : new SpatialForce();

            S = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, nDoFs);
            U = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, nDoFs);
            D = new DMatrixRMaj(nDoFs, nDoFs);
            Dinv = new DMatrixRMaj(nDoFs, nDoFs);
            U_Dinv = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, nDoFs);
            U_Dinv_UT = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, SpatialVectorReadOnly.SIZE);
            tau = new DMatrixRMaj(nDoFs, 1);
            u = new DMatrixRMaj(nDoFs, 1);
            c = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            pa = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            qdd = new DMatrixRMaj(nDoFs, 1);
            qdd_intermediate = new DMatrixRMaj(nDoFs, 1);
            a = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            inverseSolver = nDoFs == 6 ? LinearSolverFactory_DDRM.symmPosDef(6) : null;
            transformToParentJointFrame = new RigidBodyTransform();
            if (!getJoint().isMotionSubspaceVariable())
               getJoint().getMotionSubspace(S);
            jointWrench = new Wrench();
            jointWrenchMatrix = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
         }
      }

      private void updateIgnoredSubtreeInertia()
      {
         if (bodySubtreeInertia != null)
         {
            bodyInertia.setToZero();
            bodySubtreeInertia.setToZero();
         }

         if (!isRoot() && children.size() != rigidBody.getChildrenJoints().size())
         {
            for (JointReadOnly childJoint : rigidBody.getChildrenJoints())
            {
               if (input.getJointsToIgnore().contains(childJoint))
               {
                  SpatialInertia subtreeIneria = MultiBodySystemTools.computeSubtreeInertia(childJoint);
                  subtreeIneria.changeFrame(getBodyFixedFrame());
                  if (bodySubtreeInertia == null)
                  {
                     bodyInertia = new SpatialInertia(getBodyFixedFrame(), getBodyFixedFrame());
                     bodySubtreeInertia = new SpatialInertia(getBodyFixedFrame(), getBodyFixedFrame());
                  }
                  bodySubtreeInertia.add(subtreeIneria);
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).updateIgnoredSubtreeInertia();
      }

      /**
       * The first pass consists in calculating the bias wrench resulting from external and Coriolis
       * forces, and the bias acceleration resulting from the Coriolis acceleration.
       *
       * @return {@code true} if there is at least one joint that is locked.
       */
      public boolean passOne()
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

            if (bodyInertia != null)
            {
               bodyInertia.setIncludingFrame(rigidBody.getInertia());
               bodyInertia.add(bodySubtreeInertia);
               bodyInertia.computeDynamicWrench(null, getBodyTwist(), biasWrench);
            }
            else
            {
               rigidBody.getInertia().computeDynamicWrench(null, getBodyTwist(), biasWrench);
            }
            biasWrench.sub(externalWrench);
            biasWrench.changeFrame(frameAfterJoint);

            biasAcceleration.setToZero(frameAfterJoint, input.getInertialFrame(), frameBeforeJoint);
            biasAcceleration.changeFrame(frameAfterJoint, getJoint().getJointTwist(), frameAfterJoint.getTwistOfFrame());
            if (getJoint().isMotionSubspaceVariable())
               biasAcceleration.add((SpatialVectorReadOnly) getJoint().getJointBiasAcceleration());
            biasAcceleration.get(c);
         }

         isJointWrenchDirty = true;
         boolean atLeastOneAccelSourceJoint = sourceMode == JointSourceMode.ACCELERATION_SOURCE;

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            atLeastOneAccelSourceJoint |= children.get(childIndex).passOne();
         return atLeastOneAccelSourceJoint;
      }

      /**
       * The second two calculates the articulated-body inertia and the bias wrench that includes
       * Coriolis, external, and joint forces.
       * <p>
       * This pass also computes several intermediate variables to reduce the number of calculations.
       * </p>
       */
      public void passTwo(DMatrix jointTauInput)
      {
         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).passTwo(jointTauInput);

         if (isRoot())
            return;

         MovingReferenceFrame frameAfterJoint = getFrameAfterJoint();

         if (bodyInertia != null)
            spatialInertia.setIncludingFrame(bodyInertia);
         else
            spatialInertia.setIncludingFrame(rigidBody.getInertia());
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

         int nDoFs = getJoint().getDegreesOfFreedom();

         if (getJoint().isMotionSubspaceVariable())
            getJoint().getMotionSubspace(S);

         // Computing intermediate variables used in later calculation
         if (sourceMode == JointSourceMode.EFFORT_SOURCE)
         {
            // U_[6xN] = IA_[6x6] * S_[6xN]
            mult(articulatedInertia, S, U);
            // D_[NxN] = (S_[6xN])^T * U_[6xN]
            CommonOps_DDRM.multTransA(S, U, D);

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
               UnrolledInverseFromMinor_DDRM.inv(D, Dinv);
            }
            else
            {
               inverseSolver.setA(D);
               inverseSolver.invert(Dinv);
            }

            // Computing u_i = tau_i - S_i^T * p_i^A
            if (jointTauInput != null)
            {
               for (int dofIndex = 0; dofIndex < nDoFs; dofIndex++)
               {
                  tau.set(dofIndex, 0, jointTauInput.get(jointIndices[dofIndex], 0));
               }
            }
            else
            {
               getJoint().getJointTau(0, tau);
            }

            // u_[Nx1] = -(S_[6xN])^T * pA_[6x1]
            multTransA(-1.0, S, articulatedBiasWrench, u);
            // u_[Nx1] = u_[Nx1] + tau_[Nx1]
            CommonOps_DDRM.addEquals(u, tau);

            if (!parent.isRoot())
            {
               // U_Dinv_[6xN] = U_[6xN] * Dinv_[NxN]
               CommonOps_DDRM.mult(U, Dinv, U_Dinv);
               // U_Dinv_UT[6x6] = U_Dinv_[6xN] * (U_[6xN])^T
               CommonOps_DDRM.multTransB(U_Dinv, U, U_Dinv_UT);

               // Computing I_i^a = I_i^A - U_i * D_i^-1 * U_i^T
               articulatedInertiaForParent.setIncludingFrame(articulatedInertia);
               articulatedInertiaForParent.sub(U_Dinv_UT);

               // Computing p_i^a = p_i^A + I_i^a * c_i + U_i * D_i^-1 * u_i
               articulatedBiasWrench.get(pa);
               // pa_[6x1] += Ia_[6x6] * c_[6x1]
               multAdd(articulatedInertiaForParent, c, pa);
               // pa_[6x1] += U_Dinv_[6xN] * u_[Nx1]
               CommonOps_DDRM.multAdd(U_Dinv, u, pa);
               articulatedBiasWrenchForParent.setIncludingFrame(frameAfterJoint, pa);
            }
         }
         else
         {
            if (!parent.isRoot())
            {
               // Computing I_i^a = I_i^A - U_i * D_i^-1 * U_i^T
               articulatedInertiaForParent.setIncludingFrame(articulatedInertia);

               // Computing p_i^a = p_i^A + I_i^a * c_i + U_i * D_i^-1 * u_i
               articulatedBiasWrench.get(pa);
               multAdd(articulatedInertiaForParent, c, pa);

               getJoint().getJointAcceleration().get(a);
               multAdd(articulatedInertiaForParent, a, pa);

               articulatedBiasWrenchForParent.setIncludingFrame(frameAfterJoint, pa);
            }
         }
      }

      /**
       * The third and last pass calculate the joint acceleration and body spatial acceleration.
       */
      public void passThree(DMatrix jointAccelerationInput)
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

            int nDoFs = getJoint().getDegreesOfFreedom();

            if (sourceMode == JointSourceMode.EFFORT_SOURCE)
            {
               // Computing qdd_i = D_i^-1 * ( u_i - U_i^T * a'_i )
               multTransA(-1.0, U, rigidBodyAcceleration, qdd_intermediate);
               CommonOps_DDRM.addEquals(qdd_intermediate, u);
               CommonOps_DDRM.mult(Dinv, qdd_intermediate, qdd);
            }
            else
            {
               if (jointAccelerationInput != null)
               {
                  for (int dofIndex = 0; dofIndex < nDoFs; dofIndex++)
                  {
                     qdd.set(dofIndex, 0, jointAccelerationInput.get(jointIndices[dofIndex], 0));
                  }
               }
               else
               {
                  getJoint().getJointAcceleration(0, qdd);
               }
            }

            // Computing a_i = a'_i + S_i * qdd_i
            CommonOps_DDRM.mult(S, qdd, a);

            rigidBodyZeroVelocityAcceleration.add(a);

            addEquals(a, rigidBodyAcceleration);
            rigidBodyAcceleration.setIncludingFrame(getBodyFixedFrame(), input.getInertialFrame(), getFrameAfterJoint(), a);
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).passThree(jointAccelerationInput);
      }

      /**
       * Only needed when there is one or more locked joint. This will compute their joint wrench.
       */
      public void passFour()
      {
         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).passFour();

         if (isRoot())
            return;

         if (sourceMode == JointSourceMode.ACCELERATION_SOURCE)
         {
            getJointWrench().get(jointWrenchMatrix);
            CommonOps_DDRM.multTransA(S, jointWrenchMatrix, tau);
         }
      }

      private WrenchReadOnly getJointWrench()
      {
         if (isJointWrenchDirty)
         {
            if (!isRoot())
            {
               MovingReferenceFrame frameAfterJoint = getFrameAfterJoint();

         rigidBodyAcceleration.changeFrame(getBodyFixedFrame());

         if (bodyInertia != null)
            bodyInertia.computeDynamicWrench(rigidBodyAcceleration, null, jointWrench);
         else
            rigidBody.getInertia().computeDynamicWrench(rigidBodyAcceleration, null, jointWrench);

         jointWrench.sub(externalWrench);
         jointWrench.changeFrame(frameAfterJoint);
         jointWrench.add(biasWrench);

               for (int childIndex = 0; childIndex < children.size(); childIndex++)
                  addJointWrenchFromChild(children.get(childIndex));
            }

            isJointWrenchDirty = false;
         }
         return jointWrench;
      }

      private void addJointWrenchFromChild(ArticulatedBodyRecursionStep child)
      {
         jointForceFromChild.setIncludingFrame(child.getJointWrench());
         jointForceFromChild.changeFrame(getFrameAfterJoint());
         jointWrench.add(jointForceFromChild);
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

      public void getAccelerationOutput(DMatrix allJointAccelerationMatrix)
      {
         if (isRoot())
            return;

         int nDoFs = getJoint().getDegreesOfFreedom();

         for (int dofIndex = 0; dofIndex < nDoFs; dofIndex++)
         {
            allJointAccelerationMatrix.set(jointIndices[dofIndex], 0, qdd.get(dofIndex, 0));
         }
      }

      public void getTauOutput(DMatrix allJointTauMatrix)
      {
         if (isRoot())
            return;

         int nDoFs = getJoint().getDegreesOfFreedom();

         for (int dofIndex = 0; dofIndex < nDoFs; dofIndex++)
         {
            allJointTauMatrix.set(jointIndices[dofIndex], 0, tau.get(dofIndex, 0));
         }
      }

      @Override
      public String toString()
      {
         return "RigidBody: " + rigidBody + ", parent: " + parent.rigidBody + ", children: " + Arrays.asList(children.stream().map(c -> c.rigidBody).toArray());
      }
   }

   /**
    * Same as {@link CommonOps_DDRM#addEquals(DMatrixD1, DMatrixD1)}.
    */
   static void addEquals(DMatrixD1 a, SpatialVectorReadOnly b)
   {
      if (a.numCols != 1 || a.numRows != 6)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");

      a.plus(0, b.getAngularPartX());
      a.plus(1, b.getAngularPartY());
      a.plus(2, b.getAngularPartZ());
      a.plus(3, b.getLinearPartX());
      a.plus(4, b.getLinearPartY());
      a.plus(5, b.getLinearPartZ());
   }

   /**
    * Same as {@link CommonOps_DDRM#mult(DMatrix1Row, DMatrix1Row, DMatrix1Row)}.
    */
   static void mult(ArticulatedBodyInertia a, DMatrix1Row b, DMatrix1Row c)
   {
      if (b.numRows != 6)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");

      c.reshape(6, b.numCols);

      Matrix3D angularInertia = a.getAngularInertia();
      Matrix3D crossInertia = a.getCrossInertia();
      Matrix3D linearInertia = a.getLinearInertia();

      double a00 = angularInertia.getM00();
      double a01 = angularInertia.getM01();
      double a02 = angularInertia.getM02();
      double a10 = angularInertia.getM10();
      double a11 = angularInertia.getM11();
      double a12 = angularInertia.getM12();
      double a20 = angularInertia.getM20();
      double a21 = angularInertia.getM21();
      double a22 = angularInertia.getM22();

      double a03 = crossInertia.getM00();
      double a04 = crossInertia.getM01();
      double a05 = crossInertia.getM02();
      double a13 = crossInertia.getM10();
      double a14 = crossInertia.getM11();
      double a15 = crossInertia.getM12();
      double a23 = crossInertia.getM20();
      double a24 = crossInertia.getM21();
      double a25 = crossInertia.getM22();

      double a33 = linearInertia.getM00();
      double a34 = linearInertia.getM01();
      double a35 = linearInertia.getM02();
      double a43 = linearInertia.getM10();
      double a44 = linearInertia.getM11();
      double a45 = linearInertia.getM12();
      double a53 = linearInertia.getM20();
      double a54 = linearInertia.getM21();
      double a55 = linearInertia.getM22();

      for (int i = 0; i < b.getNumCols(); i++)
      {
         double b0i = b.unsafe_get(0, i);
         double b1i = b.unsafe_get(1, i);
         double b2i = b.unsafe_get(2, i);
         double b3i = b.unsafe_get(3, i);
         double b4i = b.unsafe_get(4, i);
         double b5i = b.unsafe_get(5, i);

         c.unsafe_set(0, i, a00 * b0i + a01 * b1i + a02 * b2i + a03 * b3i + a04 * b4i + a05 * b5i);
         c.unsafe_set(1, i, a10 * b0i + a11 * b1i + a12 * b2i + a13 * b3i + a14 * b4i + a15 * b5i);
         c.unsafe_set(2, i, a20 * b0i + a21 * b1i + a22 * b2i + a23 * b3i + a24 * b4i + a25 * b5i);
         c.unsafe_set(3, i, a03 * b0i + a13 * b1i + a23 * b2i + a33 * b3i + a34 * b4i + a35 * b5i);
         c.unsafe_set(4, i, a04 * b0i + a14 * b1i + a24 * b2i + a43 * b3i + a44 * b4i + a45 * b5i);
         c.unsafe_set(5, i, a05 * b0i + a15 * b1i + a25 * b2i + a53 * b3i + a54 * b4i + a55 * b5i);
      }
   }

   /**
    * Same as {@link CommonOps_DDRM#multTransA(double, DMatrix1Row, DMatrix1Row, DMatrix1Row)}.
    */
   static void multTransA(double alpha, DMatrix1Row a, SpatialVectorReadOnly b, DMatrix1Row c)
   {
      if (a.numRows != 6)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      c.reshape(a.numCols, 1);

      double b0 = b.getAngularPartX();
      double b1 = b.getAngularPartY();
      double b2 = b.getAngularPartZ();
      double b3 = b.getLinearPartX();
      double b4 = b.getLinearPartY();
      double b5 = b.getLinearPartZ();

      for (int aCol = 0; aCol < a.numCols; aCol++)
      {
         double total = a.unsafe_get(0, aCol) * b0;
         total += a.unsafe_get(1, aCol) * b1;
         total += a.unsafe_get(2, aCol) * b2;
         total += a.unsafe_get(3, aCol) * b3;
         total += a.unsafe_get(4, aCol) * b4;
         total += a.unsafe_get(5, aCol) * b5;
         c.set(aCol, 0, alpha * total);
      }
   }

   /**
    * Same as {@link CommonOps_DDRM#multAdd(DMatrix1Row, DMatrix1Row, DMatrix1Row)}.
    */
   static void multAdd(ArticulatedBodyInertia a, DMatrix1Row b, DMatrix1Row c)
   {
      if (b.numRows != 6)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");

      if (c.numRows != 6 || c.numCols != b.numCols)
         throw new MatrixDimensionException("The 'c' is not compatible");

      Matrix3D angularInertia = a.getAngularInertia();
      Matrix3D crossInertia = a.getCrossInertia();
      Matrix3D linearInertia = a.getLinearInertia();

      double a00 = angularInertia.getM00();
      double a01 = angularInertia.getM01();
      double a02 = angularInertia.getM02();
      double a10 = angularInertia.getM10();
      double a11 = angularInertia.getM11();
      double a12 = angularInertia.getM12();
      double a20 = angularInertia.getM20();
      double a21 = angularInertia.getM21();
      double a22 = angularInertia.getM22();

      double a03 = crossInertia.getM00();
      double a04 = crossInertia.getM01();
      double a05 = crossInertia.getM02();
      double a13 = crossInertia.getM10();
      double a14 = crossInertia.getM11();
      double a15 = crossInertia.getM12();
      double a23 = crossInertia.getM20();
      double a24 = crossInertia.getM21();
      double a25 = crossInertia.getM22();

      double a33 = linearInertia.getM00();
      double a34 = linearInertia.getM01();
      double a35 = linearInertia.getM02();
      double a43 = linearInertia.getM10();
      double a44 = linearInertia.getM11();
      double a45 = linearInertia.getM12();
      double a53 = linearInertia.getM20();
      double a54 = linearInertia.getM21();
      double a55 = linearInertia.getM22();

      for (int i = 0; i < b.getNumCols(); i++)
      {
         double b0i = b.unsafe_get(0, i);
         double b1i = b.unsafe_get(1, i);
         double b2i = b.unsafe_get(2, i);
         double b3i = b.unsafe_get(3, i);
         double b4i = b.unsafe_get(4, i);
         double b5i = b.unsafe_get(5, i);

         c.unsafe_set(0, i, c.unsafe_get(0, i) + a00 * b0i + a01 * b1i + a02 * b2i + a03 * b3i + a04 * b4i + a05 * b5i);
         c.unsafe_set(1, i, c.unsafe_get(1, i) + a10 * b0i + a11 * b1i + a12 * b2i + a13 * b3i + a14 * b4i + a15 * b5i);
         c.unsafe_set(2, i, c.unsafe_get(2, i) + a20 * b0i + a21 * b1i + a22 * b2i + a23 * b3i + a24 * b4i + a25 * b5i);
         c.unsafe_set(3, i, c.unsafe_get(3, i) + a03 * b0i + a13 * b1i + a23 * b2i + a33 * b3i + a34 * b4i + a35 * b5i);
         c.unsafe_set(4, i, c.unsafe_get(4, i) + a04 * b0i + a14 * b1i + a24 * b2i + a43 * b3i + a44 * b4i + a45 * b5i);
         c.unsafe_set(5, i, c.unsafe_get(5, i) + a05 * b0i + a15 * b1i + a25 * b2i + a53 * b3i + a54 * b4i + a55 * b5i);
      }
   }
}
