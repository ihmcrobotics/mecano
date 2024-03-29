package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.*;

/**
 * Computes the joint torque regressor matrix of a rigid body system based on joint acceleration.
 * <p>
 * In this implementation, we calculate the joint torque regressor matrix via repeated application of a modified
 * Featherstone recursive Newton-Euler method. Focusing on one rigid body, we set the spatial inertia to the parameter to
 * identify, and set the spatial inertias of all other rigid bodies to zero. Solving these modified inverse dynamics via
 * recursive Newton-Euler gives the column of the joint torque regressor matrix for that rigid body / parameter combination.
 * By repeating this over all rigid bodies and parameter combinations, we complete the joint torque regressor matrix.
 * </p>
 * <p>
 * The paper introducing the joint torque regressor matrix is Atkeson, An, & Hollerbach - "Estimation of inertial
 * parameters of manipulator loads and links" (1986): <a href=https://journals.sagepub.com/doi/10.1177/027836498600500306>link</a>
 * </p>
 * <p>
 * The procedure in this implementation is best described in lecture slides by Wensing - "Optimization-Based Robotics,
 * AME60621, Lecture 05 - Dynamics Structure and Identification" (2021):
 * <a href=https://sites.nd.edu/pwensing/teaching/>link</a> [Last accessed 2023-04-21].
 * </p>
 * <p>
 * See the confluence page <a href=https://journals.sagepub.com/doi/10.1177/027836498600500306>link</a> for more details, and copies of the above files.
 * </p>
 * <p>
 * Note on kinematic loops: this calculator does not support kinematic loops yet.
 * </p>
 *
 * @author James Foster
 */
public class JointTorqueRegressorCalculator
{
   /**
    * Defines the multi-body system to use with this calculator.
    */
   private final MultiBodySystemReadOnly input;

   /**
    * The output of this algorithm: the joint torque regressor matrix that relates the inertial parameters of the
    * multi-body system to its joint torques in the current state (position, velocity, acceleration of the joints). For
    * a multi-body system with n DoFs and N rigid bodies, with each rigid body having 10 inertial parameters, the
    * joint torque regressor matrix will have shape n-by-10N.
    */
   private final DMatrixRMaj jointTorqueRegressorMatrix;

   /**
    * A vector containing the inertial parameters of the multi-body system {@link #input}. Assuming a multi-body system
    * consisting of N bodies, with each body having 10 inertial parameters, this vector will be 10N-by-1. This variable
    * is most useful for testing purposes, especially considering that in theory, we shouldn't have (or need) access to
    * the inertial parameters in order to generate the joint torque regressor. However, one can multiply the joint
    * torque regressor by this {@code parameterVector} to see if the torque result matches the result from inverse
    * dynamics.
    */
   private final DMatrixRMaj parameterVector;

   /**
    * The common inverse dynamics calculator that is shared between each recursion step of the algorithm.
    */
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   /**
    * The root of the internal recursive algorithm.
    */
   private final JointTorqueRegressorRecursionStep initialRecursionStep;

   /**
    * Collecting the components of the joint torque regressor matrix is an iterative, rather than recursive process, so here one can use
    * a List rather than a Map.
    */
   private final JointTorqueRegressorRecursionStep[] jointTorqueRegressorRecursionSteps;

   /**
    * If computing blocks of the joint torque regressor matrix for individual bodies, this map can be used to perform body-wise computations.
    */
   private final Map<RigidBodyReadOnly, JointTorqueRegressorRecursionStep> rigidBodyToRecursionStepMap = new HashMap<>();

   /**
    * Static convenience variable for the number of inertial parameters in a rigid body.
    */
   private static final int PARAMETERS_PER_BODY = 10;

   /**
    * Creates a calculator for computing the joint torque regressor for multi-body system associated with the rigid body {@code rootBody}.
    * <p>
    * Do not forget to set the gravitational acceleration so this calculator can properly account for it.
    * </p>
    *
    * @param rootBody the root rigid body of the multi-body system to be evaluated by this calculator.
    * @throws UnsupportedOperationException if the multi-body system associated with the {@code rootBody} contains kinematic loop(s).
    */
   public JointTorqueRegressorCalculator(RigidBodyBasics rootBody)
   {
      this(MultiBodySystemBasics.toMultiBodySystemBasics(rootBody));
   }

   /**
    * Creates a calculator for computing the joint torque regressor for the given multi-body system {@code input}.
    * <p>
    * Do not forget to set the gravitational acceleration so this calculator can properly account for it.
    * </p>
    *
    * @param input the definition of the system to be evaluated by this calculator.
    * @throws UnsupportedOperationException if the {@code input} contains kinematic loop(s).
    */
   public JointTorqueRegressorCalculator(MultiBodySystemBasics input)
   {
      this.input = input;
      inverseDynamicsCalculator = new InverseDynamicsCalculator(input);

      RigidBodyBasics rootBody = input.getRootBody();
      initialRecursionStep = new JointTorqueRegressorRecursionStep(rootBody, null, inverseDynamicsCalculator, null);
      rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
      buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore());
      jointTorqueRegressorRecursionSteps = rigidBodyToRecursionStepMap.values().toArray(new JointTorqueRegressorRecursionStep[0]);

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      int nBodies = jointTorqueRegressorRecursionSteps.length;
      int nParams = PARAMETERS_PER_BODY * nBodies;

      jointTorqueRegressorMatrix = new DMatrixRMaj(nDoFs, nParams);
      parameterVector = new DMatrixRMaj(nParams, 1);
      collectParameterVectorFromRecursionSteps();
   }

   private void buildMultiBodyTree(JointTorqueRegressorRecursionStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<JointTorqueRegressorRecursionStep> recursionSteps = new ArrayList<>();
      // Omit root bodies and elevators from list of recursion steps, we don't want to calculate the regressor with respect to them
      if (!parent.rigidBody.isRootBody())
         recursionSteps.add(parent);

      List<JointReadOnly> childrenJoints = MultiBodySystemTools.sortLoopClosureInChildrenJoints(parent.rigidBody);

      for (JointReadOnly childJoint : childrenJoints)
      {
         if (jointsToIgnore.contains(childJoint))
            continue;

         if (childJoint.isLoopClosure())
         {
            throw new UnsupportedOperationException(
                  getClass().getSimpleName() + ": This calculator does not support kinematic loops. Found loop closure at joint: " + childJoint.getName());
         }

         RigidBodyBasics childBody = (RigidBodyBasics) childJoint.getSuccessor();
         if (childBody != null)
         {
            int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
            JointTorqueRegressorRecursionStep child = new JointTorqueRegressorRecursionStep(childBody, parent, inverseDynamicsCalculator, jointIndices);
            rigidBodyToRecursionStepMap.put(childBody, child);
            buildMultiBodyTree(child, jointsToIgnore);
         }
      }
   }

   /**
    * Computes the n-by-10N joint torque regressor matrix for the current state of the multi-body system {@code input},
    * where n is the number of DoFs and N is the number of rigid bodies in the multi-body system.
    * <p>
    * The position, velocity, and acceleration of the joints are extracted directly from the joint state.
    * </p>
    */
   public void compute()
   {
      // Set all spatial inertias to zero
      initialRecursionStep.setSpatialInertiasToZeroRecursively();

      // Perform an initial forward pass of the inverse dynamics calculator (which roughly corresponds to the forward
      // kinematics). We'll subsequently call the forward pass just one for every rigid body, instead of redundantly
      // calling it once every rigid body / parameter combination
      inverseDynamicsCalculator.initializeJointAccelerationMatrix(null);
      inverseDynamicsCalculator.initialRecursionStep.passOneRecursive();

      // Calculate each column of the regressor, where every column represents a unique rigid body and parameter
      // combination. By stacking all of these columns together in the next step, we'll get the regressor
      initialRecursionStep.calculateRegressorRecursively();

      // Collect regressor results for all recursion steps
      collectJointTorqueRegressorFromRecursionSteps();
   }

   /**
    * Computes the n-by-10 block of the joint torque regressor matrix corresponding to {@code body} for the current state of the multi-body system
    * {@code input}, where n is the number of DoFs.
    * <p>
    * The position, velocity, and acceleration of the joints are extracted directly from the joint state.
    * </p>
    *
    * @param body the rigid body to compute the joint torque regressor matrix block for.
    */
   public void compute(RigidBodyReadOnly body)
   {
      // Set all spatial inertias to zero
      initialRecursionStep.setSpatialInertiasToZeroRecursively();

      // Perform an initial forward pass of the inverse dynamics calculator (which roughly corresponds to the forward
      // kinematics)
      inverseDynamicsCalculator.initializeJointAccelerationMatrix(null);
      inverseDynamicsCalculator.initialRecursionStep.passOneRecursive();

      JointTorqueRegressorRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);

      recursionStep.markUpstreamAsModifiedRecursively();

      // Perform the needed backward passes just for this rigid body
      recursionStep.calculateRegressor();

      // After finishing with this body, clear the modified markers and mark as stale
      recursionStep.clearModifiedMarkersRecursively();
      recursionStep.isUpToDate = false;
   }

   /**
    * Compute the joint torque regressor matrix blocks for the rigid bodies in {@code bodies}.
    * <p>
    * The results will be stored in the recursion steps, see {@link #getJointTorqueRegressorMatrixBlock(RigidBodyReadOnly)}
    * </p>
    *
    * @param bodies the rigid bodies to compute the joint torque regressor matrix blocks for.
    */
   public void compute(RigidBodyReadOnly[] bodies)
   {
      initialRecursionStep.setSpatialInertiasToZeroRecursively();
      // Perform an initial forward pass of the inverse dynamics calculator (which roughly corresponds to the forward
      // kinematics)
      inverseDynamicsCalculator.initializeJointAccelerationMatrix(null);
      inverseDynamicsCalculator.initialRecursionStep.passOneRecursive();

      for (int i = 0; i < bodies.length; ++i)
      {
         JointTorqueRegressorRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(bodies[i]);
         recursionStep.markUpstreamAsModifiedRecursively();

         // Perform the needed backward passes just for this rigid body
         recursionStep.calculateRegressor();

         recursionStep.clearModifiedMarkersRecursively();
         recursionStep.isUpToDate = false;
      }
   }

   /**
    * Computes the joint torque regressor matrix slice corresponding to the rigid body / parameter combination given by {@code body} and {@code basis}.
    * <p>
    * The result is stored in the recursion step, see {@link #getJointTorqueRegressorMatrixSlice(RigidBodyReadOnly, SpatialInertiaBasisOption)}
    * </p>
    *
    * @param body  the rigid body to compute the joint torque regressor matrix slice for.
    * @param basis the basis to compute the joint torque regressor matrix slice for.
    */
   public void compute(RigidBodyReadOnly body, SpatialInertiaBasisOption basis)
   {
      // Set all spatial inertias to zero
      initialRecursionStep.setSpatialInertiasToZeroRecursively();

      // Perform an initial forward pass of the inverse dynamics calculator (which roughly corresponds to the forward
      // kinematics)
      inverseDynamicsCalculator.initializeJointAccelerationMatrix(null);
      inverseDynamicsCalculator.initialRecursionStep.passOneRecursive();

      JointTorqueRegressorRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);

      recursionStep.markUpstreamAsModifiedRecursively();

      // Perform the needed backward passes just for this rigid body
      recursionStep.calculateRegressorColumn(basis);

      initialRecursionStep.clearModifiedMarkersRecursively();
      recursionStep.isUpToDate = false;
   }

   /**
    * Sequentially computes the joint torque regressor matrix slices corresponding to the rigid body / parameter combinations given by {@code body} and the
    * array {@code bases}.
    * <p>
    * The results are stored in the recursion steps, see {@link #getJointTorqueRegressorMatrixSlice(RigidBodyReadOnly, SpatialInertiaBasisOption)}
    * </p>
    *
    * @param body  the rigid body to compute the joint torque regressor matrix slices for.
    * @param bases the bases to compute the joint torque regressor matrix slices for.
    */
   public void compute(RigidBodyReadOnly body, SpatialInertiaBasisOption[] bases)
   {
      // Set all spatial inertias to zero
      initialRecursionStep.setSpatialInertiasToZeroRecursively();

      // Perform an initial forward pass of the inverse dynamics calculator (which roughly corresponds to the forward
      // kinematics)
      inverseDynamicsCalculator.initializeJointAccelerationMatrix(null);
      inverseDynamicsCalculator.initialRecursionStep.passOneRecursive();

      JointTorqueRegressorRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);

      recursionStep.markUpstreamAsModifiedRecursively();

      // Perform the needed backward passes just for the parameters under consideration
      for (int i = 0; i < bases.length; ++i)
         recursionStep.calculateRegressorColumn(bases[i]);

      recursionStep.clearModifiedMarkersRecursively();
      recursionStep.isUpToDate = false;
   }

   /**
    * Assuming all the recursion steps of the algorithm have been run through, this method will collect the results
    * into the overall joint torque regressor matrix.
    */
   private void collectJointTorqueRegressorFromRecursionSteps()
   {
      int i = 0;
      for (JointTorqueRegressorRecursionStep step : jointTorqueRegressorRecursionSteps)
      {
         if (step.rigidBody.getInertia() != null)
         {
            CommonOps_DDRM.insert(step.regressorMatrixBlock, jointTorqueRegressorMatrix, 0, i * PARAMETERS_PER_BODY);
            i++;
         }
      }
   }

   /**
    * NOTE: you should only use this *before* performing any regressor calculations. The reason for this is that regressor
    * calculations involve manipulation of the multibody system's spatial inertia information, corrupting it from the
    * initial input state. For that reason, only use this method before any regressor calculation, preferably as soon
    * as possible after the constructor.
    */
   private void collectParameterVectorFromRecursionSteps()
   {
      int i = 0;
      for (JointTorqueRegressorRecursionStep step : jointTorqueRegressorRecursionSteps)
      {
         if (step.rigidBody.getInertia() != null)
         {
            CommonOps_DDRM.insert(step.parameterVector, parameterVector, i * PARAMETERS_PER_BODY, 0);
            i++;
         }
      }
   }

   /**
    * Set the gravitational acceleration to account for in the multi-body system {@code input}.
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
    * Set the gravitational acceleration to account for in the multi-body system {@code input}.
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
      inverseDynamicsCalculator.setGravitationalAcceleration(gravityX, gravityY, gravityZ);
   }

   /**
    * Gets the 10N-by-1 inertial parameter vector of the multi-body system {@code input}, where N is the number of rigid
    * bodies in the multi-body system.
    * <p>
    * In general, this should mostly be used for debugging purposes, and if called, should be called soon after the
    * constructor, else the result wil be meaningless (e.g. set to all zeros, or all zeros with a few ones).
    * </p>
    * <p>
    * For more details, see {@link #parameterVector}.
    * </p>
    *
    * @return the 10N-by-1 vector of inertial parameters.
    */
   public DMatrixRMaj getParameterVector()
   {
      return parameterVector;
   }

   /**
    * Gets the 10-by-1 slice of the inertial parameter vector corresponding to {@code body}.
    * <p>
    * In general, this should mostly be used for debugging purposes, and if called, should be called soon after the
    * constructor, else the result wil be meaningless (e.g. set to all zeros, or all zeros with a few ones).
    * </p>
    * <p>
    * For more details, see {@link JointTorqueRegressorRecursionStep#parameterVector}.
    * </p>
    *
    * @param body the rigid body to get the inertial parameter vector slice for.
    * @return the 10-by-1 vector of inertial parameters.
    */
   public DMatrixRMaj getParameterVectorSlice(RigidBodyReadOnly body)
   {
      JointTorqueRegressorRecursionStep step = rigidBodyToRecursionStepMap.get(body);
      if (step != null)
         return step.parameterVector;
      else
         return null;
   }

   /**
    * Gets the inertial parameter corresponding to {@code body} and {@code basis}.
    * <p>
    * In general, this should mostly be used for debugging purposes, and if called, should be called soon after the constructor, else the result wil be
    * meaningless (e.g. zero).
    * </p>
    *
    * @param body  the rigid body to get the inertial parameter for.
    * @param basis the basis to get the inertial parameter for.
    * @return the inertial parameter.
    */
   public double getParameter(RigidBodyReadOnly body, SpatialInertiaBasisOption basis)
   {
      JointTorqueRegressorRecursionStep step = rigidBodyToRecursionStepMap.get(body);
      if (step != null)
         return step.parameterVector.get(basis.ordinal());
      else
         return Double.NaN;
   }

   /**
    * Gets the computed n-by-10N joint torque regressor matrix of the multi-body system {@code input}, where n is the
    * number of DoFs and N is the number of rigid bodies in the multi-body system.
    *
    * @return the n-by-10N joint torque regressor matrix.
    */
   public DMatrixRMaj getJointTorqueRegressorMatrix()
   {
      return jointTorqueRegressorMatrix;
   }

   /**
    * Gets the computed n-by-10 block of the joint torque regressor matrix corresponding to {@code body}, where n is the number of
    * DoFs.
    *
    * @param body the rigid body to get the joint torque regressor matrix block for.
    * @return the n-by-10 joint torque regressor matrix.
    */
   public DMatrixRMaj getJointTorqueRegressorMatrixBlock(RigidBodyReadOnly body)
   {
      return rigidBodyToRecursionStepMap.get(body).regressorMatrixBlock;
   }

   /**
    * Gets the computed n-dimensional joint torque regressor matrix slice corresponding to {@code body} and {@code basis}, where n is the number of
    * DoFs.
    *
    * @param body  the rigid body to get the joint torque regressor matrix slice for.
    * @param basis the basis to get the joint torque regressor matrix slice for.
    * @return the n-dimensional joint torque regressor matrix slice.
    */
   public DMatrixRMaj getJointTorqueRegressorMatrixSlice(RigidBodyReadOnly body, SpatialInertiaBasisOption basis)
   {
      CommonOps_DDRM.extract(rigidBodyToRecursionStepMap.get(body).regressorMatrixBlock,
                             0,
                             basis.ordinal(),
                             rigidBodyToRecursionStepMap.get(body).regressorColumn);
      return rigidBodyToRecursionStepMap.get(body).regressorColumn;
   }

   /**
    * Set whether to consider joint accelerations in the internal inverse dynamics calculations.
    *
    * @param considerJointAccelerations whether to consider joint accelerations in the internal inverse dynamics calculations.
    */
   public void setConsiderJointAccelerations(boolean considerJointAccelerations)
   {
      this.inverseDynamicsCalculator.setConsiderJointAccelerations(considerJointAccelerations);
   }

   /**
    * Set whether to consider Coriolis and centrifugal forces in the internal inverse dynamics calculations.
    *
    * @param considerCoriolisAndCentrifugalForces whether to consider Coriolis and centrifugal forces in the internal inverse dynamics calculations.
    */
   public void setConsiderCoriolisAndCentrifugalForces(boolean considerCoriolisAndCentrifugalForces)
   {
      this.inverseDynamicsCalculator.setConsiderCoriolisAndCentrifugalForces(considerCoriolisAndCentrifugalForces);
   }

   /**
    * A {@code SpatialInertia} object is linear with respect to the ten inertial parameters that constitute it.
    * Therefore, we can create a basis for all spatial inertias with respect to these inertial parameters. This
    * enum represents the unit vectors of this basis:
    * <ul>
    * <li> mass
    * <li> first moment of mass (mass multiplied by center of mass offset, also known as "lever")
    * <li> unique components of the moment of inertia
    * </ul>
    */
   public enum SpatialInertiaBasisOption
   {
      M, MCOM_X, MCOM_Y, MCOM_Z, I_XX, I_XY, I_XZ, I_YY, I_YZ, I_ZZ;

      /**
       * Utility method to get all the basis options, for use in loops.
       */
      public static final SpatialInertiaBasisOption[] values = values();

      /**
       * Populates a list with randomly chosen elements of the {@link SpatialInertiaBasisOption} enum.
       * <p>
       * This method generates garbage, and is predominantly intended for testing purposes.
       * </p>
       *
       * @param random the random number generator to use when determining whether to include or exclude {@link SpatialInertiaBasisOption}s.
       * @return a list containing randomly chosen {@link SpatialInertiaBasisOption}s.
       */
      public static List<SpatialInertiaBasisOption> generateRandomBases(Random random)
      {
         List<SpatialInertiaBasisOption> result = new ArrayList<>();
         for (SpatialInertiaBasisOption option : values)
         {
            if (random.nextBoolean())
               result.add(option);
         }
         return result;
      }
   }

   /**
    * A {@code SpatialInertia} object is linearly parameterised with respect to the ten inertial parameters that
    * constitute it. Therefore, we can create a basis for all spatial inertias with respect to these parameters. This
    * class is a simple wrapper around {@code SpatialInertia} that facilitates generation of the spatial inertia basis
    * matrices, usually by the user providing a desired basis in the form of {@code SpatialInertiaParamaterBasisOptions}.
    * <p>
    * Note: as they are bases of the set of spatial inertias, the basis matrices are nearly always non-physical.
    * </p>
    *
    * @author James Foster
    */
   static class SpatialInertiaParameterBasis extends SpatialInertia
   {
      /**
       * The basis matrix is initialised as a zero spatial inertia, inheriting the reference frames of the input
       * rigid body.
       *
       * @param rigidBody the rigid body for which a spatial inertia basis is to be constructed.
       */
      public SpatialInertiaParameterBasis(RigidBodyBasics rigidBody)
      {
         super(rigidBody.getInertia().getBodyFrame(), rigidBody.getInertia().getReferenceFrame());
         setToZero();
      }

      /**
       * Set the spatial inertia to the basis specified by the chosen {@code SpatialInertiaBasisOption}.
       *
       * @param basisOption the basis to set the spatial inertia to.
       */
      public void setBasis(SpatialInertiaBasisOption basisOption)
      {
         setToZero();
         switch (basisOption)
         {
            case M -> setMass(1.0);
            case MCOM_X -> setCenterOfMassOffset(1.0, 0.0, 0.0);
            case MCOM_Y -> setCenterOfMassOffset(0.0, 1.0, 0.0);
            case MCOM_Z -> setCenterOfMassOffset(0.0, 0.0, 1.0);
            case I_XX -> setMomentOfInertia(1.0, 0.0, 0.0);
            case I_XY -> setMomentOfInertia(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
            case I_XZ -> setMomentOfInertia(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
            case I_YY -> setMomentOfInertia(0.0, 1.0, 0.0);
            case I_YZ -> setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
            case I_ZZ -> setMomentOfInertia(0.0, 0.0, 1.0);
         }
      }

      @Override
      public void get(DMatrix matrixToPack)
      {
         get(0, 0, matrixToPack);
      }

      /**
       * This override is subtly different to the original implementation. The original implementation's
       * functionality enforces that the first moments of mass in the upper-right and lower-left blocks of the spatial
       * inertia are coupled to the diagonal matrix block of mass in the lower-right corner, as is correct for
       * physically realisable spatial inertias. However, for the physically unrealisable
       * {@code SpatialInertiaParameterBasis} objects needed here, this coupling must be removed.
       */
      @Override
      public void get(int startRow, int startColumn, DMatrix matrixToPack)
      {
         // Set upper left block corresponding to moment of inertia as normal
         getMomentOfInertia().get(startRow, startColumn, matrixToPack);

         // For upper right block and lower left block, which correspond to CoM offsets, assume mass is 1.0 for
         // the time being
         MecanoTools.toTildeForm(1.0, getCenterOfMassOffset(), false, startRow, startColumn + 3, matrixToPack);
         MecanoTools.toTildeForm(1.0, getCenterOfMassOffset(), true, startRow + 3, startColumn, matrixToPack);

         // For lower right block, which is a diagonal matrix of mass values, as normal
         startRow += 3;
         startColumn += 3;

         for (int i = 0; i < 3; i++)
         {
            matrixToPack.set(startRow + i, startColumn + i, getMass());
            matrixToPack.set(startRow + i, startColumn + (i + 1) % 3, 0.0);
            matrixToPack.set(startRow + i, startColumn + (i + 2) % 3, 0.0);
         }
      }
   }

   /**
    * Represents a single recursion step for the two passes needed for regressor calculation for a rigid body.
    *
    * @author James Foster
    */
   private class JointTorqueRegressorRecursionStep
   {
      /**
       * The rigid body belonging to this recursion step.
       */
      private final RigidBodyBasics rigidBody;

      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid body.
       */
      private final JointTorqueRegressorRecursionStep parent;

      /**
       * A list containing the recursion steps holding onto the direct successors of this recursion step's
       * rigid body.
       */
      private final List<JointTorqueRegressorRecursionStep> children = new ArrayList<>();

      /**
       * A vector containing the 10 inertial parameters of the {@code rigidBody} in this recursion step. This is created on construction, and is not modified
       * during the algorithm's run.
       */
      private final DMatrixRMaj parameterVector;

      /**
       * The inverse dynamics calculator used for regressor calculation.
       * <p>
       * Note: this calculator is shared throughout the whole algorithm, including all of the recursion steps. As
       * such, be wary of arbitrarily changing its state.
       * </p>
       */
      private InverseDynamicsCalculator inverseDynamicsCalculator;

      /**
       * Intermediate variable responsible for holding on to the spatial inertia parameter basis currently being used
       * for other calculations.
       */
      private SpatialInertiaParameterBasis spatialInertiaParameterBasis;

      /**
       * Intermediate variable to hold a column of the regressor matrix. This column will correspond to a
       * {@code rigidBody} and {@code SpatialInertiaBasisOption} combination. It is an n-by-1 vector, where n
       * is the number of DoFs of the multi-body system {@code input}.
       */
      private DMatrixRMaj regressorColumn;

      /**
       * Intermediate variable the block of the regressor matrix corresponding to this {@code rigidBody}. It is an
       * n-by-10 matrix, where n is the number of DoFs of the multi-body system {@code input}.
       */
      private DMatrixRMaj regressorMatrixBlock;

      /**
       * The recursion step in the internal {@code inverseDynamicsCalculator} that corresponds to this {@code rigidBody}.
       */
      private InverseDynamicsCalculator.RecursionStep inverseDynamicsRecursionStep;

      /**
       * Boolean indicating whether the {@code rigidBody} in this recursion step is on a modified branch of the
       * multi-body system. It is classed as modified if it is upstream of another rigid body that is having its
       * spatial inertia modified.
       */
      boolean isOnModifiedBranch;

      /**
       * Boolean indicating whether the inverse dynamics calculation for the {@code rigidBody} in this recursion step
       * is up-to-date. If a link is either downstream or on a separate upstream branch from the link currently being
       * modified, then the inverse dynamics calculation only needs to be performed once. A link is therefore stale
       * on two occasions: i) on initialisation of the algorithm; ii) when the link has just finished its turn of
       * having the spatial inertia modified, it needs one solve to refresh it.
       */
      boolean isUpToDate;

      public JointTorqueRegressorRecursionStep(RigidBodyBasics rigidBody,
                                               JointTorqueRegressorRecursionStep parent,
                                               InverseDynamicsCalculator inverseDynamicsCalculator,
                                               int[] jointsToIgnore)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.parameterVector = new DMatrixRMaj(PARAMETERS_PER_BODY, 1);

         if (parent != null)
         {
            parent.children.add(this);
            spatialInertiaToParameterVector(rigidBody.getInertia(), parameterVector);
            this.inverseDynamicsCalculator = inverseDynamicsCalculator;
            spatialInertiaParameterBasis = new SpatialInertiaParameterBasis(this.rigidBody);
            regressorColumn = new DMatrixRMaj(inverseDynamicsCalculator.getJointTauMatrix().numRows, 1);
            this.inverseDynamicsRecursionStep = inverseDynamicsCalculator.rigidBodyToRecursionStepMap.get(rigidBody);
            regressorMatrixBlock = new DMatrixRMaj(inverseDynamicsCalculator.getJointTauMatrix().numRows, PARAMETERS_PER_BODY);
            isOnModifiedBranch = false;
            isUpToDate = false;
         }
      }

      /**
       * First pass: set the spatial inertias of all the rigid bodies to zero.
       */
      public void setSpatialInertiasToZeroRecursively()
      {
         // Only make changes if the rigid body of concern isn't a null body like the root / elevator
         if (rigidBody.getInertia() != null)
            rigidBody.getInertia().setToZero();

         for (JointTorqueRegressorRecursionStep child : children)
         {
            child.setSpatialInertiasToZeroRecursively();
         }
      }

      /**
       * Second pass: for each rigid body starting from the leaves to the root, iterate over and perform inverse dynamics for all the spatial inertia
       * parameter bases.
       */
      public void calculateRegressorRecursively()
      {
         for (JointTorqueRegressorRecursionStep child : children)
         {
            child.calculateRegressorRecursively();
         }

         calculateRegressor();

         // When finished iterating over this link, clear all rigid bodies of modification markers, starting from
         // the root body
         initialRecursionStep.clearModifiedMarkersRecursively();
         // This rigid body is now stale, and requires one more solve to get it up to date
         isUpToDate = false;
      }

      /**
       * Inner loop of second pass specific to a given rigid body.
       * <p>
       * Also used independently of its recursive parent method {@link #calculateRegressorRecursively()} in order to calculate regressor blocks for specific
       * rigid bodies, see {@link #compute(RigidBodyReadOnly)}.
       * </p>
       */
      public void calculateRegressor()
      {
         // Only calculate if the rigid body of concern isn't a null body like the root / elevator
         if (rigidBody.getInertia() != null)
         {
            markUpstreamAsModifiedRecursively();
            for (SpatialInertiaBasisOption basis : SpatialInertiaBasisOption.values)
               calculateRegressorColumn(basis);

            // When done with this rigid body, set the spatial inertia back to zero for subsequent iterations / recursions
            rigidBody.getInertia().setToZero();
         }
      }

      /**
       * Inner loop of second pass specific to a given rigid body and parameter combination.
       * <p>
       * Also used independently of its recursive parent method {@link #calculateRegressorRecursively()} and {@link #compute(RigidBodyReadOnly)} in order to
       * calculate regressor slices for specific spatial inertia basis options, see {@link #compute(RigidBodyReadOnly, SpatialInertiaBasisOption)}.
       * </p>
       *
       * @param basis the {@code SpatialInertiaBasisOption} to calculate the regressor column for.
       */
      public void calculateRegressorColumn(SpatialInertiaBasisOption basis)
      {
         // Set spatial inertia of this rigid body to be the desired basis
         spatialInertiaParameterBasis.setBasis(basis);
         rigidBody.getInertia().set(spatialInertiaParameterBasis);

         // The forward pass of the inverse dynamics has already been called, only call the backward pass
         rigidBody.getInertia().set(spatialInertiaParameterBasis);
         initialRecursionStep.calculateInverseDynamicsToRootRecursively();
         regressorColumn.set(inverseDynamicsCalculator.getJointTauMatrix());
         setRegressorMatrixColumn(regressorColumn, basis);
      }

      public void calculateInverseDynamicsToRootRecursively()
      {
         for (JointTorqueRegressorRecursionStep child : children)
         {
            child.calculateInverseDynamicsToRootRecursively();
         }
         // Null check on root body
         if (rigidBody.getInertia() != null)
         {
            // If the rigid body is upstream of a modified link, re-run the backward pass. If the rigid body is not
            // up-to-date (happens i. at initialisation; and ii. when we leave a rigid body), we need to re-run the
            // backward pass.
            if (isOnModifiedBranch || !isUpToDate)
            {
               // If the rigid body is not on the modified branch, then after the inverse dynamics solve, it will
               // be up-to-date and won't change
               if (!isOnModifiedBranch)
                  isUpToDate = true;
               inverseDynamicsRecursionStep.passTwo();
            }
         }
      }

      /**
       * Set the {@code isOnModifiedBranch} booleans for the upstream branch of this recursion step to {@code true}.
       * The upstream branch is found by recursively going through the parent of the current recursion step.
       */
      public void markUpstreamAsModifiedRecursively()
      {
         if (rigidBody.getInertia() != null)
         {
            isOnModifiedBranch = true;
            parent.markUpstreamAsModifiedRecursively();
         }
      }

      /**
       * Set the {@code isOnModifiedBranch} booleans for all children subtree recursion steps to {@code false}.
       */
      public void clearModifiedMarkersRecursively()
      {
         for (JointTorqueRegressorRecursionStep child : children)
         {
            child.clearModifiedMarkersRecursively();
         }
         isOnModifiedBranch = false;
      }

      /**
       * Utility method for inserting a calculated column (vector) of the regressor corresponding to a chosen
       * {@code SpatialInertiaBasisOption} to the appropriate place in this recursion step's
       * {@code regressorMatrixBlock}.
       * <p>
       * The block of the joint torque regressor matrix corresponding to a given rigid body will have ten columns. We
       * choose to order them as they are ordered in the enum {@link SpatialInertiaBasisOption}.
       * </p>
       *
       * @param regressorColumn the n-by-1 vector representing the column for parameter {@code basis}.
       * @param basis           the {@code SpatialInertiaBasisOption} parameter basis, which informs the ordering and
       *                        placement of regressor column entries.
       */
      public void setRegressorMatrixColumn(DMatrixRMaj regressorColumn, SpatialInertiaBasisOption basis)
      {
         CommonOps_DDRM.insert(regressorColumn, regressorMatrixBlock, 0, basis.ordinal());
      }

      /**
       * Utility method for converting SpatialInertia objects to matrix form.
       */
      private void spatialInertiaToParameterVector(SpatialInertiaBasics spatialInertia, DMatrixRMaj parameterVectorToPack)
      {
         parameterVectorToPack.set(0, 0, spatialInertia.getMass());
         parameterVectorToPack.set(1, 0, spatialInertia.getCenterOfMassOffset().getX());
         parameterVectorToPack.set(2, 0, spatialInertia.getCenterOfMassOffset().getY());
         parameterVectorToPack.set(3, 0, spatialInertia.getCenterOfMassOffset().getZ());
         parameterVectorToPack.set(4, 0, spatialInertia.getMomentOfInertia().getM00());  // Ixx
         parameterVectorToPack.set(5, 0, spatialInertia.getMomentOfInertia().getM01());  // Ixy
         parameterVectorToPack.set(6, 0, spatialInertia.getMomentOfInertia().getM02());  // Ixz
         parameterVectorToPack.set(7, 0, spatialInertia.getMomentOfInertia().getM11());  // Iyy
         parameterVectorToPack.set(8, 0, spatialInertia.getMomentOfInertia().getM12());  // Iyz
         parameterVectorToPack.set(9, 0, spatialInertia.getMomentOfInertia().getM22());  // Izz
      }
   }
}
