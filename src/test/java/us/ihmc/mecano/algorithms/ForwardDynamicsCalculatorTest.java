package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.mecano.tools.MecanoRandomTools.nextWrench;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class ForwardDynamicsCalculatorTest
{
   private static final int ITERATIONS = 500;
   private static final double ONE_DOF_JOINT_EPSILON = 8.0e-12;
   private static final double FLOATING_JOINT_EPSILON = 4.0e-11;
   private static final double ALL_JOINT_EPSILON = 1.0e-4;

   @Test
   public void testPrismaticJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, i, joints, ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random,
                                                              i,
                                                              joints,
                                                              Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                              ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                 ONE_DOF_JOINT_EPSILON);

         compareAgainstSpatialAccelerationCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testPrismaticJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointTree(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, i, joints, ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random,
                                                              i,
                                                              joints,
                                                              Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                              ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                 ONE_DOF_JOINT_EPSILON);

         compareAgainstSpatialAccelerationCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testRevoluteJointChain() throws Exception
   {
      Random random = new Random(2654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), 2.0 * ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, 0, joints, ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random,
                                                              i,
                                                              joints,
                                                              Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                              ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), 2.0 * ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                 2.0 * ONE_DOF_JOINT_EPSILON);

         compareAgainstSpatialAccelerationCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testRevoluteJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, i, joints, ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random,
                                                              i,
                                                              joints,
                                                              Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                              ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                 2.0 * ONE_DOF_JOINT_EPSILON);

         compareAgainstSpatialAccelerationCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testOneDoFJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, i, joints, ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random,
                                                              i,
                                                              joints,
                                                              Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                              ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                 ONE_DOF_JOINT_EPSILON);

         compareAgainstSpatialAccelerationCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testOneDoFJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, i, joints, ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random,
                                                              i,
                                                              joints,
                                                              Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                              ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                 ONE_DOF_JOINT_EPSILON);

         compareAgainstSpatialAccelerationCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testFloatingRevoluteJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(40) + 1;
         List<Joint> joints = new RandomFloatingRevoluteJointChain(random, numberOfJoints).getJoints();
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), FLOATING_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, i, joints, FLOATING_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random,
                                                              i,
                                                              joints,
                                                              Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                              FLOATING_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), FLOATING_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                 2.0 * FLOATING_JOINT_EPSILON);

         compareAgainstSpatialAccelerationCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), FLOATING_JOINT_EPSILON);
      }
   }

   @Test
   public void testJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(40) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ALL_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, i, joints, ALL_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random,
                                                              i,
                                                              joints,
                                                              Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                              ALL_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ALL_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(numberOfJoints))),
                                                 ALL_JOINT_EPSILON);

         compareAgainstSpatialAccelerationCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ALL_JOINT_EPSILON);
      }
   }

   private static void compareAgainstInverseDynamicsCalculator(Random random, int iteration, List<? extends JointBasics> joints,
                                                               Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                               List<? extends JointReadOnly> jointsToIgnore, double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(multiBodySystemInput);
      inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);

      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      DMatrixRMaj qdd_expected = new DMatrixRMaj(numberOfDoFs, 1);
      int index = 0;
      for (JointBasics joint : joints)
      {
         joint.getJointAcceleration(index, qdd_expected);
         index += joint.getDegreesOfFreedom();
      }

      externalWrenches.forEach(inverseDynamicsCalculator::setExternalWrench);

      inverseDynamicsCalculator.compute();

      inverseDynamicsCalculator.writeComputedJointWrenches(joints);

      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);

      forwardDynamicsCalculator.compute();
      joints.forEach(forwardDynamicsCalculator::writeComputedJointAcceleration);

      DMatrixRMaj qdd_actual = new DMatrixRMaj(numberOfDoFs, 1);
      index = 0;
      for (JointBasics joint : joints)
      {
         joint.getJointAcceleration(index, qdd_actual);
         index += joint.getDegreesOfFreedom();
      }

      boolean areEqual = MatrixFeatures_DDRM.isEquals(qdd_expected, qdd_actual, epsilon);
      if (!areEqual)
      {
         System.out.println("iteration: " + iteration);
         double maxError = 0.0;
         DMatrixRMaj output = new DMatrixRMaj(numberOfDoFs, 3);
         for (int row = 0; row < numberOfDoFs; row++)
         {
            output.set(row, 0, qdd_expected.get(row, 0));
            output.set(row, 1, qdd_actual.get(row, 0));
            double error = qdd_expected.get(row, 0) - qdd_actual.get(row, 0);
            output.set(row, 2, error);
            maxError = Math.max(maxError, Math.abs(error));
         }
         output.print(EuclidCoreIOTools.getStringFormat(9, 6));
         System.out.println("Max error: " + maxError);
      }
      assertTrue(areEqual);

      List<? extends RigidBodyBasics> allRigidBodies = rootBody.subtreeList();

      for (RigidBodyReadOnly rigidBody : allRigidBodies)
      {
         SpatialAccelerationReadOnly expectedAccelerationOfBody = inverseDynamicsCalculator.getAccelerationProvider().getAccelerationOfBody(rigidBody);
         if (expectedAccelerationOfBody == null)
         {
            assertNull(forwardDynamicsCalculator.getAccelerationProvider().getAccelerationOfBody(rigidBody));
         }
         else
         {
            SpatialAcceleration actualAccelerationOfBody = new SpatialAcceleration(forwardDynamicsCalculator.getAccelerationProvider()
                                                                                                            .getAccelerationOfBody(rigidBody));
            actualAccelerationOfBody.changeFrame(expectedAccelerationOfBody.getReferenceFrame());
            MecanoTestTools.assertSpatialAccelerationEquals(expectedAccelerationOfBody, actualAccelerationOfBody, epsilon);

            for (int i = 0; i < 5; i++)
            {
               RigidBodyBasics otherRigidBody = allRigidBodies.get(random.nextInt(allRigidBodies.size()));
               SpatialAccelerationReadOnly expectedRelativeAcceleration = inverseDynamicsCalculator.getAccelerationProvider()
                                                                                                   .getRelativeAcceleration(rigidBody, otherRigidBody);
               if (expectedAccelerationOfBody == null)
               {
                  assertNull(forwardDynamicsCalculator.getAccelerationProvider().getRelativeAcceleration(otherRigidBody, rigidBody));
               }
               else
               {
                  SpatialAccelerationReadOnly actualRelativeAcceleration = forwardDynamicsCalculator.getAccelerationProvider()
                                                                                                    .getRelativeAcceleration(rigidBody, otherRigidBody);
                  MecanoTestTools.assertSpatialAccelerationEquals(expectedRelativeAcceleration, actualRelativeAcceleration, epsilon);
               }
            }
         }
      }
   }

   private static void compareAgainstCompositeRigidBodyMassMatrixCalculator(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      compareAgainstCompositeRigidBodyMassMatrixCalculator(random, iteration, joints, Collections.emptyList(), epsilon);
   }

   private static void compareAgainstCompositeRigidBodyMassMatrixCalculator(Random random, int iteration, List<? extends JointBasics> joints,
                                                                            List<? extends JointBasics> jointsToIgnore, double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      rootBody.updateFramesRecursively();
      MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody, jointsToIgnore);

      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());

      DMatrixRMaj qdd_expected = new DMatrixRMaj(numberOfDoFs, 1);
      int index = 0;

      for (JointBasics joint : input.getJointsToConsider())
      {
         joint.getJointAcceleration(index, qdd_expected);
         joint.setJointAccelerationToZero();
         index += joint.getDegreesOfFreedom();
      }

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(input);
      inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(input);

      inverseDynamicsCalculator.compute();
      massMatrixCalculator.reset();

      inverseDynamicsCalculator.writeComputedJointWrenches(joints);

      DMatrixRMaj massMatrix = massMatrixCalculator.getMassMatrix();
      DMatrixRMaj biasMatrix = new DMatrixRMaj(numberOfDoFs, 1);

      index = 0;
      for (JointReadOnly joint : input.getJointsToConsider())
      {
         joint.getJointTau(index, biasMatrix);
         index += joint.getDegreesOfFreedom();
      }

      DMatrixRMaj tauMatrix = new DMatrixRMaj(numberOfDoFs, 1);

      CommonOps_DDRM.mult(massMatrix, qdd_expected, tauMatrix);
      CommonOps_DDRM.addEquals(tauMatrix, biasMatrix);

      index = 0;
      for (JointBasics joint : input.getJointsToConsider())
      {
         joint.setJointTau(index, tauMatrix);
         index += joint.getDegreesOfFreedom();
      }

      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(input);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);

      forwardDynamicsCalculator.compute();

      DMatrixRMaj qdd_actual = forwardDynamicsCalculator.getJointAccelerationMatrix();

      boolean areEqual = MatrixFeatures_DDRM.isEquals(qdd_expected, qdd_actual, epsilon);
      if (!areEqual)
      {
         LogTools.info("iteration: " + iteration);
         double maxError = 0.0;
         DMatrixRMaj output = new DMatrixRMaj(numberOfDoFs, 3);
         for (int row = 0; row < numberOfDoFs; row++)
         {
            output.set(row, 0, qdd_expected.get(row, 0));
            output.set(row, 1, qdd_actual.get(row, 0));
            double error = qdd_expected.get(row, 0) - qdd_actual.get(row, 0);
            output.set(row, 2, error);
            maxError = Math.max(maxError, Math.abs(error));
         }
         output.print(EuclidCoreIOTools.getStringFormat(9, 6));
         LogTools.info("Max error: " + maxError);
      }
      assertTrue(areEqual);
   }

   private static void compareAgainstSpatialAccelerationCalculator(Random random, int iteration, List<? extends JointBasics> joints,
                                                                   Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                   List<? extends JointReadOnly> jointsToIgnore, double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);
      forwardDynamicsCalculator.compute();
      joints.forEach(forwardDynamicsCalculator::writeComputedJointAcceleration);
      RigidBodyAccelerationProvider fwdDynAccelerationProvider = forwardDynamicsCalculator.getAccelerationProvider();
      RigidBodyAccelerationProvider fwdDynZeroVelocityAccelerationProvider = forwardDynamicsCalculator.getAccelerationProvider(false);

      assertTrue(fwdDynAccelerationProvider.areAccelerationsConsidered());
      assertTrue(fwdDynAccelerationProvider.areVelocitiesConsidered());
      assertTrue(fwdDynZeroVelocityAccelerationProvider.areAccelerationsConsidered());
      assertFalse(fwdDynZeroVelocityAccelerationProvider.areVelocitiesConsidered());

      SpatialAccelerationCalculator accelerationCalculator = new SpatialAccelerationCalculator(rootBody, multiBodySystemInput.getInertialFrame());
      SpatialAccelerationCalculator zeroVelocityAccelerationCalculator = new SpatialAccelerationCalculator(rootBody,
                                                                                                           multiBodySystemInput.getInertialFrame(),
                                                                                                           false);
      accelerationCalculator.setGravitionalAcceleration(gravity);
      zeroVelocityAccelerationCalculator.setGravitionalAcceleration(gravity);

      List<? extends RigidBodyBasics> allRigidBodies = rootBody.subtreeList();

      for (RigidBodyReadOnly rigidBody : allRigidBodies)
      {
         SpatialAccelerationReadOnly expectedAccelerationOfBody = accelerationCalculator.getAccelerationOfBody(rigidBody);
         SpatialAccelerationReadOnly expectedZeroVelocityAccelerationOfBody = zeroVelocityAccelerationCalculator.getAccelerationOfBody(rigidBody);

         if (expectedAccelerationOfBody == null)
         {
            assertNull(fwdDynAccelerationProvider.getAccelerationOfBody(rigidBody));
            assertNull(fwdDynZeroVelocityAccelerationProvider.getAccelerationOfBody(rigidBody));
         }
         else
         {
            SpatialAcceleration actualAccelerationOfBody = new SpatialAcceleration(fwdDynAccelerationProvider.getAccelerationOfBody(rigidBody));
            MecanoTestTools.assertSpatialAccelerationEquals(expectedAccelerationOfBody,
                                                            actualAccelerationOfBody,
                                                            Math.max(1.0, expectedAccelerationOfBody.length()) * epsilon);

            SpatialAcceleration actualZeroVelocityAccelerationOfBody = new SpatialAcceleration(fwdDynZeroVelocityAccelerationProvider.getAccelerationOfBody(rigidBody));
            MecanoTestTools.assertSpatialAccelerationEquals(expectedZeroVelocityAccelerationOfBody,
                                                            actualZeroVelocityAccelerationOfBody,
                                                            Math.max(1.0, expectedZeroVelocityAccelerationOfBody.length()) * epsilon);

            for (int i = 0; i < 5; i++)
            {
               RigidBodyBasics otherRigidBody = allRigidBodies.get(random.nextInt(allRigidBodies.size()));
               SpatialAccelerationReadOnly expectedRelativeAcceleration = accelerationCalculator.getRelativeAcceleration(rigidBody, otherRigidBody);
               SpatialAccelerationReadOnly expectedRelativeZeroVelocityAcceleration = zeroVelocityAccelerationCalculator.getRelativeAcceleration(rigidBody,
                                                                                                                                                 otherRigidBody);

               if (expectedAccelerationOfBody == null)
               {
                  assertNull(fwdDynAccelerationProvider.getRelativeAcceleration(otherRigidBody, rigidBody));
                  assertNull(fwdDynZeroVelocityAccelerationProvider.getRelativeAcceleration(otherRigidBody, rigidBody));
               }
               else
               {
                  SpatialAccelerationReadOnly actualRelativeAcceleration = fwdDynAccelerationProvider.getRelativeAcceleration(rigidBody, otherRigidBody);
                  MecanoTestTools.assertSpatialAccelerationEquals(expectedRelativeAcceleration,
                                                                  actualRelativeAcceleration,
                                                                  Math.max(1.0, expectedRelativeAcceleration.length()) * epsilon);

                  SpatialAccelerationReadOnly actualRelativeZeroVelocityAcceleration = fwdDynZeroVelocityAccelerationProvider.getRelativeAcceleration(rigidBody,
                                                                                                                                                      otherRigidBody);
                  MecanoTestTools.assertSpatialAccelerationEquals(expectedRelativeZeroVelocityAcceleration,
                                                                  actualRelativeZeroVelocityAcceleration,
                                                                  Math.max(1.0, expectedRelativeZeroVelocityAcceleration.length()) * epsilon);
               }
            }
         }
      }
   }

   public static Map<RigidBodyReadOnly, WrenchReadOnly> nextExternalWrenches(Random random, List<? extends JointReadOnly> joints)
   {
      return joints.stream().filter(j -> random.nextBoolean()).map(j -> j.getSuccessor())
                   .collect(Collectors.toMap(b -> b, b -> nextWrench(random, b.getBodyFixedFrame(), b.getBodyFixedFrame())));
   }
}
