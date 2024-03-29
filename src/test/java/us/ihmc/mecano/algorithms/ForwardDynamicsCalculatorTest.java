package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator.JointSourceMode;
import us.ihmc.mecano.algorithms.TablePrinter.Alignment;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.*;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;

import java.util.*;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.mecano.tools.MecanoRandomTools.nextWrench;
import static us.ihmc.mecano.tools.MecanoTestTools.assertWrenchEquals;

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

   @Test
   public void testJointAccelerationSourceWithZeroVelocityAcceleration()
   {
      Random random = new Random(2343);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(40) + 1;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         int numberOfLockedJoints = numberOfJoints == 1 ? 1 : random.nextInt(numberOfJoints - 1) + 1;
         List<JointBasics> jointsToLock = new ArrayList<>(joints);
         Collections.shuffle(jointsToLock, random);
         while (jointsToLock.size() > numberOfLockedJoints)
            jointsToLock.remove(jointsToLock.size() - 1);

         jointsToLock.forEach(j -> j.setJointTwistToZero());
         jointsToLock.forEach(j -> j.setJointAccelerationToZero());

         InverseDynamicsCalculator invDyn = new InverseDynamicsCalculator(input);
         invDyn.setGravitationalAcceleration(-9.81);
         invDyn.compute();
         invDyn.writeComputedJointWrenches(joints);

         DMatrixRMaj expected_qdd = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
         DMatrixRMaj expected_tau = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.ACCELERATION, expected_qdd);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, expected_tau);

         ForwardDynamicsCalculator fwdDyn = new ForwardDynamicsCalculator(input);
         fwdDyn.setGravitationalAcceleration(-9.81);

         jointsToLock.forEach(j ->
                              {
                                 j.setJointTauToZero(); // Make sure the forward dynamics calculator does not use that info.
                                 fwdDyn.setJointSourceMode(j, JointSourceMode.ACCELERATION_SOURCE);
                              });

         fwdDyn.compute();

         DMatrixRMaj actual_qdd = fwdDyn.getJointAccelerationMatrix();
         DMatrixRMaj actual_tau = fwdDyn.getJointTauMatrix();

         try
         {
            MecanoTestTools.assertDMatrixEquals(expected_qdd, actual_qdd, 1.0e-12 * Math.max(1.0, CommonOps_DDRM.elementMax(expected_qdd)));
            MecanoTestTools.assertDMatrixEquals(expected_tau, actual_tau, 1.0e-12 * Math.max(1.0, CommonOps_DDRM.elementMax(expected_tau)));
         }
         catch (Throwable e)
         {
            DMatrixRMaj diff_qdd = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
            DMatrixRMaj diff_tau = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
            DMatrixRMaj lockedMatrix = new DMatrixRMaj(input.getNumberOfDoFs(), 1);

            CommonOps_DDRM.subtract(expected_tau, actual_tau, diff_tau);
            CommonOps_DDRM.subtract(expected_qdd, actual_qdd, diff_qdd);
            CommonOps_DDRM.abs(diff_tau);
            CommonOps_DDRM.abs(diff_qdd);

            int row = 0;

            for (JointBasics joint : joints)
            {
               if (jointsToLock.contains(joint))
               {
                  for (int dof = 0; dof < joint.getDegreesOfFreedom(); dof++)
                     lockedMatrix.set(row + dof, 0, 1.0);
               }
               row += joint.getDegreesOfFreedom();
            }

            TablePrinter tablePrinter = new TablePrinter();
            tablePrinter.setColumnSeparator(" \t ");
            tablePrinter.setRow(0, "Joint", "Exp qdd", "Act qdd", "Diff qdd", "Exp tau", "Act tau", "Diff tau", "Locked");
            int col = 1;
            tablePrinter.setSubTable(1, col++, expected_qdd);
            tablePrinter.setSubTable(1, col++, actual_qdd);
            tablePrinter.setSubTable(1, col++, diff_qdd);
            tablePrinter.setSubTable(1, col++, expected_tau);
            tablePrinter.setSubTable(1, col++, actual_tau);
            tablePrinter.setSubTable(1, col++, diff_tau);
            tablePrinter.setSubTable(1, col++, lockedMatrix);

            row = 1;

            for (JointReadOnly joint : input.getJointMatrixIndexProvider().getIndexedJointsInOrder())
            {
               for (int dof = 0; dof < joint.getDegreesOfFreedom(); dof++)
               {
                  tablePrinter.setCell(row, 0, joint.getName(), Alignment.LEFT);
                  row++;
               }
            }
            LogTools.info("\n" + tablePrinter.toString());
            LogTools.info("Max qdd error: {}, max tau error: {}", CommonOps_DDRM.elementMax(diff_qdd), CommonOps_DDRM.elementMax(diff_tau));

            throw e;
         }
      }
   }

   @Test
   public void testJointMixedSourceModeGeneral()
   {
      Random random = new Random(2343);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(40) + 1;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         InverseDynamicsCalculator invDyn = new InverseDynamicsCalculator(input);
         invDyn.setGravitationalAcceleration(-9.81);
         invDyn.compute();
         invDyn.writeComputedJointWrenches(joints);

         DMatrixRMaj expected_qdd = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
         DMatrixRMaj expected_tau = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.ACCELERATION, expected_qdd);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, expected_tau);

         ForwardDynamicsCalculator fwdDyn = new ForwardDynamicsCalculator(input);
         fwdDyn.setGravitationalAcceleration(-9.81);

         int numberOfLockedJoints = numberOfJoints == 1 ? 1 : random.nextInt(numberOfJoints - 1) + 1;
         List<JointBasics> jointsToLock = new ArrayList<>(joints);
         Collections.shuffle(jointsToLock, random);
         while (jointsToLock.size() > numberOfLockedJoints)
            jointsToLock.remove(jointsToLock.size() - 1);
         jointsToLock.forEach(j ->
                              {
                                 j.setJointTauToZero(); // Make sure the forward dynamics calculator does not use that info.
                                 fwdDyn.setJointSourceMode(j, JointSourceMode.ACCELERATION_SOURCE);
                              });

         fwdDyn.compute();

         DMatrixRMaj actual_qdd = fwdDyn.getJointAccelerationMatrix();
         DMatrixRMaj actual_tau = fwdDyn.getJointTauMatrix();

         try
         {
            MecanoTestTools.assertDMatrixEquals(expected_qdd, actual_qdd, 1.0e-12 * Math.max(1.0, CommonOps_DDRM.elementMax(expected_qdd)));
            MecanoTestTools.assertDMatrixEquals(expected_tau, actual_tau, 1.0e-12 * Math.max(1.0, CommonOps_DDRM.elementMax(expected_tau)));
         }
         catch (Throwable e)
         {
            DMatrixRMaj diff_qdd = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
            DMatrixRMaj diff_tau = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
            DMatrixRMaj lockedMatrix = new DMatrixRMaj(input.getNumberOfDoFs(), 1);

            CommonOps_DDRM.subtract(expected_tau, actual_tau, diff_tau);
            CommonOps_DDRM.subtract(expected_qdd, actual_qdd, diff_qdd);
            CommonOps_DDRM.abs(diff_tau);
            CommonOps_DDRM.abs(diff_qdd);

            int row = 0;

            for (JointBasics joint : joints)
            {
               if (jointsToLock.contains(joint))
               {
                  for (int dof = 0; dof < joint.getDegreesOfFreedom(); dof++)
                     lockedMatrix.set(row + dof, 0, 1.0);
               }
               row += joint.getDegreesOfFreedom();
            }

            TablePrinter tablePrinter = new TablePrinter();
            tablePrinter.setColumnSeparator(" \t ");
            tablePrinter.setRow(0, "Joint", "Exp qdd", "Act qdd", "Diff qdd", "Exp tau", "Act tau", "Diff tau", "Locked");
            int col = 1;
            tablePrinter.setSubTable(1, col++, expected_qdd);
            tablePrinter.setSubTable(1, col++, actual_qdd);
            tablePrinter.setSubTable(1, col++, diff_qdd);
            tablePrinter.setSubTable(1, col++, expected_tau);
            tablePrinter.setSubTable(1, col++, actual_tau);
            tablePrinter.setSubTable(1, col++, diff_tau);
            tablePrinter.setSubTable(1, col++, lockedMatrix);

            row = 1;

            for (JointReadOnly joint : input.getJointMatrixIndexProvider().getIndexedJointsInOrder())
            {
               for (int dof = 0; dof < joint.getDegreesOfFreedom(); dof++)
               {
                  tablePrinter.setCell(row, 0, joint.getName(), Alignment.LEFT);
                  row++;
               }
            }
            LogTools.info("\n" + tablePrinter.toString());
            LogTools.info("Max qdd error: {}, max tau error: {}", CommonOps_DDRM.elementMax(diff_qdd), CommonOps_DDRM.elementMax(diff_tau));

            throw e;
         }
      }
   }

   @Test
   public void testAddEquals()
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      {
         DMatrixRMaj expected_a = RandomMatrices_DDRM.rectangle(6, 1, -1.0, 1.0, random);
         DMatrixRMaj actual_a = new DMatrixRMaj(expected_a);
         SpatialVector b_vector = MecanoRandomTools.nextSpatialVector(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj b_matrix = new DMatrixRMaj(6, 1);
         b_vector.get(b_matrix);

         ForwardDynamicsCalculator.addEquals(actual_a, b_vector);
         CommonOps_DDRM.addEquals(expected_a, b_matrix);

         MecanoTestTools.assertDMatrixEquals(actual_a, expected_a, 1.0e-12);
      }
   }

   @Test
   public void testMult()
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int aNumCol = random.nextInt(10);
         ArticulatedBodyInertia a_mecano = new ArticulatedBodyInertia();
         a_mecano.getAngularInertia().set(EuclidCoreRandomTools.nextMatrix3D(random));
         a_mecano.getCrossInertia().set(EuclidCoreRandomTools.nextMatrix3D(random));
         a_mecano.getLinearInertia().set(EuclidCoreRandomTools.nextMatrix3D(random));
         DMatrixRMaj a_ejml = new DMatrixRMaj(6, 6);
         a_mecano.get(a_ejml);
         DMatrixRMaj b = RandomMatrices_DDRM.rectangle(6, aNumCol, -1.0, 1.0, random);
         DMatrixRMaj expected_c = new DMatrixRMaj(1, 1);
         DMatrixRMaj actual_c = new DMatrixRMaj(1, 1);

         ForwardDynamicsCalculator.mult(a_mecano, b, actual_c);
         CommonOps_DDRM.mult(a_ejml, b, expected_c);

         MecanoTestTools.assertDMatrixEquals(expected_c, actual_c, 1.0e-12);
      }
   }

   @Test
   public void testMultTransA()
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int aNumCol = random.nextInt(10);
         double alpha = EuclidCoreRandomTools.nextDouble(random);
         DMatrixRMaj a = RandomMatrices_DDRM.rectangle(6, aNumCol, -1.0, 1.0, random);
         SpatialVector b_vector = MecanoRandomTools.nextSpatialVector(random, ReferenceFrame.getWorldFrame());
         DMatrixRMaj b_matrix = new DMatrixRMaj(6, 1);
         b_vector.get(b_matrix);
         DMatrixRMaj expected_c = new DMatrixRMaj(1, 1);
         DMatrixRMaj actual_c = new DMatrixRMaj(1, 1);

         ForwardDynamicsCalculator.multTransA(alpha, a, b_vector, actual_c);
         CommonOps_DDRM.multTransA(alpha, a, b_matrix, expected_c);

         MecanoTestTools.assertDMatrixEquals(expected_c, actual_c, 1.0e-12);
      }
   }

   @Test
   public void testMultAdd()
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int aNumCol = random.nextInt(10);
         ArticulatedBodyInertia a_mecano = new ArticulatedBodyInertia();
         a_mecano.getAngularInertia().set(EuclidCoreRandomTools.nextMatrix3D(random));
         a_mecano.getCrossInertia().set(EuclidCoreRandomTools.nextMatrix3D(random));
         a_mecano.getLinearInertia().set(EuclidCoreRandomTools.nextMatrix3D(random));
         DMatrixRMaj a_ejml = new DMatrixRMaj(6, 6);
         a_mecano.get(a_ejml);
         DMatrixRMaj b = RandomMatrices_DDRM.rectangle(6, aNumCol, -1.0, 1.0, random);
         DMatrixRMaj expected_c = RandomMatrices_DDRM.rectangle(6, b.getNumCols(), -1, 1, random);
         DMatrixRMaj actual_c = new DMatrixRMaj(expected_c);

         ForwardDynamicsCalculator.multAdd(a_mecano, b, actual_c);
         CommonOps_DDRM.multAdd(a_ejml, b, expected_c);

         MecanoTestTools.assertDMatrixEquals(expected_c, actual_c, 1.0e-12);
      }
   }

   @Test
   public void testModifiedRigidBodyParameters()
   {
      Random random = new Random(120398);

      // No external wrenches, No joints to ignore
      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(40) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody, Collections.emptyList());
         rootBody.updateFramesRecursively();

         double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(multiBodySystemInput);
         inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
         ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
         forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
         CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(multiBodySystemInput);
         SpatialAccelerationCalculator accelerationCalculator = new SpatialAccelerationCalculator(rootBody, multiBodySystemInput.getInertialFrame());
         SpatialAccelerationCalculator zeroVelocityAccelerationCalculator = new SpatialAccelerationCalculator(rootBody,
                                                                                                              multiBodySystemInput.getInertialFrame(),
                                                                                                              false);
         accelerationCalculator.setGravitionalAcceleration(gravity);
         zeroVelocityAccelerationCalculator.setGravitionalAcceleration(gravity);

         compareAgainstExistingInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), ALL_JOINT_EPSILON,
                                                         forwardDynamicsCalculator,
                                                         inverseDynamicsCalculator);
         compareAgainstExistingCompositeRigidBodyMassMatrixCalculator(random,
                                                                      i,
                                                                      joints,
                                                                      multiBodySystemInput,
                                                                      ALL_JOINT_EPSILON,
                                                                      forwardDynamicsCalculator,
                                                                      inverseDynamicsCalculator,
                                                                      massMatrixCalculator);
         compareAgainstExistingSpatialAccelerationCalculator(random,
                                                             i,
                                                             joints,
                                                             Collections.emptyMap(),
                                                             ALL_JOINT_EPSILON,
                                                             forwardDynamicsCalculator,
                                                             accelerationCalculator,
                                                             zeroVelocityAccelerationCalculator);

         RigidBodyBasics body = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));

         compareAgainstExistingInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), ALL_JOINT_EPSILON,
                                                         forwardDynamicsCalculator,
                                                         inverseDynamicsCalculator);
         compareAgainstExistingCompositeRigidBodyMassMatrixCalculator(random,
                                                                      i,
                                                                      joints,
                                                                      multiBodySystemInput,
                                                                      ALL_JOINT_EPSILON,
                                                                      forwardDynamicsCalculator,
                                                                      inverseDynamicsCalculator,
                                                                      massMatrixCalculator);
         compareAgainstExistingSpatialAccelerationCalculator(random,
                                                             i,
                                                             joints,
                                                             Collections.emptyMap(),
                                                             ALL_JOINT_EPSILON,
                                                             forwardDynamicsCalculator,
                                                             accelerationCalculator,
                                                             zeroVelocityAccelerationCalculator);
      }

      // External wrenches, No joints to ignore
      for (int j = 0; j < ITERATIONS; j++)
      {
         int numberOfJoints = random.nextInt(40) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody, Collections.emptyList());
         rootBody.updateFramesRecursively();

         double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(multiBodySystemInput);
         inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
         ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
         forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
         SpatialAccelerationCalculator accelerationCalculator = new SpatialAccelerationCalculator(rootBody, multiBodySystemInput.getInertialFrame());
         SpatialAccelerationCalculator zeroVelocityAccelerationCalculator = new SpatialAccelerationCalculator(rootBody,
                                                                                                              multiBodySystemInput.getInertialFrame(),
                                                                                                              false);
         accelerationCalculator.setGravitionalAcceleration(gravity);
         zeroVelocityAccelerationCalculator.setGravitionalAcceleration(gravity);

         Map<RigidBodyReadOnly, WrenchReadOnly> wrenchesToApply = nextExternalWrenches(random, joints);

         compareAgainstExistingInverseDynamicsCalculator(random, j, joints, wrenchesToApply, ALL_JOINT_EPSILON,
                                                         forwardDynamicsCalculator,
                                                         inverseDynamicsCalculator);
         // No CompositeRigidBodyMassMatrixCalculator, N/A for wrenches
         compareAgainstExistingSpatialAccelerationCalculator(random, j,
                                                             joints,
                                                             wrenchesToApply,
                                                             ALL_JOINT_EPSILON,
                                                             forwardDynamicsCalculator,
                                                             accelerationCalculator,
                                                             zeroVelocityAccelerationCalculator);

         RigidBodyBasics body = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));

         compareAgainstExistingInverseDynamicsCalculator(random, j, joints, wrenchesToApply, ALL_JOINT_EPSILON,
                                                         forwardDynamicsCalculator,
                                                         inverseDynamicsCalculator);
         // No CompositeRigidBodyMassMatrixCalculator, N/A for wrenches
         compareAgainstExistingSpatialAccelerationCalculator(random, j,
                                                             joints,
                                                             wrenchesToApply,
                                                             ALL_JOINT_EPSILON,
                                                             forwardDynamicsCalculator,
                                                             accelerationCalculator,
                                                             zeroVelocityAccelerationCalculator);
      }

      // No external wrenches, random joint selected to ignore
      for (int k = 0; k < ITERATIONS; k++)
      {
         int numberOfJoints = random.nextInt(40) + 2;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody, Collections.singletonList(joints.get(random.nextInt(numberOfJoints))));
         rootBody.updateFramesRecursively();

         double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(multiBodySystemInput);
         inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
         ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
         forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
         CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(multiBodySystemInput);

         compareAgainstExistingInverseDynamicsCalculator(random, k, joints, Collections.emptyMap(), ALL_JOINT_EPSILON,
                                                         forwardDynamicsCalculator,
                                                         inverseDynamicsCalculator);
         compareAgainstExistingCompositeRigidBodyMassMatrixCalculator(random,
                                                                      k,
                                                                      joints,
                                                                      multiBodySystemInput,
                                                                      ALL_JOINT_EPSILON,
                                                                      forwardDynamicsCalculator,
                                                                      inverseDynamicsCalculator,
                                                                      massMatrixCalculator);
         // No SpatialAccelerationCalculator, N/A for ignored joints

         RigidBodyBasics body = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));

         compareAgainstExistingInverseDynamicsCalculator(random, k, joints, Collections.emptyMap(), ALL_JOINT_EPSILON,
                                                         forwardDynamicsCalculator,
                                                         inverseDynamicsCalculator);
         compareAgainstExistingCompositeRigidBodyMassMatrixCalculator(random,
                                                                      k,
                                                                      joints,
                                                                      multiBodySystemInput,
                                                                      ALL_JOINT_EPSILON,
                                                                      forwardDynamicsCalculator,
                                                                      inverseDynamicsCalculator,
                                                                      massMatrixCalculator);
         // No SpatialAccelerationCalculator, N/A for ignored joints
      }
   }

   private static void compareAgainstInverseDynamicsCalculator(Random random,
                                                               int iteration,
                                                               List<? extends JointBasics> joints,
                                                               Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                               List<? extends JointReadOnly> jointsToIgnore,
                                                               double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(multiBodySystemInput);
      inverseDynamicsCalculator.setGravitationalAcceleration(gravity);
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
      forwardDynamicsCalculator.setGravitationalAcceleration(gravity);

      compareAgainstExistingInverseDynamicsCalculator(random, iteration, joints, externalWrenches, epsilon, forwardDynamicsCalculator, inverseDynamicsCalculator);
   }

   private static void compareAgainstExistingInverseDynamicsCalculator(Random random,
                                                                       int iteration,
                                                                       List<? extends JointBasics> joints,
                                                                       Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                       double epsilon,
                                                                       ForwardDynamicsCalculator forwardDynamicsCalculator,
                                                                       InverseDynamicsCalculator inverseDynamicsCalculator)
   {
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

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      List<? extends RigidBodyBasics> allRigidBodies = rootBody.subtreeList();

      // Test the acceleration of each body.
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
            assertAccelerationEquals(expectedAccelerationOfBody, actualAccelerationOfBody, epsilon);

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
                  assertAccelerationEquals(expectedRelativeAcceleration, actualRelativeAcceleration, epsilon);
               }
            }
         }
      }

      List<JointReadOnly> reverseJointList = new ArrayList<>(joints);
      Collections.reverse(reverseJointList);

      // Test the wrench of each joint.
      for (JointReadOnly joint : joints)
      {
         WrenchReadOnly expectedWrench = inverseDynamicsCalculator.getComputedJointWrench(joint);
         if (expectedWrench == null)
         {
            assertNull(forwardDynamicsCalculator.getJointWrench(joint));
         }
         else
         {
            Wrench actualWrench = new Wrench(forwardDynamicsCalculator.getJointWrench(joint));
            actualWrench.changeFrame(expectedWrench.getReferenceFrame());
            assertWrenchEquals(expectedWrench, actualWrench, Math.max(1.0, expectedWrench.length()) * epsilon);
         }
      }
   }

   private static void compareAgainstCompositeRigidBodyMassMatrixCalculator(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      compareAgainstCompositeRigidBodyMassMatrixCalculator(random, iteration, joints, Collections.emptyList(), epsilon);
   }

   private static void compareAgainstCompositeRigidBodyMassMatrixCalculator(Random random,
                                                                            int iteration,
                                                                            List<? extends JointBasics> joints,
                                                                            List<? extends JointBasics> jointsToIgnore,
                                                                            double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      rootBody.updateFramesRecursively();
      MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody, jointsToIgnore);

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(input);
      inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(input);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(input);

      compareAgainstExistingCompositeRigidBodyMassMatrixCalculator(random, iteration, joints, input, epsilon, forwardDynamicsCalculator, inverseDynamicsCalculator, massMatrixCalculator);
   }

   private static void compareAgainstExistingCompositeRigidBodyMassMatrixCalculator(Random random,
                                                                                    int iteration,
                                                                                    List<? extends JointBasics> joints,
                                                                                    MultiBodySystemBasics input,
                                                                                    double epsilon,
                                                                                    ForwardDynamicsCalculator forwardDynamicsCalculator,
                                                                                    InverseDynamicsCalculator inverseDynamicsCalculator,
                                                                                    CompositeRigidBodyMassMatrixCalculator massMatrixCalculator)
   {
      int numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(forwardDynamicsCalculator.getInput().getJointsToConsider());

      DMatrixRMaj qdd_expected = new DMatrixRMaj(numberOfDoFs, 1);
      int index = 0;

      for (JointBasics joint : input.getJointsToConsider())
      {
         joint.getJointAcceleration(index, qdd_expected);
         joint.setJointAccelerationToZero();
         index += joint.getDegreesOfFreedom();
      }

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

   private static void compareAgainstSpatialAccelerationCalculator(Random random,
                                                                   int iteration,
                                                                   List<? extends JointBasics> joints,
                                                                   Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                   List<? extends JointReadOnly> jointsToIgnore,
                                                                   double epsilon)
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
      SpatialAccelerationCalculator accelerationCalculator = new SpatialAccelerationCalculator(rootBody, multiBodySystemInput.getInertialFrame());
      SpatialAccelerationCalculator zeroVelocityAccelerationCalculator = new SpatialAccelerationCalculator(rootBody,
                                                                                                           multiBodySystemInput.getInertialFrame(),
                                                                                                           false);
      accelerationCalculator.setGravitionalAcceleration(gravity);
      zeroVelocityAccelerationCalculator.setGravitionalAcceleration(gravity);

      compareAgainstExistingSpatialAccelerationCalculator(random, iteration, joints, externalWrenches, epsilon, forwardDynamicsCalculator, accelerationCalculator, zeroVelocityAccelerationCalculator);
   }

   private static void compareAgainstExistingSpatialAccelerationCalculator(Random random,
                                 int iteration,
                                 List<? extends JointBasics> joints,
                                 Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                 double epsilon,
                                 ForwardDynamicsCalculator forwardDynamicsCalculator,
                                 SpatialAccelerationCalculator accelerationCalculator,
                                 SpatialAccelerationCalculator zeroVelocityAccelerationCalculator)
   {
      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);
      forwardDynamicsCalculator.compute();
      joints.forEach(forwardDynamicsCalculator::writeComputedJointAcceleration);
      RigidBodyAccelerationProvider fwdDynAccelerationProvider = forwardDynamicsCalculator.getAccelerationProvider();
      RigidBodyAccelerationProvider fwdDynZeroVelocityAccelerationProvider = forwardDynamicsCalculator.getAccelerationProvider(false);

      assertTrue(fwdDynAccelerationProvider.areAccelerationsConsidered());
      assertTrue(fwdDynAccelerationProvider.areVelocitiesConsidered());
      assertTrue(fwdDynZeroVelocityAccelerationProvider.areAccelerationsConsidered());
      assertFalse(fwdDynZeroVelocityAccelerationProvider.areVelocitiesConsidered());

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
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
            assertAccelerationEquals(expectedAccelerationOfBody, actualAccelerationOfBody, epsilon);

            SpatialAcceleration actualZeroVelocityAccelerationOfBody = new SpatialAcceleration(fwdDynZeroVelocityAccelerationProvider.getAccelerationOfBody(
                  rigidBody));
            assertAccelerationEquals(expectedZeroVelocityAccelerationOfBody, actualZeroVelocityAccelerationOfBody, epsilon);

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
                  assertAccelerationEquals(expectedRelativeAcceleration, actualRelativeAcceleration, epsilon);

                  SpatialAccelerationReadOnly actualRelativeZeroVelocityAcceleration = fwdDynZeroVelocityAccelerationProvider.getRelativeAcceleration(rigidBody,
                                                                                                                                                      otherRigidBody);
                  assertAccelerationEquals(expectedRelativeZeroVelocityAcceleration, actualRelativeZeroVelocityAcceleration, epsilon);
               }
            }
         }
      }
   }

   private static void assertAccelerationEquals(SpatialAccelerationReadOnly expectedAcceleration,
                                                SpatialAccelerationReadOnly actualAcceleration,
                                                double epsilon)
   {
      double adjustedEpsilon = epsilon * Math.max(1.0, expectedAcceleration == null ? 1.0 : expectedAcceleration.length());
      MecanoTestTools.assertSpatialAccelerationEquals(expectedAcceleration, actualAcceleration, adjustedEpsilon);
   }

   public static Map<RigidBodyReadOnly, WrenchReadOnly> nextExternalWrenches(Random random, List<? extends JointReadOnly> joints)
   {
      return joints.stream()
                   .filter(j -> random.nextBoolean())
                   .map(j -> j.getSuccessor())
                   .collect(Collectors.toMap(b -> b, b -> nextWrench(random, b.getBodyFixedFrame(), b.getBodyFixedFrame())));
   }
}
