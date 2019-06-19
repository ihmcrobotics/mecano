package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.mecano.tools.MecanoRandomTools.*;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class ForwardDynamicsCalculatorTest
{
   private static final int ITERATIONS = 500;
   private static final double ONE_DOF_JOINT_EPSILON = 8.0e-12;
   private static final double FLOATING_JOINT_EPSILON = 2.0e-11;
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

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
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

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testRevoluteJointChain() throws Exception
   {
      Random random = new Random(2654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 3; //random.nextInt(3) + 1;
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, 0, joints, ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
      }
   }

   @Test
   public void testRevoluteJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, random.nextInt(50) + 1);
         compareAgainstInverseDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);
         compareAgainstCompositeRigidBodyMassMatrixCalculator(random, i, joints, ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, random.nextInt(40) + 1);
         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
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

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
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

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ONE_DOF_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ONE_DOF_JOINT_EPSILON);
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

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), FLOATING_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 2.0 * FLOATING_JOINT_EPSILON);
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

         compareAgainstInverseDynamicsCalculator(random, i, joints, nextExternalWrenches(random, joints), Collections.emptyList(), ALL_JOINT_EPSILON);

         compareAgainstInverseDynamicsCalculator(random,
                                                 i,
                                                 joints,
                                                 Collections.emptyMap(),
                                                 Collections.singletonList(joints.get(random.nextInt(joints.size()))),
                                                 ALL_JOINT_EPSILON);
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

      int numberOfTicks = 5;

      for (int i = 0; i < numberOfTicks; i++)
      {
         DenseMatrix64F qdd_expected = new DenseMatrix64F(numberOfDoFs, 1);
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

         DenseMatrix64F qdd_actual = new DenseMatrix64F(numberOfDoFs, 1);
         index = 0;
         for (JointBasics joint : joints)
         {
            joint.getJointAcceleration(index, qdd_actual);
            index += joint.getDegreesOfFreedom();
         }

         boolean areEqual = MatrixFeatures.isEquals(qdd_expected, qdd_actual, epsilon);
         if (!areEqual)
         {
            System.out.println("iteration: " + iteration);
            double maxError = 0.0;
            DenseMatrix64F output = new DenseMatrix64F(numberOfDoFs, 3);
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
      }
   }

   private static void compareAgainstCompositeRigidBodyMassMatrixCalculator(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      rootBody.updateFramesRecursively();

      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      DenseMatrix64F qdd_expected = new DenseMatrix64F(numberOfDoFs, 1);
      int index = 0;
      for (JointBasics joint : joints)
      {
         joint.getJointAcceleration(index, qdd_expected);
         joint.setJointAccelerationToZero();
         index += joint.getDegreesOfFreedom();
      }

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(rootBody);
      inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootBody);

      inverseDynamicsCalculator.compute();
      massMatrixCalculator.reset();

      inverseDynamicsCalculator.writeComputedJointWrenches(joints);

      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();
      DenseMatrix64F biasMatrix = new DenseMatrix64F(numberOfDoFs, 1);

      index = 0;
      for (JointReadOnly joint : massMatrixCalculator.getInput().getJointsToConsider())
      {
         joint.getJointTau(index, biasMatrix);
         index += joint.getDegreesOfFreedom();
      }

      DenseMatrix64F tauMatrix = new DenseMatrix64F(numberOfDoFs, 1);

      CommonOps.mult(massMatrix, qdd_expected, tauMatrix);
      CommonOps.addEquals(tauMatrix, biasMatrix);

      index = 0;
      for (JointBasics joint : joints)
      {
         joint.setJointTau(index, tauMatrix);
         index += joint.getDegreesOfFreedom();
      }

      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(rootBody);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);

      forwardDynamicsCalculator.compute();

      DenseMatrix64F qdd_actual = forwardDynamicsCalculator.getJointAccelerationMatrix();

      boolean areEqual = MatrixFeatures.isEquals(qdd_expected, qdd_actual, epsilon);
      if (!areEqual)
      {
         System.out.println("iteration: " + iteration);
         double maxError = 0.0;
         DenseMatrix64F output = new DenseMatrix64F(numberOfDoFs, 3);
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
   }

   private static Map<RigidBodyReadOnly, WrenchReadOnly> nextExternalWrenches(Random random, List<? extends JointReadOnly> joints)
   {
      return joints.stream().filter(j -> random.nextBoolean()).map(j -> j.getSuccessor())
                   .collect(Collectors.toMap(b -> b, b -> nextWrench(random, b.getBodyFixedFrame(), b.getBodyFixedFrame())));
   }
}
