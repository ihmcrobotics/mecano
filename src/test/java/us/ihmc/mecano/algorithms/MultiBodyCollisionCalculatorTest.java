package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;

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
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class MultiBodyCollisionCalculatorTest
{
   private static final int ITERATIONS = 500;
   private static final double EPSILON = 1.0e-12;
   private static final double JOINT_EPSILON = 1.0e-8;

   @Test
   public void testPrismaticJointChain()
   {
      Random random = new Random(435346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
         compareAgainstForwardDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), EPSILON);
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
         compareAgainstForwardDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), EPSILON);
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
         compareAgainstForwardDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), EPSILON);
      }
   }

   @Test
   public void testRevoluteJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, random.nextInt(50) + 1);
         compareAgainstForwardDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), EPSILON);
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
         compareAgainstForwardDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), EPSILON);
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
         compareAgainstForwardDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), EPSILON);
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
         compareAgainstForwardDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), EPSILON);
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
         compareAgainstForwardDynamicsCalculator(random, i, joints, Collections.emptyMap(), Collections.emptyList(), JOINT_EPSILON);
      }
   }

   private static void compareAgainstForwardDynamicsCalculator(Random random, int iteration, List<? extends JointBasics> joints,
                                                               Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                               List<? extends JointReadOnly> jointsToIgnore, double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      RigidBodyBasics target = joints.get(random.nextInt(joints.size())).getSuccessor();
      Wrench testWrench = MecanoRandomTools.nextWrench(random, target.getBodyFixedFrame(), target.getBodyFixedFrame());

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
      MultiBodyCollisionCalculator multiBodyCollisionCalculator = new MultiBodyCollisionCalculator(multiBodySystemInput);
      multiBodyCollisionCalculator.getForwardDynamicsCalculator().setGravitionalAcceleration(gravity);

      forwardDynamicsCalculator.setExternalWrenchesToZero();
      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);
      forwardDynamicsCalculator.getExternalWrench(target).add(testWrench);
      forwardDynamicsCalculator.compute();
      SpatialAcceleration expectedTargetAcceleration = new SpatialAcceleration(forwardDynamicsCalculator.getAccelerationProvider()
                                                                                                        .getAccelerationOfBody(target));

      multiBodyCollisionCalculator.getForwardDynamicsCalculator().setExternalWrenchesToZero();
      externalWrenches.forEach(multiBodyCollisionCalculator.getForwardDynamicsCalculator()::setExternalWrench);
      multiBodyCollisionCalculator.getForwardDynamicsCalculator().compute();
      DenseMatrix64F qdd_change = new DenseMatrix64F(numberOfDoFs, 1);
      multiBodyCollisionCalculator.applyWrench(target, testWrench, qdd_change);
      SpatialAcceleration accelerationChange = new SpatialAcceleration(multiBodyCollisionCalculator.getAccelerationChangeProvider()
                                                                                                   .getAccelerationOfBody(target));
      accelerationChange.changeFrame(target.getBodyFixedFrame());
      SpatialAccelerationReadOnly originalTargetAcceleration = multiBodyCollisionCalculator.getForwardDynamicsCalculator().getAccelerationProvider()
                                                                                           .getAccelerationOfBody(target);
      SpatialAcceleration actualTargetAcceleration = new SpatialAcceleration(originalTargetAcceleration);
      actualTargetAcceleration.add((SpatialVectorReadOnly) accelerationChange);
      MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + iteration, expectedTargetAcceleration, actualTargetAcceleration, epsilon);

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_actual = new DenseMatrix64F(numberOfDoFs, 1);
      DenseMatrix64F qdd_original = multiBodyCollisionCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      CommonOps.add(qdd_original, qdd_change, qdd_actual);

      boolean areEqual = MatrixFeatures.isEquals(qdd_expected, qdd_actual, epsilon);
      if (!areEqual)
      {
         System.out.println("iteration: " + iteration);
         double maxError = 0.0;
         DenseMatrix64F output = new DenseMatrix64F(numberOfDoFs, 5);

         for (int row = 0; row < numberOfDoFs; row++)
         {
            double error = qdd_expected.get(row, 0) - qdd_actual.get(row, 0);

            output.set(row, 0, qdd_expected.get(row, 0));
            output.set(row, 1, qdd_original.get(row, 0));
            output.set(row, 2, qdd_change.get(row, 0));
            output.set(row, 3, qdd_actual.get(row, 0));
            output.set(row, 4, error);
            maxError = Math.max(maxError, Math.abs(error));
         }
         output.print(EuclidCoreIOTools.getStringFormat(9, 6));
         System.out.println("Max error: " + maxError);
      }
      assertTrue(areEqual);
   }
}
