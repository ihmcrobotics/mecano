package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.*;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.*;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;

public class MultiBodyResponseCalculatorTest
{
   private static final int ITERATIONS = 500;
   private static final double EPSILON = 1.0e-12;
   private static final double JOINT_EPSILON = 1.0e-9;

   @Test
   public void testPrismaticJointChain()
   {
      Random random = new Random(435346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, EPSILON);

         assertApplyMultipleWrenches(random, i, joints, EPSILON);
         assertApplyMultipleImpulses(random, i, joints, EPSILON);
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
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, EPSILON);

         assertApplyMultipleWrenches(random, i, joints, EPSILON);
         assertApplyMultipleImpulses(random, i, joints, EPSILON);
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
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, EPSILON);

         assertApplyMultipleWrenches(random, i, joints, EPSILON);
         assertApplyMultipleImpulses(random, i, joints, EPSILON);
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
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, EPSILON);

         assertApplyMultipleWrenches(random, i, joints, EPSILON);
         assertApplyMultipleImpulses(random, i, joints, EPSILON);
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
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, EPSILON);

         assertApplyMultipleWrenches(random, i, joints, EPSILON);
         assertApplyMultipleImpulses(random, i, joints, EPSILON);
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
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, EPSILON);

         assertApplyMultipleWrenches(random, i, joints, EPSILON);
         assertApplyMultipleImpulses(random, i, joints, EPSILON);
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
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, EPSILON);

         assertApplyMultipleWrenches(random, i, joints, EPSILON);
         assertApplyMultipleImpulses(random, i, joints, EPSILON);
      }
   }

   @Test
   public void testJointChain() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(20) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         assertApplySingleRigidBodyWrench(random, i, joints, JOINT_EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, JOINT_EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, JOINT_EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, JOINT_EPSILON);

         assertApplySingleJointWrench(random, i, joints, JOINT_EPSILON);
         assertApplySingleJointImpulse(random, i, joints, JOINT_EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, JOINT_EPSILON);

         assertApplyMultipleWrenches(random, i, joints, JOINT_EPSILON);
         assertApplyMultipleImpulses(random, i, joints, JOINT_EPSILON);
      }
   }

   private static void assertApplySingleRigidBodyWrench(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplySingleRigidBodyWrench(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleRigidBodyWrench(Random random, int iteration, List<? extends JointBasics> joints,
                                                        Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches, List<? extends JointReadOnly> jointsToIgnore,
                                                        double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      RigidBodyBasics target = joints.get(random.nextInt(joints.size())).getSuccessor();
      Wrench testWrench = MecanoRandomTools.nextWrench(random, target.getBodyFixedFrame(), target.getBodyFixedFrame());

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = setupForwardDynamicsCalculator(multiBodySystemInput,
                                                                                           gravity,
                                                                                           externalWrenches,
                                                                                           Collections.singletonMap(target, testWrench),
                                                                                           Collections.emptyMap());

      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);
      multiBodyResponseCalculator.applyRigidBodyWrench(target, testWrench);

      runAssertionsViaAccelerationProvider(random, iteration, joints, epsilon, forwardDynamicsCalculator, multiBodyResponseCalculator);

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.propagateWrench();
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, Math.max(1.0, CommonOps.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertApplySingleRigidBodyImpulse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplySingleRigidBodyImpulse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleRigidBodyImpulse(Random random, int iteration, List<? extends JointBasics> joints,
                                                         Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches, List<? extends JointReadOnly> jointsToIgnore,
                                                         double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      RigidBodyBasics target = joints.get(random.nextInt(joints.size())).getSuccessor();
      SpatialImpulse testImpulse = MecanoRandomTools.nextSpatialImpulse(random, target.getBodyFixedFrame(), target.getBodyFixedFrame());

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = setupForwardDynamicsCalculator(multiBodySystemInput,
                                                                                           gravity,
                                                                                           externalWrenches,
                                                                                           Collections.singletonMap(target, testImpulse),
                                                                                           Collections.emptyMap());

      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);
      multiBodyResponseCalculator.applyRigidBodyImpulse(target, testImpulse);

      runAssertionViaTwistProvider(random, iteration, joints, epsilon, forwardDynamicsCalculator, multiBodyResponseCalculator);

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.propagateImpulse();
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, Math.max(1.0, CommonOps.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertRigidBodyApparentInertiaInverse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertRigidBodyApparentInertiaInverse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertRigidBodyApparentInertiaInverse(Random random, int iteration, List<? extends JointBasics> joints,
                                                             Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                             List<? extends JointReadOnly> jointsToIgnore, double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      RigidBodyBasics target = joints.get(random.nextInt(joints.size())).getSuccessor();
      ReferenceFrame testWrenchFrame = EuclidFrameRandomTools.nextReferenceFrame(random, target.getBodyFixedFrame());
      Wrench testWrench = MecanoRandomTools.nextWrench(random, target.getBodyFixedFrame(), testWrenchFrame);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);

      DenseMatrix64F apparentSpatialInertiaInverse = new DenseMatrix64F(6, 6);
      multiBodyResponseCalculator.computeRigidBodyApparentSpatialInertiaInverse(target, testWrenchFrame, apparentSpatialInertiaInverse);
      DenseMatrix64F accelerationChangeMatrix = new DenseMatrix64F(6, 1);
      DenseMatrix64F testWrenchMatrix = new DenseMatrix64F(6, 1);
      testWrench.get(testWrenchMatrix);
      CommonOps.mult(apparentSpatialInertiaInverse, testWrenchMatrix, accelerationChangeMatrix);
      SpatialAcceleration actualAccelerationChange = new SpatialAcceleration(target.getBodyFixedFrame(),
                                                                             ReferenceFrame.getWorldFrame(),
                                                                             testWrenchFrame,
                                                                             accelerationChangeMatrix);
      actualAccelerationChange.changeFrame(target.getBodyFixedFrame());

      multiBodyResponseCalculator.applyRigidBodyWrench(target, testWrench);
      SpatialAcceleration expectedAccelerationChange = new SpatialAcceleration(multiBodyResponseCalculator.getAccelerationChangeProvider()
                                                                                                          .getAccelerationOfBody(target));

      MecanoTestTools.assertSpatialAccelerationEquals(expectedAccelerationChange,
                                                      actualAccelerationChange,
                                                      Math.max(1.0, expectedAccelerationChange.length()) * epsilon);
   }

   private static void assertRigidBodyApparentLinearInertiaInverse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertRigidBodyApparentLinearInertiaInverse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertRigidBodyApparentLinearInertiaInverse(Random random, int iteration, List<? extends JointBasics> joints,
                                                                   Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                   List<? extends JointReadOnly> jointsToIgnore, double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      RigidBodyBasics target = joints.get(random.nextInt(joints.size())).getSuccessor();
      ReferenceFrame testWrenchFrame = EuclidFrameRandomTools.nextReferenceFrame(random, target.getBodyFixedFrame());
      Wrench testWrench = MecanoRandomTools.nextWrench(random, target.getBodyFixedFrame(), testWrenchFrame);
      testWrench.getAngularPart().setToZero();

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);

      DenseMatrix64F apparentSpatialInertiaInverse = new DenseMatrix64F(3, 3);
      multiBodyResponseCalculator.computeRigidBodyApparentLinearInertiaInverse(target, testWrenchFrame, apparentSpatialInertiaInverse);
      DenseMatrix64F accelerationChangeMatrix = new DenseMatrix64F(3, 1);
      DenseMatrix64F testWrenchMatrix = new DenseMatrix64F(3, 1);
      testWrench.getLinearPart().get(testWrenchMatrix);
      CommonOps.mult(apparentSpatialInertiaInverse, testWrenchMatrix, accelerationChangeMatrix);
      FrameVector3D actualLinearAccelerationChange = new FrameVector3D(testWrenchFrame);
      actualLinearAccelerationChange.set(accelerationChangeMatrix);

      multiBodyResponseCalculator.applyRigidBodyWrench(target, testWrench);
      SpatialAcceleration expectedAccelerationChange = new SpatialAcceleration(multiBodyResponseCalculator.getAccelerationChangeProvider()
                                                                                                          .getAccelerationOfBody(target));
      expectedAccelerationChange.changeFrame(testWrenchFrame);
      EuclidFrameTestTools.assertFrameTuple3DEquals(expectedAccelerationChange.getLinearPart(),
                                                    actualLinearAccelerationChange,
                                                    Math.max(1.0, expectedAccelerationChange.getLinearPart().length()) * epsilon);
   }

   private static void assertApplySingleJointWrench(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplySingleJointWrench(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleJointWrench(Random random, int iteration, List<? extends JointBasics> joints,
                                                    Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches, List<? extends JointReadOnly> jointsToIgnore,
                                                    double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      JointBasics target = joints.get(random.nextInt(joints.size()));
      DenseMatrix64F testJointWrench = RandomMatrices.createRandom(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = setupForwardDynamicsCalculator(multiBodySystemInput,
                                                                                           gravity,
                                                                                           externalWrenches,
                                                                                           Collections.emptyMap(),
                                                                                           Collections.singletonMap(target, testJointWrench));
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);
      if (target instanceof OneDoFJointBasics && random.nextBoolean())
         multiBodyResponseCalculator.applyJointWrench((OneDoFJointReadOnly) target, testJointWrench.get(0));
      else
         multiBodyResponseCalculator.applyJointWrench(target, testJointWrench);

      runAssertionsViaAccelerationProvider(random, iteration, joints, epsilon, forwardDynamicsCalculator, multiBodyResponseCalculator);

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.propagateWrench();
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, Math.max(1.0, CommonOps.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertApplySingleJointImpulse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplySingleJointImpulse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleJointImpulse(Random random, int iteration, List<? extends JointBasics> joints,
                                                     Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches, List<? extends JointReadOnly> jointsToIgnore,
                                                     double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      JointBasics target = joints.get(random.nextInt(joints.size()));
      DenseMatrix64F testJointImpulse = RandomMatrices.createRandom(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = setupForwardDynamicsCalculator(multiBodySystemInput,
                                                                                           gravity,
                                                                                           externalWrenches,
                                                                                           Collections.emptyMap(),
                                                                                           Collections.singletonMap(target, testJointImpulse));
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);
      if (target instanceof OneDoFJointBasics && random.nextBoolean())
         multiBodyResponseCalculator.applyJointImpulse((OneDoFJointReadOnly) target, testJointImpulse.get(0));
      else
         multiBodyResponseCalculator.applyJointImpulse(target, testJointImpulse);

      runAssertionViaTwistProvider(random, iteration, joints, epsilon, forwardDynamicsCalculator, multiBodyResponseCalculator);

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.propagateImpulse();
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, Math.max(1.0, CommonOps.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertJointApparentInertiaInverse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertJointApparentInertiaInverse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertJointApparentInertiaInverse(Random random, int iteration, List<? extends JointBasics> joints,
                                                         Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches, List<? extends JointReadOnly> jointsToIgnore,
                                                         double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      JointBasics target = joints.get(random.nextInt(joints.size()));
      DenseMatrix64F testWrench = RandomMatrices.createRandom(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);

      DenseMatrix64F apparentInertiaInverse = new DenseMatrix64F(target.getDegreesOfFreedom(), target.getDegreesOfFreedom());
      multiBodyResponseCalculator.computeJointApparentInertiaInverse(target, apparentInertiaInverse);
      DenseMatrix64F actualMotionChange = new DenseMatrix64F(target.getDegreesOfFreedom(), 1);
      CommonOps.mult(apparentInertiaInverse, testWrench, actualMotionChange);

      if (target instanceof OneDoFJointBasics)
      {
         double oneDoFJointInertiaInverse = multiBodyResponseCalculator.computeJointApparentInertiaInverse((OneDoFJointReadOnly) target);
         double expectedOneDoFJointMotionChange = oneDoFJointInertiaInverse * testWrench.get(0);
         multiBodyResponseCalculator.applyJointWrench(target, testWrench);
         assertEquals(expectedOneDoFJointMotionChange, multiBodyResponseCalculator.getJointAccelerationChange((OneDoFJointReadOnly) target), epsilon);
      }
      else
      {
         multiBodyResponseCalculator.applyJointWrench(target, testWrench);
      }

      DenseMatrix64F expectedMotionChange = multiBodyResponseCalculator.getJointAccelerationChange(target);
      assertMatrixEquals(iteration, expectedMotionChange, actualMotionChange, epsilon);
   }

   private static void assertApplyMultipleWrenches(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplyMultipleWrenches(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplyMultipleWrenches(Random random, int iteration, List<? extends JointBasics> joints,
                                                   Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches, List<? extends JointReadOnly> jointsToIgnore,
                                                   double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      int numberOfRigidBodyDisturbances = random.nextInt(joints.size()) + 1;
      Map<RigidBodyBasics, Wrench> rigidBodyDisturbances = new HashMap<>();

      List<RigidBodyBasics> rigidBodies = joints.stream().map(JointBasics::getSuccessor).collect(Collectors.toList());

      for (int i = 0; i < numberOfRigidBodyDisturbances; i++)
      {
         RigidBodyBasics target = rigidBodies.get(random.nextInt(rigidBodies.size()));
         Wrench testWrench = MecanoRandomTools.nextWrench(random, target.getBodyFixedFrame(), target.getBodyFixedFrame());
         rigidBodyDisturbances.put(target, testWrench);
      }

      int numberOfJointDisturbances = random.nextInt(joints.size()) + 1;
      Map<JointBasics, DenseMatrix64F> jointDisturbances = new HashMap<>();

      for (int i = 0; i < numberOfJointDisturbances; i++)
      {
         JointBasics target = joints.get(random.nextInt(joints.size()));
         DenseMatrix64F testWrench = RandomMatrices.createRandom(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);
         jointDisturbances.put(target, testWrench);
      }

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = setupForwardDynamicsCalculator(multiBodySystemInput,
                                                                                           gravity,
                                                                                           externalWrenches,
                                                                                           rigidBodyDisturbances,
                                                                                           jointDisturbances);

      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);
      rigidBodyDisturbances.forEach(multiBodyResponseCalculator::applyRigidBodyWrench);
      jointDisturbances.forEach(multiBodyResponseCalculator::applyJointWrench);

      runAssertionsViaAccelerationProvider(random, iteration, joints, epsilon, forwardDynamicsCalculator, multiBodyResponseCalculator);

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.propagateWrench();
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, Math.max(1.0, CommonOps.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertApplyMultipleImpulses(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplyMultipleImpulse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplyMultipleImpulse(Random random, int iteration, List<? extends JointBasics> joints,
                                                  Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches, List<? extends JointReadOnly> jointsToIgnore,
                                                  double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      int numberOfRigidBodyDisturbances = random.nextInt(joints.size()) + 1;
      Map<RigidBodyBasics, SpatialImpulse> rigidBodyDisturbances = new HashMap<>();

      List<RigidBodyBasics> rigidBodies = joints.stream().map(JointBasics::getSuccessor).collect(Collectors.toList());

      for (int i = 0; i < numberOfRigidBodyDisturbances; i++)
      {
         RigidBodyBasics target = rigidBodies.get(random.nextInt(rigidBodies.size()));
         SpatialImpulse testImpulse = MecanoRandomTools.nextSpatialImpulse(random, target.getBodyFixedFrame(), target.getBodyFixedFrame());
         rigidBodyDisturbances.put(target, testImpulse);
      }

      int numberOfJointDisturbances = random.nextInt(joints.size()) + 1;
      Map<JointBasics, DenseMatrix64F> jointDisturbances = new HashMap<>();

      for (int i = 0; i < numberOfJointDisturbances; i++)
      {
         JointBasics target = joints.get(random.nextInt(joints.size()));
         DenseMatrix64F testWrench = RandomMatrices.createRandom(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);
         jointDisturbances.put(target, testWrench);
      }

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = setupForwardDynamicsCalculator(multiBodySystemInput,
                                                                                           gravity,
                                                                                           externalWrenches,
                                                                                           rigidBodyDisturbances,
                                                                                           jointDisturbances);

      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);
      rigidBodyDisturbances.forEach(multiBodyResponseCalculator::applyRigidBodyImpulse);
      jointDisturbances.forEach(multiBodyResponseCalculator::applyJointImpulse);

      runAssertionViaTwistProvider(random, iteration, joints, epsilon, forwardDynamicsCalculator, multiBodyResponseCalculator);

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.propagateImpulse();
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, Math.max(1.0, CommonOps.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void runAssertionViaTwistProvider(Random random, int iteration, List<? extends JointBasics> joints, double epsilon,
                                                    ForwardDynamicsCalculator forwardDynamicsCalculator,
                                                    MultiBodyResponseCalculator multiBodyResponseCalculator)
   {
      for (int i = 0; i < 10; i++)
      {
         RigidBodyBasics body = joints.get(random.nextInt(joints.size())).getSuccessor();

         SpatialAcceleration expectedTargetAcceleration = new SpatialAcceleration(forwardDynamicsCalculator.getAccelerationProvider()
                                                                                                           .getAccelerationOfBody(body));

         RigidBodyTwistProvider twistChangeProvider = multiBodyResponseCalculator.getTwistChangeProvider();
         RigidBodyAccelerationProvider originalAccelerationProvider = multiBodyResponseCalculator.getForwardDynamicsCalculator().getAccelerationProvider();
         SpatialAcceleration actualTargetAcceleration = new SpatialAcceleration(originalAccelerationProvider.getAccelerationOfBody(body));
         actualTargetAcceleration.add((SpatialVectorReadOnly) twistChangeProvider.getTwistOfBody(body)); // <= Same equations as for the acceleration.
         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + iteration + ", body: " + i,
                                                         expectedTargetAcceleration,
                                                         actualTargetAcceleration,
                                                         Math.max(1.0, expectedTargetAcceleration.length()) * epsilon);
      }
   }

   private static void runAssertionsViaAccelerationProvider(Random random, int iteration, List<? extends JointBasics> joints, double epsilon,
                                                            ForwardDynamicsCalculator forwardDynamicsCalculator,
                                                            MultiBodyResponseCalculator multiBodyResponseCalculator)
   {
      for (int i = 0; i < 10; i++)
      {
         RigidBodyBasics body = joints.get(random.nextInt(joints.size())).getSuccessor();

         SpatialAcceleration expectedTargetAcceleration = new SpatialAcceleration(forwardDynamicsCalculator.getAccelerationProvider()
                                                                                                           .getAccelerationOfBody(body));

         RigidBodyAccelerationProvider accelerationChangeProvider = multiBodyResponseCalculator.getAccelerationChangeProvider();
         RigidBodyAccelerationProvider originalAccelerationProvider = multiBodyResponseCalculator.getForwardDynamicsCalculator().getAccelerationProvider();
         SpatialAcceleration actualTargetAcceleration = new SpatialAcceleration(originalAccelerationProvider.getAccelerationOfBody(body));
         actualTargetAcceleration.add((SpatialVectorReadOnly) accelerationChangeProvider.getAccelerationOfBody(body));
         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + iteration + ", body: " + i,
                                                         expectedTargetAcceleration,
                                                         actualTargetAcceleration,
                                                         Math.max(1.0, expectedTargetAcceleration.length()) * epsilon);
      }
   }

   private static void assertMatrixEquals(int iteration, DenseMatrix64F expected, DenseMatrix64F actual, double epsilon)
   {
      boolean areEqual = MatrixFeatures.isEquals(expected, actual, epsilon);
      if (!areEqual)
      {
         System.out.println("iteration: " + iteration);
         double maxError = 0.0;
         DenseMatrix64F output = new DenseMatrix64F(expected.getNumRows(), 3);

         for (int row = 0; row < expected.getNumRows(); row++)
         {
            double error = expected.get(row, 0) - actual.get(row, 0);

            output.set(row, 0, expected.get(row, 0));
            output.set(row, 1, actual.get(row, 0));
            output.set(row, 2, error);
            maxError = Math.max(maxError, Math.abs(error));
         }
         output.print(EuclidCoreIOTools.getStringFormat(9, 6));
         System.out.println("Max error: " + maxError);
      }
      assertTrue(areEqual);
   }

   private static ForwardDynamicsCalculator setupForwardDynamicsCalculator(MultiBodySystemReadOnly input, double gravity,
                                                                           Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                           Map<? extends RigidBodyReadOnly, ? extends SpatialForceReadOnly> testRigidBodyDisturbances,
                                                                           Map<? extends JointReadOnly, DenseMatrix64F> testJointDisturbances)
   {
      ForwardDynamicsCalculator calculator = new ForwardDynamicsCalculator(input);
      calculator.setGravitionalAcceleration(gravity);
      calculator.setExternalWrenchesToZero();
      externalWrenches.forEach(calculator::setExternalWrench);
      testRigidBodyDisturbances.forEach((body, disturbance) -> calculator.getExternalWrench(body).add(disturbance));

      List<? extends JointReadOnly> joints = input.getJointsToConsider();
      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();
      DenseMatrix64F tauWithDisturbances = new DenseMatrix64F(numberOfDoFs, 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tauWithDisturbances);
      JointMatrixIndexProvider jointMatrixIndexProvider = input.getJointMatrixIndexProvider();
      for (Entry<? extends JointReadOnly, DenseMatrix64F> entry : testJointDisturbances.entrySet())
      {
         JointReadOnly joint = entry.getKey();
         DenseMatrix64F jointDisturbance = entry.getValue();

         for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
            tauWithDisturbances.add(jointMatrixIndexProvider.getJointDoFIndices(joint)[i], 0, jointDisturbance.get(i));
      }
      calculator.compute(tauWithDisturbances);

      return calculator;
   }

   private static MultiBodyResponseCalculator setupMultiBodyResponseCalculator(MultiBodySystemReadOnly input, double gravity,
                                                                               Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches)
   {
      MultiBodyResponseCalculator multiBodyResponseCalculator = new MultiBodyResponseCalculator(input);
      ForwardDynamicsCalculator forwardDynamicsCalculator = multiBodyResponseCalculator.getForwardDynamicsCalculator();
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);

      forwardDynamicsCalculator.setExternalWrenchesToZero();
      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);
      forwardDynamicsCalculator.compute();
      return multiBodyResponseCalculator;
   }

   private static void assertJointAccelerationMatrixEquals(int iteration, DenseMatrix64F qdd_expected, DenseMatrix64F qdd_original, DenseMatrix64F qdd_change,
                                                           double epsilon)
   {
      DenseMatrix64F qdd_actual = new DenseMatrix64F(qdd_change.getNumRows(), 1);
      CommonOps.add(qdd_original, qdd_change, qdd_actual);

      boolean areEqual = MatrixFeatures.isEquals(qdd_expected, qdd_actual, epsilon);
      if (!areEqual)
      {
         System.out.println("iteration: " + iteration);
         double maxError = 0.0;
         DenseMatrix64F output = new DenseMatrix64F(qdd_expected.getNumRows(), 5);

         for (int row = 0; row < qdd_expected.getNumRows(); row++)
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
