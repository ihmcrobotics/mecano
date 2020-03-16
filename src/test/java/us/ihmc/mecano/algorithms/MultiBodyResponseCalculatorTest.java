package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
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
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.*;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;

public class MultiBodyResponseCalculatorTest
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
         assertApplyAndPropagateRigidBodyWrench(random, i, joints, EPSILON);
         assertApplyAndPropagateRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplyAndPropagateJointEffort(random, i, joints, EPSILON);
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
         assertApplyAndPropagateRigidBodyWrench(random, i, joints, EPSILON);
         assertApplyAndPropagateRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplyAndPropagateJointEffort(random, i, joints, EPSILON);
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
         assertApplyAndPropagateRigidBodyWrench(random, i, joints, EPSILON);
         assertApplyAndPropagateRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplyAndPropagateJointEffort(random, i, joints, EPSILON);
      }
   }

   @Test
   public void testRevoluteJointTree() throws Exception
   {
      Random random = new Random(21654);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, random.nextInt(50) + 1);
         assertApplyAndPropagateRigidBodyWrench(random, i, joints, EPSILON);
         assertApplyAndPropagateRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplyAndPropagateJointEffort(random, i, joints, EPSILON);
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
         assertApplyAndPropagateRigidBodyWrench(random, i, joints, EPSILON);
         assertApplyAndPropagateRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplyAndPropagateJointEffort(random, i, joints, EPSILON);
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
         assertApplyAndPropagateRigidBodyWrench(random, i, joints, EPSILON);
         assertApplyAndPropagateRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplyAndPropagateJointEffort(random, i, joints, EPSILON);
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
         assertApplyAndPropagateRigidBodyWrench(random, i, joints, EPSILON);
         assertApplyAndPropagateRigidBodyImpulse(random, i, joints, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplyAndPropagateJointEffort(random, i, joints, EPSILON);
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
         assertApplyAndPropagateRigidBodyWrench(random, i, joints, JOINT_EPSILON);
         assertApplyAndPropagateRigidBodyImpulse(random, i, joints, JOINT_EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, JOINT_EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, JOINT_EPSILON);

         assertApplyAndPropagateJointEffort(random, i, joints, EPSILON);
      }
   }

   private static void assertApplyAndPropagateRigidBodyWrench(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplyAndPropagateRigidBodyWrench(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplyAndPropagateRigidBodyWrench(Random random, int iteration, List<? extends JointBasics> joints,
                                                              Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                              List<? extends JointReadOnly> jointsToIgnore, double epsilon)
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
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
      MultiBodyResponseCalculator multiBodyResponseCalculator = new MultiBodyResponseCalculator(multiBodySystemInput);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().setGravitionalAcceleration(gravity);

      forwardDynamicsCalculator.setExternalWrenchesToZero();
      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);
      forwardDynamicsCalculator.getExternalWrench(target).add(testWrench);
      forwardDynamicsCalculator.compute();

      multiBodyResponseCalculator.getForwardDynamicsCalculator().setExternalWrenchesToZero();
      externalWrenches.forEach(multiBodyResponseCalculator.getForwardDynamicsCalculator()::setExternalWrench);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().compute();
      multiBodyResponseCalculator.applyWrench(target, testWrench);

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
                                                         epsilon);
      }

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.applyAndPropagateWrench(target, testWrench);
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, epsilon);
   }

   private static void assertApplyAndPropagateRigidBodyImpulse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplyAndPropagateRigidBodyImpulse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplyAndPropagateRigidBodyImpulse(Random random, int iteration, List<? extends JointBasics> joints,
                                                               Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                               List<? extends JointReadOnly> jointsToIgnore, double epsilon)
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
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
      MultiBodyResponseCalculator multiBodyResponseCalculator = new MultiBodyResponseCalculator(multiBodySystemInput);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().setGravitionalAcceleration(gravity);

      forwardDynamicsCalculator.setExternalWrenchesToZero();
      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);
      forwardDynamicsCalculator.getExternalWrench(target).add(testImpulse);
      forwardDynamicsCalculator.compute();

      multiBodyResponseCalculator.getForwardDynamicsCalculator().setExternalWrenchesToZero();
      externalWrenches.forEach(multiBodyResponseCalculator.getForwardDynamicsCalculator()::setExternalWrench);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().compute();
      multiBodyResponseCalculator.applyImpulse(target, testImpulse);

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
                                                         epsilon);
      }

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.applyAndPropagateImpulse(target, testImpulse); // <= Same equations as for the accelerations.
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, epsilon);
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
      MultiBodyResponseCalculator multiBodyResponseCalculator = new MultiBodyResponseCalculator(multiBodySystemInput);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().setGravitionalAcceleration(gravity);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().setExternalWrenchesToZero();
      externalWrenches.forEach(multiBodyResponseCalculator.getForwardDynamicsCalculator()::setExternalWrench);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().compute();

      DenseMatrix64F apparentSpatialInertiaInverse = new DenseMatrix64F(6, 6);
      multiBodyResponseCalculator.computeApparentSpatialInertiaInverse(target, testWrenchFrame, apparentSpatialInertiaInverse);
      DenseMatrix64F accelerationChangeMatrix = new DenseMatrix64F(6, 1);
      DenseMatrix64F testWrenchMatrix = new DenseMatrix64F(6, 1);
      testWrench.get(testWrenchMatrix);
      CommonOps.mult(apparentSpatialInertiaInverse, testWrenchMatrix, accelerationChangeMatrix);
      SpatialAcceleration actualAccelerationChange = new SpatialAcceleration(target.getBodyFixedFrame(),
                                                                             ReferenceFrame.getWorldFrame(),
                                                                             testWrenchFrame,
                                                                             accelerationChangeMatrix);
      actualAccelerationChange.changeFrame(target.getBodyFixedFrame());

      multiBodyResponseCalculator.applyWrench(target, testWrench);
      SpatialAcceleration expectedAccelerationChange = new SpatialAcceleration(multiBodyResponseCalculator.getAccelerationChangeProvider()
                                                                                                          .getAccelerationOfBody(target));

      MecanoTestTools.assertSpatialAccelerationEquals(expectedAccelerationChange, actualAccelerationChange, epsilon);
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
      MultiBodyResponseCalculator multiBodyResponseCalculator = new MultiBodyResponseCalculator(multiBodySystemInput);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().setGravitionalAcceleration(gravity);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().setExternalWrenchesToZero();
      externalWrenches.forEach(multiBodyResponseCalculator.getForwardDynamicsCalculator()::setExternalWrench);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().compute();

      DenseMatrix64F apparentSpatialInertiaInverse = new DenseMatrix64F(3, 3);
      multiBodyResponseCalculator.computeApparentLinearInertiaInverse(target, testWrenchFrame, apparentSpatialInertiaInverse);
      DenseMatrix64F accelerationChangeMatrix = new DenseMatrix64F(3, 1);
      DenseMatrix64F testWrenchMatrix = new DenseMatrix64F(3, 1);
      testWrench.getLinearPart().get(testWrenchMatrix);
      CommonOps.mult(apparentSpatialInertiaInverse, testWrenchMatrix, accelerationChangeMatrix);
      FrameVector3D actualLinearAccelerationChange = new FrameVector3D(testWrenchFrame);
      actualLinearAccelerationChange.set(accelerationChangeMatrix);

      multiBodyResponseCalculator.applyWrench(target, testWrench);
      SpatialAcceleration expectedAccelerationChange = new SpatialAcceleration(multiBodyResponseCalculator.getAccelerationChangeProvider()
                                                                                                          .getAccelerationOfBody(target));
      expectedAccelerationChange.changeFrame(testWrenchFrame);
      EuclidFrameTestTools.assertFrameTuple3DEquals(expectedAccelerationChange.getLinearPart(), actualLinearAccelerationChange, epsilon);
   }

   private static void assertApplyAndPropagateJointEffort(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplyAndPropagateJointEffort(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplyAndPropagateJointEffort(Random random, int iteration, List<? extends JointBasics> joints,
                                                          Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches, List<? extends JointReadOnly> jointsToIgnore,
                                                          double epsilon)
   {
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.EFFORT, joints);

      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      List<OneDoFJointBasics> oneDoFJoints = joints.stream().filter(OneDoFJointBasics.class::isInstance).map(OneDoFJointBasics.class::cast)
                                                   .collect(Collectors.toList());
      OneDoFJointBasics target = oneDoFJoints.get(random.nextInt(oneDoFJoints.size()));
      double testJointEffort = EuclidCoreRandomTools.nextDouble(random, 100.0);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(multiBodySystemInput);
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
      MultiBodyResponseCalculator multiBodyResponseCalculator = new MultiBodyResponseCalculator(multiBodySystemInput);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().setGravitionalAcceleration(gravity);

      DenseMatrix64F tau_FwdDyn = new DenseMatrix64F(numberOfDoFs, 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tau_FwdDyn);
      tau_FwdDyn.add(forwardDynamicsCalculator.getInput().getJointMatrixIndexProvider().getJointConfigurationIndices(target)[0], 0, testJointEffort);
      forwardDynamicsCalculator.setExternalWrenchesToZero();
      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);
      forwardDynamicsCalculator.compute(tau_FwdDyn);

      multiBodyResponseCalculator.getForwardDynamicsCalculator().setExternalWrenchesToZero();
      externalWrenches.forEach(multiBodyResponseCalculator.getForwardDynamicsCalculator()::setExternalWrench);
      multiBodyResponseCalculator.getForwardDynamicsCalculator().compute();
      multiBodyResponseCalculator.applyWrench(target, testJointEffort);

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
                                                         epsilon);
      }

      DenseMatrix64F qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DenseMatrix64F qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DenseMatrix64F qdd_change = multiBodyResponseCalculator.applyAndPropagateWrench(target, testJointEffort);
      assertJointAccelerationMatrixEquals(iteration, qdd_expected, qdd_original, qdd_change, epsilon);
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
