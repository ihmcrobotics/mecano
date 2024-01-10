package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.TablePrinter.Alignment;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.*;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;

import java.util.*;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

public class MultiBodyResponseCalculatorTest
{
   private static final int ITERATIONS = 500;
   private static final double EPSILON = 1.0e-12;
   private static final double JOINT_EPSILON = 1.0e-9;

   @Test
   public void testSpatialAccelerationIntegration()
   { // Test used to demonstrate how to integrate spatial acceleration to predict the twist of a rigid-body.
      Random random = new Random(4757);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      {
         double gravity = EuclidCoreRandomTools.nextDouble(random, -20.0, -1.0);
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);

         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);

         for (JointStateType state : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, state, joints);

         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         rootBody.updateFramesRecursively();

         /*
          * When integrating, the centrifugal and Coriolis accelerations have to be ignored. Still the option
          * that there's another way to do this while considering bias acceleration.
          */
         boolean considerVelocities = false;
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, worldFrame, considerVelocities);
         spatialAccelerationCalculator.setGravitionalAcceleration(gravity);

         RigidBodyBasics rigidBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();

         Twist bodyTwistOld = new Twist(rigidBody.getBodyFixedFrame().getTwistOfFrame());
         // When integrating, need to ignore the gravitational acceleration of the root body.
         Twist bodyAccelerationIntegrated = new Twist(spatialAccelerationCalculator.getRelativeAcceleration(rootBody, rigidBody));
         bodyAccelerationIntegrated.scale(dt);

         Twist bodyTwistNewActual = new Twist(bodyTwistOld);
         bodyTwistNewActual.add((SpatialVectorReadOnly) bodyAccelerationIntegrated);

         joints.forEach(joint -> joint.setQd(joint.getQd() + dt * joint.getQdd()));
         rootBody.updateFramesRecursively();

         TwistReadOnly bodyTwistNewExpected = rigidBody.getBodyFixedFrame().getTwistOfFrame();

         MecanoTestTools.assertTwistEquals("Iteration " + i, bodyTwistNewExpected, bodyTwistNewActual, EPSILON);
      }
   }

   @Test
   public void testPrismaticJointChain()
   {
      Random random = new Random(435346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulseViaIntegration(random, i, joints, dt, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertApplySingleJointImpulseViaIntegration(random, i, joints, dt, EPSILON);
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
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulseViaIntegration(random, i, joints, dt, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertApplySingleJointImpulseViaIntegration(random, i, joints, dt, EPSILON);
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
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulseViaIntegration(random, i, joints, dt, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertApplySingleJointImpulseViaIntegration(random, i, joints, dt, EPSILON);
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
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulseViaIntegration(random, i, joints, dt, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertApplySingleJointImpulseViaIntegration(random, i, joints, dt, EPSILON);
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
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulseViaIntegration(random, i, joints, dt, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertApplySingleJointImpulseViaIntegration(random, i, joints, dt, EPSILON);
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
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulseViaIntegration(random, i, joints, dt, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertApplySingleJointImpulseViaIntegration(random, i, joints, dt, EPSILON);
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
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);
         assertApplySingleRigidBodyWrench(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, EPSILON);
         assertApplySingleRigidBodyImpulseViaIntegration(random, i, joints, dt, EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, EPSILON);

         assertApplySingleJointWrench(random, i, joints, EPSILON);
         assertApplySingleJointImpulse(random, i, joints, EPSILON);
         assertApplySingleJointImpulseViaIntegration(random, i, joints, dt, EPSILON);
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
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-2);
         assertApplySingleRigidBodyWrench(random, i, joints, JOINT_EPSILON);
         assertApplySingleRigidBodyImpulse(random, i, joints, JOINT_EPSILON);
         assertApplySingleRigidBodyImpulseViaIntegration(random, i, joints, dt, JOINT_EPSILON);
         assertRigidBodyApparentInertiaInverse(random, i, joints, JOINT_EPSILON);
         assertRigidBodyApparentLinearInertiaInverse(random, i, joints, JOINT_EPSILON);

         assertApplySingleJointWrench(random, i, joints, JOINT_EPSILON);
         assertApplySingleJointImpulse(random, i, joints, JOINT_EPSILON);
         assertApplySingleJointImpulseViaIntegration(random, i, joints, dt, JOINT_EPSILON);
         assertJointApparentInertiaInverse(random, i, joints, JOINT_EPSILON);

         assertApplyMultipleWrenches(random, i, joints, JOINT_EPSILON);
         assertApplyMultipleImpulses(random, i, joints, JOINT_EPSILON);
      }
   }

   private static void assertApplySingleRigidBodyWrench(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplySingleRigidBodyWrench(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleRigidBodyWrench(Random random,
                                                        int iteration,
                                                        List<? extends JointBasics> joints,
                                                        Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                        List<? extends JointReadOnly> jointsToIgnore,
                                                        double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

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

      DMatrixRMaj qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DMatrixRMaj qdd_change = multiBodyResponseCalculator.propagateWrench();
      assertJointAccelerationMatrixEquals(multiBodySystemInput,
                                          iteration,
                                          qdd_expected,
                                          qdd_original,
                                          qdd_change,
                                          Math.max(1.0, CommonOps_DDRM.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertApplySingleRigidBodyImpulse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplySingleRigidBodyImpulse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleRigidBodyImpulse(Random random,
                                                         int iteration,
                                                         List<? extends JointBasics> joints,
                                                         Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                         List<? extends JointReadOnly> jointsToIgnore,
                                                         double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

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

      DMatrixRMaj qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DMatrixRMaj qdd_change = multiBodyResponseCalculator.propagateImpulse();
      assertJointAccelerationMatrixEquals(multiBodySystemInput,
                                          iteration,
                                          qdd_expected,
                                          qdd_original,
                                          qdd_change,
                                          Math.max(1.0, CommonOps_DDRM.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertApplySingleRigidBodyImpulseViaIntegration(Random random,
                                                                       int iteration,
                                                                       List<? extends JointBasics> joints,
                                                                       double dt,
                                                                       double epsilon)
   {
      assertApplySingleRigidBodyImpulseViaIntegration(random, iteration, joints, dt, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleRigidBodyImpulseViaIntegration(Random random,
                                                                       int iteration,
                                                                       List<? extends JointBasics> joints,
                                                                       double dt,
                                                                       Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                       List<? extends JointReadOnly> jointsToIgnore,
                                                                       double epsilon)
   {
      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);

      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();
      DMatrixRMaj qd_original = new DMatrixRMaj(numberOfDoFs, 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, qd_original);

      RigidBodyBasics target = joints.get(random.nextInt(joints.size())).getSuccessor();

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      Wrench externalWrench = MecanoRandomTools.nextWrench(random, target.getBodyFixedFrame(), target.getBodyFixedFrame(), 1000.0, 1000.0);
      SpatialImpulse externalImpulse = new SpatialImpulse(target.getBodyFixedFrame(), externalWrench);
      externalImpulse.scale(dt);

      ForwardDynamicsCalculator forwardDynamicsCalculator = setupForwardDynamicsCalculator(multiBodySystemInput,
                                                                                           gravity,
                                                                                           externalWrenches,
                                                                                           Collections.singletonMap(target, externalWrench),
                                                                                           Collections.emptyMap());
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);
      multiBodyResponseCalculator.applyRigidBodyImpulse(target, externalImpulse);

      DMatrixRMaj qdd = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj qd_expected = new DMatrixRMaj(numberOfDoFs, 1);
      CommonOps_DDRM.add(qd_original, dt, qdd, qd_expected);

      DMatrixRMaj qd_change = multiBodyResponseCalculator.propagateImpulse();
      DMatrixRMaj qd_noImpulse = new DMatrixRMaj(numberOfDoFs, 1);
      CommonOps_DDRM.add(qd_original, dt, multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix(), qd_noImpulse);
      assertJointAccelerationMatrixEquals(multiBodySystemInput, iteration, qd_expected, qd_noImpulse, qd_change, epsilon);

      TwistReadOnly targetTwistChangeActual = multiBodyResponseCalculator.getTwistChangeProvider().getTwistOfBody(target);
      Twist targetTwistOld = new Twist(target.getBodyFixedFrame().getTwistOfFrame());
      Twist targetAccelerationIntegrated = new Twist(multiBodyResponseCalculator.getForwardDynamicsCalculator()
                                                                                .getAccelerationProvider(false)
                                                                                .getRelativeAcceleration(rootBody, target));
      targetAccelerationIntegrated.scale(dt);

      Twist targetTwistNewActual = new Twist(targetTwistOld);
      targetTwistNewActual.add((SpatialVectorReadOnly) targetAccelerationIntegrated);
      targetTwistNewActual.add((SpatialVectorReadOnly) targetTwistChangeActual);

      MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, qd_expected);
      rootBody.updateFramesRecursively();
      TwistReadOnly targetTwistNewExpected = target.getBodyFixedFrame().getTwistOfFrame();

      Twist targetTwistChangeExpected = new Twist(targetTwistNewExpected);
      targetTwistChangeExpected.sub((SpatialVectorReadOnly) targetAccelerationIntegrated);
      targetTwistChangeExpected.sub((SpatialVectorReadOnly) targetTwistOld);

      MecanoTestTools.assertTwistEquals("Iteration: " + iteration, targetTwistChangeExpected, targetTwistChangeActual, epsilon);
      MecanoTestTools.assertTwistEquals("Iteration: " + iteration, targetTwistNewExpected, targetTwistNewActual, epsilon);
   }

   private static void assertRigidBodyApparentInertiaInverse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertRigidBodyApparentInertiaInverse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertRigidBodyApparentInertiaInverse(Random random,
                                                             int iteration,
                                                             List<? extends JointBasics> joints,
                                                             Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                             List<? extends JointReadOnly> jointsToIgnore,
                                                             double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

      RigidBodyBasics target = joints.get(random.nextInt(joints.size())).getSuccessor();
      ReferenceFrame testWrenchFrame = EuclidFrameRandomTools.nextReferenceFrame(random, target.getBodyFixedFrame());
      Wrench testWrench = MecanoRandomTools.nextWrench(random, target.getBodyFixedFrame(), testWrenchFrame);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);

      DMatrixRMaj apparentSpatialInertiaInverse = new DMatrixRMaj(6, 6);
      multiBodyResponseCalculator.computeRigidBodyApparentSpatialInertiaInverse(target, testWrenchFrame, apparentSpatialInertiaInverse);
      DMatrixRMaj accelerationChangeMatrix = new DMatrixRMaj(6, 1);
      DMatrixRMaj testWrenchMatrix = new DMatrixRMaj(6, 1);
      testWrench.get(testWrenchMatrix);
      CommonOps_DDRM.mult(apparentSpatialInertiaInverse, testWrenchMatrix, accelerationChangeMatrix);
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

   private static void assertRigidBodyApparentLinearInertiaInverse(Random random,
                                                                   int iteration,
                                                                   List<? extends JointBasics> joints,
                                                                   Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                   List<? extends JointReadOnly> jointsToIgnore,
                                                                   double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

      RigidBodyBasics target = joints.get(random.nextInt(joints.size())).getSuccessor();
      ReferenceFrame testWrenchFrame = EuclidFrameRandomTools.nextReferenceFrame(random, target.getBodyFixedFrame());
      Wrench testWrench = MecanoRandomTools.nextWrench(random, target.getBodyFixedFrame(), testWrenchFrame);
      testWrench.getAngularPart().setToZero();

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);

      DMatrixRMaj apparentSpatialInertiaInverse = new DMatrixRMaj(3, 3);
      multiBodyResponseCalculator.computeRigidBodyApparentLinearInertiaInverse(target, testWrenchFrame, apparentSpatialInertiaInverse);
      DMatrixRMaj accelerationChangeMatrix = new DMatrixRMaj(3, 1);
      DMatrixRMaj testWrenchMatrix = new DMatrixRMaj(3, 1);
      testWrench.getLinearPart().get(testWrenchMatrix);
      CommonOps_DDRM.mult(apparentSpatialInertiaInverse, testWrenchMatrix, accelerationChangeMatrix);
      FrameVector3D actualLinearAccelerationChange = new FrameVector3D(testWrenchFrame);
      actualLinearAccelerationChange.set(accelerationChangeMatrix);

      multiBodyResponseCalculator.applyRigidBodyWrench(target, testWrench);
      SpatialAcceleration expectedAccelerationChange = new SpatialAcceleration(multiBodyResponseCalculator.getAccelerationChangeProvider()
                                                                                                          .getAccelerationOfBody(target));
      expectedAccelerationChange.changeFrame(testWrenchFrame);
      EuclidFrameTestTools.assertEquals(expectedAccelerationChange.getLinearPart(),
                                        actualLinearAccelerationChange,
                                        Math.max(1.0, expectedAccelerationChange.getLinearPart().norm()) * epsilon);
   }

   private static void assertApplySingleJointWrench(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplySingleJointWrench(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleJointWrench(Random random,
                                                    int iteration,
                                                    List<? extends JointBasics> joints,
                                                    Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                    List<? extends JointReadOnly> jointsToIgnore,
                                                    double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

      JointBasics target = joints.get(random.nextInt(joints.size()));
      DMatrixRMaj testJointWrench = RandomMatrices_DDRM.rectangle(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);

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

      DMatrixRMaj qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DMatrixRMaj qdd_change = multiBodyResponseCalculator.propagateWrench();
      assertJointAccelerationMatrixEquals(multiBodySystemInput,
                                          iteration,
                                          qdd_expected,
                                          qdd_original,
                                          qdd_change,
                                          Math.max(1.0, CommonOps_DDRM.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertApplySingleJointImpulse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplySingleJointImpulse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleJointImpulse(Random random,
                                                     int iteration,
                                                     List<? extends JointBasics> joints,
                                                     Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                     List<? extends JointReadOnly> jointsToIgnore,
                                                     double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

      JointBasics target = joints.get(random.nextInt(joints.size()));
      DMatrixRMaj testJointImpulse = RandomMatrices_DDRM.rectangle(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);

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

      DMatrixRMaj qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DMatrixRMaj qdd_change = multiBodyResponseCalculator.propagateImpulse();
      assertJointAccelerationMatrixEquals(multiBodySystemInput,
                                          iteration,
                                          qdd_expected,
                                          qdd_original,
                                          qdd_change,
                                          Math.max(1.0, CommonOps_DDRM.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertApplySingleJointImpulseViaIntegration(Random random, int iteration, List<? extends JointBasics> joints, double dt, double epsilon)
   {
      assertApplySingleJointImpulseViaIntegration(random, iteration, joints, dt, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplySingleJointImpulseViaIntegration(Random random,
                                                                   int iteration,
                                                                   List<? extends JointBasics> joints,
                                                                   double dt,
                                                                   Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                   List<? extends JointReadOnly> jointsToIgnore,
                                                                   double epsilon)
   {
      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);

      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();
      DMatrixRMaj qd_original = new DMatrixRMaj(numberOfDoFs, 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, qd_original);

      JointBasics target = joints.get(random.nextInt(joints.size()));

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      DMatrixRMaj testJointEffort = RandomMatrices_DDRM.rectangle(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);
      DMatrixRMaj testJointImpulse = new DMatrixRMaj(testJointEffort);
      CommonOps_DDRM.scale(dt, testJointImpulse);

      ForwardDynamicsCalculator forwardDynamicsCalculator = setupForwardDynamicsCalculator(multiBodySystemInput,
                                                                                           gravity,
                                                                                           externalWrenches,
                                                                                           Collections.emptyMap(),
                                                                                           Collections.singletonMap(target, testJointEffort));
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);
      multiBodyResponseCalculator.applyJointImpulse(target, testJointImpulse);

      DMatrixRMaj qdd = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj qd_expected = new DMatrixRMaj(numberOfDoFs, 1);
      CommonOps_DDRM.add(qd_original, dt, qdd, qd_expected);

      DMatrixRMaj qd_change = multiBodyResponseCalculator.propagateImpulse();
      DMatrixRMaj qd_noImpulse = new DMatrixRMaj(numberOfDoFs, 1);
      CommonOps_DDRM.add(qd_original, dt, multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix(), qd_noImpulse);
      assertJointAccelerationMatrixEquals(multiBodySystemInput, iteration, qd_expected, qd_noImpulse, qd_change, epsilon);
   }

   private static void assertJointApparentInertiaInverse(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertJointApparentInertiaInverse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertJointApparentInertiaInverse(Random random,
                                                         int iteration,
                                                         List<? extends JointBasics> joints,
                                                         Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                         List<? extends JointReadOnly> jointsToIgnore,
                                                         double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

      JointBasics target = joints.get(random.nextInt(joints.size()));
      DMatrixRMaj testWrench = RandomMatrices_DDRM.rectangle(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
      MultiBodySystemReadOnly multiBodySystemInput = MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody, jointsToIgnore);
      rootBody.updateFramesRecursively();

      double gravity = EuclidCoreRandomTools.nextDouble(random, -10.0, -1.0);
      MultiBodyResponseCalculator multiBodyResponseCalculator = setupMultiBodyResponseCalculator(multiBodySystemInput, gravity, externalWrenches);

      DMatrixRMaj apparentInertiaInverse = new DMatrixRMaj(target.getDegreesOfFreedom(), target.getDegreesOfFreedom());
      multiBodyResponseCalculator.computeJointApparentInertiaInverse(target, apparentInertiaInverse);
      DMatrixRMaj actualMotionChange = new DMatrixRMaj(target.getDegreesOfFreedom(), 1);
      CommonOps_DDRM.mult(apparentInertiaInverse, testWrench, actualMotionChange);

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

      DMatrixRMaj expectedMotionChange = multiBodyResponseCalculator.getJointAccelerationChange(target);
      assertMatrixEquals(iteration, expectedMotionChange, actualMotionChange, epsilon);
   }

   private static void assertApplyMultipleWrenches(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplyMultipleWrenches(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplyMultipleWrenches(Random random,
                                                   int iteration,
                                                   List<? extends JointBasics> joints,
                                                   Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                   List<? extends JointReadOnly> jointsToIgnore,
                                                   double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

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
      Map<JointBasics, DMatrixRMaj> jointDisturbances = new HashMap<>();

      for (int i = 0; i < numberOfJointDisturbances; i++)
      {
         JointBasics target = joints.get(random.nextInt(joints.size()));
         DMatrixRMaj testWrench = RandomMatrices_DDRM.rectangle(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);
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

      DMatrixRMaj qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DMatrixRMaj qdd_change = multiBodyResponseCalculator.propagateWrench();
      assertJointAccelerationMatrixEquals(multiBodySystemInput,
                                          iteration,
                                          qdd_expected,
                                          qdd_original,
                                          qdd_change,
                                          Math.max(1.0, CommonOps_DDRM.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void assertApplyMultipleImpulses(Random random, int iteration, List<? extends JointBasics> joints, double epsilon)
   {
      assertApplyMultipleImpulse(random, iteration, joints, Collections.emptyMap(), Collections.emptyList(), epsilon);
   }

   private static void assertApplyMultipleImpulse(Random random,
                                                  int iteration,
                                                  List<? extends JointBasics> joints,
                                                  Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                  List<? extends JointReadOnly> jointsToIgnore,
                                                  double epsilon)
   {
      for (JointStateType state : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, state, joints);

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
      Map<JointBasics, DMatrixRMaj> jointDisturbances = new HashMap<>();

      for (int i = 0; i < numberOfJointDisturbances; i++)
      {
         JointBasics target = joints.get(random.nextInt(joints.size()));
         DMatrixRMaj testWrench = RandomMatrices_DDRM.rectangle(target.getDegreesOfFreedom(), 1, -10.0, 10.0, random);
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

      DMatrixRMaj qdd_expected = forwardDynamicsCalculator.getJointAccelerationMatrix();
      DMatrixRMaj qdd_original = multiBodyResponseCalculator.getForwardDynamicsCalculator().getJointAccelerationMatrix();
      DMatrixRMaj qdd_change = multiBodyResponseCalculator.propagateImpulse();
      assertJointAccelerationMatrixEquals(multiBodySystemInput,
                                          iteration,
                                          qdd_expected,
                                          qdd_original,
                                          qdd_change,
                                          Math.max(1.0, CommonOps_DDRM.elementMaxAbs(qdd_expected)) * epsilon);
   }

   private static void runAssertionViaTwistProvider(Random random,
                                                    int iteration,
                                                    List<? extends JointBasics> joints,
                                                    double epsilon,
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
         actualTargetAcceleration.add(twistChangeProvider.getTwistOfBody(body)); // <= Same equations as for the acceleration.
         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + iteration + ", body: " + i,
                                                         expectedTargetAcceleration,
                                                         actualTargetAcceleration,
                                                         Math.max(1.0, expectedTargetAcceleration.length()) * epsilon);
      }
   }

   private static void runAssertionsViaAccelerationProvider(Random random,
                                                            int iteration,
                                                            List<? extends JointBasics> joints,
                                                            double epsilon,
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

   private static void assertMatrixEquals(int iteration, DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      boolean areEqual = MatrixFeatures_DDRM.isEquals(expected, actual, epsilon);
      if (!areEqual)
      {
         System.out.println("iteration: " + iteration);
         double maxError = 0.0;
         DMatrixRMaj output = new DMatrixRMaj(expected.getNumRows(), 3);

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

   private static ForwardDynamicsCalculator setupForwardDynamicsCalculator(MultiBodySystemReadOnly input,
                                                                           double gravity,
                                                                           Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches,
                                                                           Map<? extends RigidBodyReadOnly, ? extends SpatialForceReadOnly> testRigidBodyDisturbances,
                                                                           Map<? extends JointReadOnly, DMatrixRMaj> testJointDisturbances)
   {
      ForwardDynamicsCalculator calculator = new ForwardDynamicsCalculator(input);
      calculator.setGravitationalAcceleration(gravity);
      calculator.setExternalWrenchesToZero();
      externalWrenches.forEach(calculator::setExternalWrench);
      testRigidBodyDisturbances.forEach((body, disturbance) -> calculator.getExternalWrench(body).add(disturbance));

      List<? extends JointReadOnly> joints = input.getJointsToConsider();
      int numberOfDoFs = joints.stream().mapToInt(JointReadOnly::getDegreesOfFreedom).sum();
      DMatrixRMaj tauWithDisturbances = new DMatrixRMaj(numberOfDoFs, 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tauWithDisturbances);
      JointMatrixIndexProvider jointMatrixIndexProvider = input.getJointMatrixIndexProvider();
      for (Entry<? extends JointReadOnly, DMatrixRMaj> entry : testJointDisturbances.entrySet())
      {
         JointReadOnly joint = entry.getKey();
         DMatrixRMaj jointDisturbance = entry.getValue();

         for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
            tauWithDisturbances.add(jointMatrixIndexProvider.getJointDoFIndices(joint)[i], 0, jointDisturbance.get(i));
      }
      calculator.compute(tauWithDisturbances);

      return calculator;
   }

   private static MultiBodyResponseCalculator setupMultiBodyResponseCalculator(MultiBodySystemReadOnly input,
                                                                               double gravity,
                                                                               Map<RigidBodyReadOnly, WrenchReadOnly> externalWrenches)
   {
      MultiBodyResponseCalculator multiBodyResponseCalculator = new MultiBodyResponseCalculator(input);
      ForwardDynamicsCalculator forwardDynamicsCalculator = multiBodyResponseCalculator.getForwardDynamicsCalculator();
      forwardDynamicsCalculator.setGravitationalAcceleration(gravity);

      forwardDynamicsCalculator.setExternalWrenchesToZero();
      externalWrenches.forEach(forwardDynamicsCalculator::setExternalWrench);
      forwardDynamicsCalculator.compute();
      return multiBodyResponseCalculator;
   }

   private static void assertJointAccelerationMatrixEquals(MultiBodySystemReadOnly input,
                                                           int iteration,
                                                           DMatrixRMaj qdd_expected,
                                                           DMatrixRMaj qdd_original,
                                                           DMatrixRMaj qdd_change,
                                                           double epsilon)
   {
      DMatrixRMaj qdd_actual = new DMatrixRMaj(qdd_change.getNumRows(), 1);
      CommonOps_DDRM.add(qdd_original, qdd_change, qdd_actual);

      boolean areEqual = MatrixFeatures_DDRM.isEquals(qdd_expected, qdd_actual, epsilon);
      if (!areEqual)
      {
         LogTools.info("iteration: " + iteration);
         double maxError = 0.0;
         DMatrixRMaj output = new DMatrixRMaj(qdd_expected.getNumRows(), 5);

         for (int row = 0; row < qdd_expected.getNumRows(); row++)
         {
            double error = qdd_expected.get(row, 0) - qdd_actual.get(row, 0);

            output.set(row, 0, qdd_original.get(row, 0));
            output.set(row, 1, qdd_change.get(row, 0));
            output.set(row, 2, qdd_expected.get(row, 0));
            output.set(row, 3, qdd_actual.get(row, 0));
            output.set(row, 4, error);
            maxError = Math.max(maxError, Math.abs(error));
         }

         TablePrinter tablePrinter = new TablePrinter();
         tablePrinter.setColumnSeparator("   ");
         tablePrinter.setRow(0, "Joint", "Successor", "original", "change", "expected", "actual", "error");
         tablePrinter.setSubTable(1, 2, output);

         int row = 1;

         for (JointReadOnly joint : input.getJointMatrixIndexProvider().getIndexedJointsInOrder())
         {
            for (int dof = 0; dof < joint.getDegreesOfFreedom(); dof++)
            {
               tablePrinter.setCell(row, 0, joint.getName(), Alignment.LEFT);
               tablePrinter.setCell(row, 1, joint.getSuccessor().getName(), Alignment.LEFT);
               row++;
            }
         }
         LogTools.info(tablePrinter.toString());
         LogTools.info("Max error: " + maxError);
      }
      assertTrue(areEqual);
   }
}
