package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.*;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import static org.junit.jupiter.api.Assertions.*;

@Disabled
public class InverseDynamicsCalculatorTest
{
   private static final int WARMUP_ITERATIONS = 5000;
   private static final int ITERATIONS = 50000;
   private static final double EPSILON = 1.0e-9;

   @Test
   public void benchmarkForOneDoFJointChain()
   {
      Random random = new Random(43);

      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, 30);
      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(joints.get(0).getPredecessor());
      calculator.setGravitionalAcceleration(-9.81);

      long totalTime = 0L;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         calculator.compute();
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         long startTime = System.nanoTime();
         calculator.compute();
         totalTime += System.nanoTime() - startTime;
      }

      LogTools.info("1-DoF chain: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }

   @Test
   public void benchmarkForFloatingOneDoFJointChain()
   {
      Random random = new Random(43);

      List<Joint> joints = new ArrayList<>();
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      joints.add(new SixDoFJoint("floating", elevator));
      RigidBody floatingBody = MultiBodySystemRandomTools.nextRigidBody(random, "floatingBody", joints.get(0));
      joints.addAll(MultiBodySystemRandomTools.nextOneDoFJointChain(random, floatingBody, 30));
      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(elevator);
      calculator.setGravitionalAcceleration(-9.81);

      long totalTime = 0L;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         calculator.compute();
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         long startTime = System.nanoTime();
         calculator.compute();
         totalTime += System.nanoTime() - startTime;
      }

      LogTools.info("Floating 1-DoF chain: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }

   @Test
   public void benchmarkForOneDoFJointTree()
   {
      Random random = new Random(43);

      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, 30);
      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(joints.get(0).getPredecessor());
      calculator.setGravitionalAcceleration(-9.81);

      long totalTime = 0L;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         calculator.compute();
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         long startTime = System.nanoTime();
         calculator.compute();
         totalTime += System.nanoTime() - startTime;
      }

      LogTools.info("1-DoF tree: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }

   @Test
   public void benchmarkForFloatingOneDoFJointTree()
   {
      Random random = new Random(43);

      List<Joint> joints = new ArrayList<>();
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      joints.add(new SixDoFJoint("floating", elevator));
      RigidBody floatingBody = MultiBodySystemRandomTools.nextRigidBody(random, "floatingBody", joints.get(0));
      joints.addAll(MultiBodySystemRandomTools.nextOneDoFJointTree(random, floatingBody, 30));
      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(elevator);
      calculator.setGravitionalAcceleration(-9.81);

      long totalTime = 0L;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         calculator.compute();
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         long startTime = System.nanoTime();
         calculator.compute();
         totalTime += System.nanoTime() - startTime;
      }

      LogTools.info("Floating 1-DoF tree: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }

   /**
    * When varying the inertial parameters of a MultiBodySystem, one should expect the inverse dynamics to be invariant to a change of mass position
    * if the joints are all prismatic -- that is, they only induce linear motion.
    */
   @Test
   public void testInverseDynamicsInvariantToCoMOffsetForPrismaticJoints()
   {
      Random random = new Random(334985);

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         int numberOfJJoints = random.nextInt(50) + 1;

         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         JointBasics[] clonedJointsArray = MultiBodySystemFactories.cloneKinematicChain(joints.toArray(JointReadOnly[]::new));
         List<JointBasics> clonedJoints = Arrays.stream(clonedJointsArray).toList();
         MultiBodySystemTools.copyJointsState(joints, clonedJoints, JointStateType.CONFIGURATION);
         MultiBodySystemTools.copyJointsState(joints, clonedJoints, JointStateType.VELOCITY);
         MultiBodySystemTools.copyJointsState(joints, clonedJoints, JointStateType.ACCELERATION);

         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(joints.get(0).getPredecessor());
         inverseDynamicsCalculator.setGravitionalAcceleration(-9.81);
         InverseDynamicsCalculator clonedInverseDynamicsCalculator = new InverseDynamicsCalculator(clonedJoints.get(0).getPredecessor());
         clonedInverseDynamicsCalculator.setGravitionalAcceleration(-9.81);

         clonedJoints.get(0).getSuccessor().getInertia().getCenterOfMassOffset().addX(random.nextDouble());
         clonedJoints.get(0).getSuccessor().getInertia().getCenterOfMassOffset().addY(random.nextDouble());
         clonedJoints.get(0).getSuccessor().getInertia().getCenterOfMassOffset().addZ(random.nextDouble());

         inverseDynamicsCalculator.compute();
         clonedInverseDynamicsCalculator.compute();

         DMatrixRMaj tau = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
         DMatrixRMaj clonedTau = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(clonedJoints), 1);

         tau.set(inverseDynamicsCalculator.getJointTauMatrix());
         clonedTau.set(clonedInverseDynamicsCalculator.getJointTauMatrix());

         assertEquals(tau.getData().length, clonedTau.getData().length);
         assertArrayEquals(tau.getData(), clonedTau.getData(), EPSILON);
      }
   }
}
