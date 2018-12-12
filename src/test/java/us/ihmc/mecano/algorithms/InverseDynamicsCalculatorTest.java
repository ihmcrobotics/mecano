package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

@Disabled
public class InverseDynamicsCalculatorTest
{
   private static final int WARMUP_ITERATIONS = 5000;
   private static final int ITERATIONS = 50000;

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

      System.out.println("1-DoF chain: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
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

      System.out.println("Floating 1-DoF chain: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
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

      System.out.println("1-DoF tree: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
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

      System.out.println("Floating 1-DoF tree: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }
}
