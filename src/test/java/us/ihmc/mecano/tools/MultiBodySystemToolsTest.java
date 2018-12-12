package us.ihmc.mecano.tools;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public class MultiBodySystemToolsTest
{
   public static final int ITERATIONS = 1000;

   @Test
   public void testCreateJointPath() throws Exception
   {
      Random random = new Random(34535);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 100;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);

         RigidBodyBasics bodyA = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         RigidBodyBasics bodyB = bodyA.subtreeList().get(random.nextInt(bodyA.subtreeList().size()));

         JointReadOnly[] jointPath = MultiBodySystemTools.createJointPath(bodyA, bodyB);

         assertNotNull(jointPath);

         assertEquals(MultiBodySystemTools.computeDistanceToAncestor(bodyB, bodyA), jointPath.length);

         if (bodyA != bodyB)
         {
            assertEquals(bodyA, jointPath[0].getPredecessor());
            RigidBodyReadOnly expectedPredessor = null;
            int nDoFs = 0;

            for (JointReadOnly joint : jointPath)
            {
               if (expectedPredessor != null)
                  assertEquals(expectedPredessor, joint.getPredecessor());
               expectedPredessor = joint.getSuccessor();
               nDoFs += joint.getDegreesOfFreedom();
            }

            assertEquals(bodyB, jointPath[jointPath.length - 1].getSuccessor());

            assertEquals(nDoFs, MultiBodySystemTools.computeDegreesOfFreedom(bodyA, bodyB));
         }
      }
   }

   @Test
   public void testCollectSuccessors() throws Exception
   {
      Random random = new Random(235423);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 100;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);

         List<JointBasics> jointSelection = joints.stream().filter(j -> random.nextInt(10) < 4).collect(Collectors.toList());
         Collections.shuffle(jointSelection);

         RigidBodyBasics[] successors = MultiBodySystemTools.collectSuccessors(jointSelection.toArray(new JointBasics[0]));

         assertArrayEquals(jointSelection.stream().map(JointBasics::getSuccessor).toArray(RigidBodyBasics[]::new), successors);
      }
   }

   @Test
   public void testCollectSubtreeSuccessors() throws Exception
   {
      Random random = new Random(2354234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 20;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);

         List<JointBasics> jointSelection = joints.stream().filter(j -> random.nextInt(10) < 4).collect(Collectors.toList());
         Collections.shuffle(jointSelection);

         RigidBodyBasics[] subtreeSuccessors = MultiBodySystemTools.collectSubtreeSuccessors(jointSelection.toArray(new JointBasics[0]));

         List<JointBasics> reducedJointSelection = new ArrayList<>();

         for (JointBasics candidate : jointSelection)
         {
            if (reducedJointSelection.contains(candidate))
               continue;

            if (jointSelection.stream().filter(joint -> joint != candidate)
                              .anyMatch(joint -> MultiBodySystemTools.isAncestor(candidate.getSuccessor(), joint.getSuccessor())))
               continue;

            reducedJointSelection.add(candidate);
         }

         Set<RigidBodyBasics> jointPredecessors = reducedJointSelection.stream().map(JointBasics::getPredecessor).collect(Collectors.toSet());

         for (RigidBodyBasics subtreeSuccessor : subtreeSuccessors)
         {
            assertTrue(jointPredecessors.stream().anyMatch(subtreePredecessor -> MultiBodySystemTools.isAncestor(subtreeSuccessor, subtreePredecessor)));
            assertTrue(jointPredecessors.stream().noneMatch(subtreePredecessor -> MultiBodySystemTools.isAncestor(subtreePredecessor, subtreeSuccessor)));
         }
      }
   }
}
