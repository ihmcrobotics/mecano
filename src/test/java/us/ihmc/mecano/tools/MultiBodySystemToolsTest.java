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

   @Test
   public void testComputeNearestCommonAncestor()
   {
      Random random = new Random(4589634);

      for (int i = 0; i < ITERATIONS; i++)
      { // Perform trivial tests on a random chain.
         int numberOfJoints = random.nextInt(100) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         List<? extends RigidBodyBasics> allBodies = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor()).subtreeList();

         RigidBodyBasics firstBody = allBodies.get(random.nextInt(allBodies.size()));
         RigidBodyBasics secondBody = allBodies.get(random.nextInt(allBodies.size()));

         RigidBodyBasics expectedNearestAncestor = MultiBodySystemTools.isAncestor(secondBody, firstBody) ? firstBody : secondBody;
         RigidBodyBasics actualNearestAncestor = MultiBodySystemTools.computeNearestCommonAncestor(firstBody, secondBody);
         assertTrue(expectedNearestAncestor == actualNearestAncestor);
         actualNearestAncestor = MultiBodySystemTools.computeNearestCommonAncestor(secondBody, firstBody);
         assertTrue(expectedNearestAncestor == actualNearestAncestor);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         /*
          * Setup random number of joints in a Y-shaped tree: 1 common trunk and 2 branches starting off the
          * end of the trunk. We pick 1 body per branch, the common ancestor should be at the bifurcation.
          */
         int trunkSize = random.nextInt(30) + 1;
         int branch1Size = random.nextInt(30) + 1;
         int branch2Size = random.nextInt(30) + 1;
         List<JointBasics> trunk = MultiBodySystemRandomTools.nextJointChain(random, trunkSize);
         RigidBodyBasics bifurcation = trunk.get(trunkSize - 1).getSuccessor();
         List<JointBasics> branch1 = MultiBodySystemRandomTools.nextJointChain(random, bifurcation, branch1Size);
         List<JointBasics> branch2 = MultiBodySystemRandomTools.nextJointChain(random, bifurcation, branch2Size);

         RigidBodyBasics firstBody = branch1.get(random.nextInt(branch1Size)).getSuccessor();
         RigidBodyBasics secondBody = branch2.get(random.nextInt(branch2Size)).getSuccessor();
         assertFalse(MultiBodySystemTools.isAncestor(firstBody, secondBody));
         assertFalse(MultiBodySystemTools.isAncestor(secondBody, firstBody));
         assertTrue(bifurcation == MultiBodySystemTools.computeNearestCommonAncestor(firstBody, secondBody));
         assertTrue(bifurcation == MultiBodySystemTools.computeNearestCommonAncestor(secondBody, firstBody));
      }

      { // Purely random generation, asserting properties of the common ancestor.
         int numberOfJoints = 500;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);

         for (int i = 0; i < ITERATIONS; i++)
         {
            RigidBodyBasics firstBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics secondBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics ancestor = MultiBodySystemTools.computeNearestCommonAncestor(firstBody, secondBody);
            assertTrue(ancestor == MultiBodySystemTools.computeNearestCommonAncestor(secondBody, firstBody));

            assertTrue(MultiBodySystemTools.isAncestor(firstBody, ancestor));
            assertTrue(MultiBodySystemTools.isAncestor(secondBody, ancestor));
            if (ancestor != firstBody && ancestor != secondBody)
            {
               assertTrue(ancestor.getChildrenJoints().size() > 1);

               for (JointBasics ancestorChild : ancestor.getChildrenJoints())
               {
                  boolean isChildAncestorOf1 = MultiBodySystemTools.isAncestor(firstBody, ancestorChild.getSuccessor());
                  boolean isChildAncestorOf2 = MultiBodySystemTools.isAncestor(secondBody, ancestorChild.getSuccessor());
                  assertTrue((isChildAncestorOf1 != isChildAncestorOf2) || (!isChildAncestorOf1 && !isChildAncestorOf2));
               }
            }
         }
      }

      { // Check exception
         int size1 = random.nextInt(100) + 1;
         int size2 = random.nextInt(100) + 1;
         List<JointBasics> system1 = MultiBodySystemRandomTools.nextJointTree(random, size1);
         List<JointBasics> system2 = MultiBodySystemRandomTools.nextJointTree(random, size2);

         assertThrows(IllegalArgumentException.class,
                      () -> MultiBodySystemTools.computeNearestCommonAncestor(system1.get(random.nextInt(size1 - 1)).getSuccessor(),
                                                                              system2.get(random.nextInt(size2 - 1)).getSuccessor()));
      }
   }
}
