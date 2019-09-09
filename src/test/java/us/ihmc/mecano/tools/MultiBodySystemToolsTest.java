package us.ihmc.mecano.tools;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
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
   public void testCollectJointPath()
   {
      Random random = new Random(5436);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial test on a single chain system.
         int numberOfJoints = random.nextInt(100) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         int startJointIndex = random.nextInt(numberOfJoints);
         int endJointIndex = random.nextInt(numberOfJoints - startJointIndex) + startJointIndex;
         List<JointBasics> expectedPath = joints.subList(startJointIndex, endJointIndex + 1);
         RigidBodyBasics start = expectedPath.get(0).getPredecessor();
         RigidBodyBasics end = expectedPath.get(expectedPath.size() - 1).getSuccessor();
         List<JointBasics> actualPathBasics = new ArrayList<>();
         MultiBodySystemTools.collectJointPath(start, end, actualPathBasics);
         assertEquals(expectedPath, actualPathBasics);
         List<JointReadOnly> actualPathReadOnly = new ArrayList<>();
         MultiBodySystemTools.collectJointPath((RigidBodyReadOnly) start, (RigidBodyReadOnly) end, actualPathReadOnly);
         assertEquals(expectedPath, actualPathReadOnly);

         Collections.reverse(expectedPath);
         MultiBodySystemTools.collectJointPath(end, start, actualPathBasics);
         assertEquals(expectedPath, actualPathBasics);
         MultiBodySystemTools.collectJointPath((RigidBodyReadOnly) end, (RigidBodyReadOnly) start, actualPathReadOnly);
         assertEquals(expectedPath, actualPathReadOnly);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test on a Y-shape tree system.
         int trunkSize = random.nextInt(20) + 1;
         int branch0Size = random.nextInt(20) + 1;
         int branch1Size = random.nextInt(20) + 1;

         List<JointBasics> trunk = MultiBodySystemRandomTools.nextJointChain(random, "trunk", trunkSize);
         RigidBodyBasics bifurcation = trunk.get(trunkSize - 1).getSuccessor();
         List<JointBasics> branch0 = MultiBodySystemRandomTools.nextJointChain(random, "branch0", bifurcation, branch0Size);
         List<JointBasics> branch1 = MultiBodySystemRandomTools.nextJointChain(random, "branch1", bifurcation, branch1Size);

         int startIndex = random.nextInt(branch0Size);
         RigidBodyBasics start = branch0.get(startIndex).getSuccessor();
         int endIndex = random.nextInt(branch1Size);
         RigidBodyBasics end = branch1.get(endIndex).getSuccessor();

         List<JointBasics> pathOnBranch0 = new ArrayList<>(branch0.subList(0, startIndex + 1));
         List<JointBasics> pathOnBranch1 = new ArrayList<>(branch1.subList(0, endIndex + 1));
         Collections.reverse(pathOnBranch0);
         List<JointBasics> expectedPath = new ArrayList<>();
         expectedPath.addAll(pathOnBranch0);
         expectedPath.addAll(pathOnBranch1);

         List<JointBasics> actualPath = new ArrayList<>();
         MultiBodySystemTools.collectJointPath(start, end, actualPath);
         assertEquals(expectedPath, actualPath);
         List<JointReadOnly> actualPathReadOnly = new ArrayList<>();
         MultiBodySystemTools.collectJointPath((RigidBodyReadOnly) start, (RigidBodyReadOnly) end, actualPathReadOnly);
         assertEquals(expectedPath, actualPathReadOnly);

         Collections.reverse(expectedPath);
         MultiBodySystemTools.collectJointPath(end, start, actualPath);
         assertEquals(expectedPath, actualPath);
         MultiBodySystemTools.collectJointPath((RigidBodyReadOnly) end, (RigidBodyReadOnly) start, actualPathReadOnly);
         assertEquals(expectedPath, actualPathReadOnly);
      }

      { // Purely random generation, using a different approach to compute the path.
         int numberOfJoints = 500;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);

         for (int i = 0; i < ITERATIONS; i++)
         {
            RigidBodyBasics firstBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics secondBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics ancestor = MultiBodySystemTools.computeNearestCommonAncestor(firstBody, secondBody);

            List<JointBasics> pathFirstHalf = new ArrayList<>();
            for (RigidBodyBasics body = firstBody; body != ancestor; body = body.getParentJoint().getPredecessor())
               pathFirstHalf.add(body.getParentJoint());
            List<JointBasics> pathSecondHalf = new ArrayList<>();
            for (RigidBodyBasics body = secondBody; body != ancestor; body = body.getParentJoint().getPredecessor())
               pathSecondHalf.add(body.getParentJoint());
            Collections.reverse(pathSecondHalf);

            List<JointBasics> expectedPath = new ArrayList<>();
            expectedPath.addAll(pathFirstHalf);
            expectedPath.addAll(pathSecondHalf);

            List<JointBasics> actualPath = new ArrayList<>();
            MultiBodySystemTools.collectJointPath(firstBody, secondBody, actualPath);
            assertEquals(expectedPath, actualPath);
            List<JointReadOnly> actualPathReadOnly = new ArrayList<>();
            MultiBodySystemTools.collectJointPath((RigidBodyReadOnly) firstBody, (RigidBodyReadOnly) secondBody, actualPathReadOnly);
            assertEquals(expectedPath, actualPathReadOnly);

            Collections.reverse(expectedPath);
            MultiBodySystemTools.collectJointPath(secondBody, firstBody, actualPath);
            assertEquals(expectedPath, actualPath);
            MultiBodySystemTools.collectJointPath((RigidBodyReadOnly) secondBody, (RigidBodyReadOnly) firstBody, actualPathReadOnly);
            assertEquals(expectedPath, actualPathReadOnly);
         }
      }
   }

   @Test
   public void testCollectRigidBodyPath()
   {
      Random random = new Random(5436);

      for (int i = 0; i < ITERATIONS; i++)
      { // Trivial test on a single chain system.
         int numberOfJoints = random.nextInt(100) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         int startJointIndex = random.nextInt(numberOfJoints);
         int endJointIndex = random.nextInt(numberOfJoints - startJointIndex) + startJointIndex;
         List<RigidBodyBasics> expectedPath = joints.subList(startJointIndex, endJointIndex + 1).stream().map(JointBasics::getPredecessor)
                                                    .collect(Collectors.toList());
         RigidBodyBasics start = expectedPath.get(0);
         RigidBodyBasics end = expectedPath.get(expectedPath.size() - 1);
         List<RigidBodyBasics> actualPathBasics = new ArrayList<>();
         MultiBodySystemTools.collectRigidBodyPath(start, end, actualPathBasics);
         assertEquals(expectedPath, actualPathBasics);
         List<RigidBodyReadOnly> actualPathReadOnly = new ArrayList<>();
         MultiBodySystemTools.collectRigidBodyPath((RigidBodyReadOnly) start, (RigidBodyReadOnly) end, actualPathReadOnly);
         assertEquals(expectedPath, actualPathReadOnly);

         Collections.reverse(expectedPath);
         MultiBodySystemTools.collectRigidBodyPath(end, start, actualPathBasics);
         assertEquals(expectedPath, actualPathBasics);
         MultiBodySystemTools.collectRigidBodyPath((RigidBodyReadOnly) end, (RigidBodyReadOnly) start, actualPathReadOnly);
         assertEquals(expectedPath, actualPathReadOnly);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test on a Y-shape tree system.
         int trunkSize = random.nextInt(20) + 1;
         int branch0Size = random.nextInt(20) + 1;
         int branch1Size = random.nextInt(20) + 1;

         List<JointBasics> trunk = MultiBodySystemRandomTools.nextJointChain(random, "trunk", trunkSize);
         RigidBodyBasics bifurcation = trunk.get(trunkSize - 1).getSuccessor();
         List<JointBasics> branch0 = MultiBodySystemRandomTools.nextJointChain(random, "branch0", bifurcation, branch0Size);
         List<JointBasics> branch1 = MultiBodySystemRandomTools.nextJointChain(random, "branch1", bifurcation, branch1Size);

         List<RigidBodyBasics> pathOnBranch0 = branch0.subList(0, random.nextInt(branch0Size) + 1).stream().map(JointBasics::getSuccessor)
                                                      .collect(Collectors.toList());
         List<RigidBodyBasics> pathOnBranch1 = branch1.subList(0, random.nextInt(branch1Size) + 1).stream().map(JointBasics::getSuccessor)
                                                      .collect(Collectors.toList());
         RigidBodyBasics start = pathOnBranch0.get(pathOnBranch0.size() - 1);
         RigidBodyBasics end = pathOnBranch1.get(pathOnBranch1.size() - 1);

         Collections.reverse(pathOnBranch0);
         List<RigidBodyBasics> expectedPath = new ArrayList<>();
         expectedPath.addAll(pathOnBranch0);
         expectedPath.add(bifurcation);
         expectedPath.addAll(pathOnBranch1);

         for (int j = 1; j < expectedPath.size(); j++)
         { // Just to make sure that the ancestor has to be added manually.
            assertTrue(expectedPath.get(j) != expectedPath.get(j - 1));
         }

         List<RigidBodyBasics> actualPath = new ArrayList<>();
         MultiBodySystemTools.collectRigidBodyPath(start, end, actualPath);
         assertEquals(expectedPath, actualPath);
         List<RigidBodyReadOnly> actualPathReadOnly = new ArrayList<>();
         MultiBodySystemTools.collectRigidBodyPath((RigidBodyReadOnly) start, (RigidBodyReadOnly) end, actualPathReadOnly);
         assertEquals(expectedPath, actualPathReadOnly);

         Collections.reverse(expectedPath);
         MultiBodySystemTools.collectRigidBodyPath(end, start, actualPath);
         assertEquals(expectedPath, actualPath);
         MultiBodySystemTools.collectRigidBodyPath((RigidBodyReadOnly) end, (RigidBodyReadOnly) start, actualPathReadOnly);
         assertEquals(expectedPath, actualPathReadOnly);
      }

      { // Purely random generation, using a different approach to compute the path.
         int numberOfJoints = 500;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);

         for (int i = 0; i < ITERATIONS; i++)
         {
            RigidBodyBasics firstBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics secondBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics ancestor = MultiBodySystemTools.computeNearestCommonAncestor(firstBody, secondBody);

            List<RigidBodyBasics> pathFirstHalf = new ArrayList<>();
            for (RigidBodyBasics body = firstBody; body != ancestor; body = body.getParentJoint().getPredecessor())
               pathFirstHalf.add(body);
            List<RigidBodyBasics> pathSecondHalf = new ArrayList<>();
            for (RigidBodyBasics body = secondBody; body != ancestor; body = body.getParentJoint().getPredecessor())
               pathSecondHalf.add(body);
            Collections.reverse(pathSecondHalf);

            List<RigidBodyBasics> expectedPath = new ArrayList<>();
            expectedPath.addAll(pathFirstHalf);
            expectedPath.add(ancestor);
            expectedPath.addAll(pathSecondHalf);

            for (int j = 1; j < expectedPath.size(); j++)
            { // Just to make sure that the ancestor has to be added manually.
               assertTrue(expectedPath.get(j) != expectedPath.get(j - 1));
            }

            List<RigidBodyBasics> actualPath = new ArrayList<>();
            MultiBodySystemTools.collectRigidBodyPath(firstBody, secondBody, actualPath);
            assertEquals(expectedPath, actualPath);
            List<RigidBodyReadOnly> actualPathReadOnly = new ArrayList<>();
            MultiBodySystemTools.collectRigidBodyPath((RigidBodyReadOnly) firstBody, (RigidBodyReadOnly) secondBody, actualPathReadOnly);
            assertEquals(expectedPath, actualPathReadOnly);

            Collections.reverse(expectedPath);
            MultiBodySystemTools.collectRigidBodyPath(secondBody, firstBody, actualPath);
            assertEquals(expectedPath, actualPath);
            MultiBodySystemTools.collectRigidBodyPath((RigidBodyReadOnly) secondBody, (RigidBodyReadOnly) firstBody, actualPathReadOnly);
            assertEquals(expectedPath, actualPathReadOnly);
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
   public void testComputeDegreesOfFreedom()
   {
      Random random = new Random(1646541);

      for (int i = 0; i < ITERATIONS; i++)
      { // Setup random number of 1-DoF joints in a chain, nDoFs should be equal to the number of joints.
         int numberOfJoints = random.nextInt(100) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);

         int index2 = random.nextInt(numberOfJoints);
         int index1 = random.nextInt(index2 + 1);

         int expectedDoFs = index2 - index1 + 1;
         int actualDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints.get(index1).getPredecessor(), joints.get(index2).getSuccessor());
         assertEquals(expectedDoFs, actualDoFs);
         actualDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints.get(index2).getSuccessor(), joints.get(index1).getPredecessor());
         assertEquals(expectedDoFs, actualDoFs);

         actualDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints.subList(index1, index2 + 1));
         assertEquals(expectedDoFs, actualDoFs);

         actualDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints.subList(index1, index2 + 1).toArray(new JointBasics[0]));
         assertEquals(expectedDoFs, actualDoFs);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Setup random number of 1-DoF joints in a tree, nDoFs should be equal to the number of joints. Picking a common ancestor and 2 descendants from different branches.
         int numberOfJoints = random.nextInt(100) + 1;
         List<OneDoFJoint> mainBranchJoints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         int branch1ParentJointIndex = random.nextInt(numberOfJoints);
         int branch2ParentJointIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics branch1Root = mainBranchJoints.get(branch1ParentJointIndex).getSuccessor();
         RigidBodyBasics branch2Root = mainBranchJoints.get(branch2ParentJointIndex).getSuccessor();
         List<OneDoFJoint> branch1Joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, branch1Root, numberOfJoints);
         List<OneDoFJoint> branch2Joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, branch2Root, numberOfJoints);

         RigidBodyBasics commonAncestor = MultiBodySystemTools.isAncestor(branch2Root, branch1Root) ? branch1Root : branch2Root;

         RigidBodyBasics body1 = branch1Joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         RigidBodyBasics body2 = branch2Joints.get(random.nextInt(numberOfJoints)).getSuccessor();

         int expectedDoFs = MultiBodySystemTools.computeDegreesOfFreedom(commonAncestor, body1)
               + MultiBodySystemTools.computeDegreesOfFreedom(commonAncestor, body2);
         int actualDoFs = MultiBodySystemTools.computeDegreesOfFreedom(body1, body2);
         assertEquals(expectedDoFs, actualDoFs);
      }

      { // Purely random generation, simple assertions using common ancestor.
         int numberOfJoints = 500;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);

         for (int i = 0; i < ITERATIONS; i++)
         {
            RigidBodyBasics firstBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics secondBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics ancestor = MultiBodySystemTools.computeNearestCommonAncestor(firstBody, secondBody);

            int expectedDoFs = MultiBodySystemTools.computeDegreesOfFreedom(firstBody, ancestor)
                  + MultiBodySystemTools.computeDegreesOfFreedom(secondBody, ancestor);
            int actualDoFs = MultiBodySystemTools.computeDegreesOfFreedom(firstBody, secondBody);
            assertEquals(expectedDoFs, actualDoFs);
            actualDoFs = MultiBodySystemTools.computeDegreesOfFreedom(secondBody, firstBody);
            assertEquals(expectedDoFs, actualDoFs);
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

   @Test
   public void testComputeDistance()
   {
      Random random = new Random(1646541);

      for (int i = 0; i < ITERATIONS; i++)
      { // Setup random number of joints in a chain.
         int numberOfJoints = random.nextInt(100) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);

         int index2 = random.nextInt(numberOfJoints);
         int index1 = random.nextInt(index2 + 1);

         int expectedDoFs = index2 - index1 + 1;
         int actualDoFs = MultiBodySystemTools.computeDistance(joints.get(index1).getPredecessor(), joints.get(index2).getSuccessor());
         assertEquals(expectedDoFs, actualDoFs);
         actualDoFs = MultiBodySystemTools.computeDistance(joints.get(index2).getSuccessor(), joints.get(index1).getPredecessor());
         assertEquals(expectedDoFs, actualDoFs);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Setup random number of joints in a tree. Picking a common ancestor and 2 descendants from different branches.
         int numberOfJoints = random.nextInt(100) + 1;
         List<JointBasics> mainBranchJoints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         int branch1ParentJointIndex = random.nextInt(numberOfJoints);
         int branch2ParentJointIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics branch1Root = mainBranchJoints.get(branch1ParentJointIndex).getSuccessor();
         RigidBodyBasics branch2Root = mainBranchJoints.get(branch2ParentJointIndex).getSuccessor();
         List<JointBasics> branch1Joints = MultiBodySystemRandomTools.nextJointChain(random, branch1Root, numberOfJoints);
         List<JointBasics> branch2Joints = MultiBodySystemRandomTools.nextJointChain(random, branch2Root, numberOfJoints);

         RigidBodyBasics commonAncestor = MultiBodySystemTools.isAncestor(branch2Root, branch1Root) ? branch1Root : branch2Root;

         RigidBodyBasics body1 = branch1Joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         RigidBodyBasics body2 = branch2Joints.get(random.nextInt(numberOfJoints)).getSuccessor();

         int expectedDoFs = MultiBodySystemTools.computeDistance(commonAncestor, body1) + MultiBodySystemTools.computeDistance(commonAncestor, body2);
         int actualDoFs = MultiBodySystemTools.computeDistance(body1, body2);
         assertEquals(expectedDoFs, actualDoFs);
      }

      { // Purely random generation, simple assertions using common ancestor.
         int numberOfJoints = 500;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);

         for (int i = 0; i < ITERATIONS; i++)
         {
            RigidBodyBasics firstBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics secondBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
            RigidBodyBasics ancestor = MultiBodySystemTools.computeNearestCommonAncestor(firstBody, secondBody);

            int expectedDoFs = MultiBodySystemTools.computeDistance(firstBody, ancestor) + MultiBodySystemTools.computeDistance(secondBody, ancestor);
            int actualDoFs = MultiBodySystemTools.computeDistance(firstBody, secondBody);
            assertEquals(expectedDoFs, actualDoFs);
            actualDoFs = MultiBodySystemTools.computeDistance(secondBody, firstBody);
            assertEquals(expectedDoFs, actualDoFs);
         }
      }
   }
}
