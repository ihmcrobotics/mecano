package us.ihmc.mecano.multiBodySystem.iterators;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Queue;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class JointIterableTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testChain()
   {
      Random random = new Random(43954);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<? extends JointReadOnly> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         IteratorSearchMode mode = EuclidCoreRandomTools.nextElementIn(random, IteratorSearchMode.values());
         JointIterable<JointReadOnly> jointIterable = new JointIterable<>(JointReadOnly.class, null, mode, joints.get(0));
         for (int j = 0; j < 2; j++) // Doing 2 calls to JointIterable.iterator() to make sure the second time the iterator is brand new.
         {
            Iterator<JointReadOnly> iterator = jointIterable.iterator();

            for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
            {
               assertTrue(iterator.hasNext());
               assertTrue(joints.get(jointIndex) == iterator.next());
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing the filtering class with OneDoFJointReadOnly
         int numberOfJoints = random.nextInt(50) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         IteratorSearchMode mode = EuclidCoreRandomTools.nextElementIn(random, IteratorSearchMode.values());
         JointIterable<OneDoFJointReadOnly> jointIterable = new JointIterable<>(OneDoFJointReadOnly.class, null, mode, joints.get(0));
         for (int j = 0; j < 2; j++) // Doing 2 calls to JointIterable.iterator() to make sure the second time the iterator is brand new.
         {
            Iterator<OneDoFJointReadOnly> iterator = jointIterable.iterator();

            for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
            {
               if (joints.get(jointIndex) instanceof OneDoFJointReadOnly)
               {
                  assertTrue(iterator.hasNext());
                  assertTrue(joints.get(jointIndex) == iterator.next());
               }
            }
         }
      }
   }

   @Test
   public void testChainWithKinematicLoop()
   {
      Random random = new Random(43954);

      for (int i = 0; i < ITERATIONS; i++)
      { // We only assert that the iterator returned each joint only once.
         int numberOfJoints = random.nextInt(50) + 2;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         IteratorSearchMode mode = EuclidCoreRandomTools.nextElementIn(random, IteratorSearchMode.values());
         int loopStartIndex = random.nextInt(numberOfJoints);
         int loopEndIndex = random.nextInt(numberOfJoints);

         while (loopEndIndex == loopStartIndex)
            loopEndIndex = random.nextInt(numberOfJoints);

         if (loopStartIndex > loopEndIndex)
         {
            int temp = loopStartIndex;
            loopStartIndex = loopEndIndex;
            loopEndIndex = temp;
         }

         int kinematicLoopSize = random.nextInt(10) + 2;
         RigidBodyBasics loopStart = joints.get(loopStartIndex).getSuccessor();
         RigidBodyBasics loopEnd = joints.get(loopEndIndex).getSuccessor();
         MultiBodySystemRandomTools.nextKinematicLoopRevoluteJoints(random, "loop", loopStart, loopEnd, kinematicLoopSize);

         JointIterable<JointBasics> jointIterable = new JointIterable<>(JointBasics.class, null, mode, joints.get(0));
         List<JointBasics> iterableJoints = new ArrayList<>();
         jointIterable.iterator().forEachRemaining(iterableJoints::add);

         assertEquals(numberOfJoints + kinematicLoopSize, iterableJoints.size());
         assertEquals(new HashSet<>(iterableJoints).size(), iterableJoints.size());
      }
   }

   @Test
   public void testTreeDepth1() throws Exception
   {
      Random random = new Random(324534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         IteratorSearchMode mode = EuclidCoreRandomTools.nextElementIn(random, IteratorSearchMode.values());
         RigidBodyBasics rootBody = new RigidBody("rootBody", ReferenceFrame.getWorldFrame());
         JointBasics rootJoint = MultiBodySystemRandomTools.nextJoint(random, "root", rootBody);
         RigidBody rootJointSuccessor = MultiBodySystemRandomTools.nextRigidBody(random, "rootJointSuccessor", rootJoint);
         int numberOfChildren = 10;
         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            JointBasics childJoint = MultiBodySystemRandomTools.nextJoint(random, "jointDepth1", rootJointSuccessor);
            MultiBodySystemRandomTools.nextRigidBody(random, "bodyDepth1", childJoint);
         }
         JointIterable<JointReadOnly> jointIterable = new JointIterable<>(JointReadOnly.class, null, mode, rootJoint);
         Iterator<JointReadOnly> iterator = jointIterable.iterator();
         assertTrue(iterator.hasNext());
         assertTrue(rootJoint == iterator.next());
         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            assertTrue(iterator.hasNext());
            assertTrue(rootJointSuccessor.getChildrenJoints().get(childIndex) == iterator.next());
         }
      }
   }

   @Test
   public void testTreeDepth2() throws Exception
   {
      Random random = new Random(324534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyBasics rootBody = new RigidBody("rootBody", ReferenceFrame.getWorldFrame());
         JointBasics rootJoint = MultiBodySystemRandomTools.nextJoint(random, "root", rootBody);
         RigidBody rootJointSuccessor = MultiBodySystemRandomTools.nextRigidBody(random, "rootJointSuccessor", rootJoint);
         int numberOfChildren = 10;
         int numberOfGrandChildrenPerChild = 10;

         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            JointBasics childJoint = MultiBodySystemRandomTools.nextJoint(random, "jointDepth1", rootJointSuccessor);
            RigidBody childBody = MultiBodySystemRandomTools.nextRigidBody(random, "bodyDepth1", childJoint);

            for (int grandChildIndex = 0; grandChildIndex < numberOfGrandChildrenPerChild; grandChildIndex++)
            {
               JointBasics grandChildJoint = MultiBodySystemRandomTools.nextJoint(random, "jointDepth2", childBody);
               MultiBodySystemRandomTools.nextRigidBody(random, "bodyDepth2", grandChildJoint);
            }
         }

         JointIterable<JointReadOnly> jointIterable = new JointIterable<>(JointReadOnly.class, null, IteratorSearchMode.BREADTH_FIRST_SEARCH, rootJoint);
         Iterator<JointReadOnly> iterator = jointIterable.iterator();
         assertTrue(iterator.hasNext());
         assertTrue(rootJoint == iterator.next());

         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            assertTrue(iterator.hasNext());
            assertTrue(rootJointSuccessor.getChildrenJoints().get(childIndex) == iterator.next());
         }
         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            JointBasics childJoint = rootJointSuccessor.getChildrenJoints().get(childIndex);

            for (int grandChildIndex = 0; grandChildIndex < numberOfGrandChildrenPerChild; grandChildIndex++)
            {
               JointBasics grandChildJoint = childJoint.getSuccessor().getChildrenJoints().get(grandChildIndex);
               assertTrue(iterator.hasNext());
               assertTrue(grandChildJoint == iterator.next());
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trying the filtering class for OneDofJointReadOnly, the 1-DoF joints are at the depth 2.
         RigidBodyBasics rootBody = new RigidBody("rootBody", ReferenceFrame.getWorldFrame());
         JointBasics rootJoint = MultiBodySystemRandomTools.nextSixDoFJoint(random, "root", rootBody);
         RigidBody rootJointSuccessor = MultiBodySystemRandomTools.nextRigidBody(random, "rootJointSuccessor", rootJoint);
         int numberOfChildren = 10;
         int numberOfGrandChildrenPerChild = 10;
         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            JointBasics childJoint = MultiBodySystemRandomTools.nextSphericalJoint(random, "jointDepth1", rootJointSuccessor);
            RigidBody childBody = MultiBodySystemRandomTools.nextRigidBody(random, "bodyDepth1", childJoint);

            for (int grandChildIndex = 0; grandChildIndex < numberOfGrandChildrenPerChild; grandChildIndex++)
            {
               JointBasics grandChildJoint = MultiBodySystemRandomTools.nextOneDoFJoint(random, "jointDepth2", childBody);
               MultiBodySystemRandomTools.nextRigidBody(random, "bodyDepth2", grandChildJoint);
            }
         }

         JointIterable<OneDoFJointReadOnly> jointIterable = new JointIterable<>(OneDoFJointReadOnly.class,
                                                                                null,
                                                                                IteratorSearchMode.BREADTH_FIRST_SEARCH,
                                                                                rootJoint);
         Iterator<OneDoFJointReadOnly> iterator = jointIterable.iterator();

         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            JointBasics childJoint = rootJointSuccessor.getChildrenJoints().get(childIndex);

            for (int grandChildIndex = 0; grandChildIndex < numberOfGrandChildrenPerChild; grandChildIndex++)
            {
               JointBasics grandChildJoint = childJoint.getSuccessor().getChildrenJoints().get(grandChildIndex);
               assertTrue(iterator.hasNext());
               JointReadOnly actual = iterator.next();
               assertTrue(grandChildJoint == actual,
                          "child: " + childIndex + ", grand-child: " + grandChildIndex + ", expected: " + grandChildJoint.getName() + ", actual: "
                                                     + actual.getName());
            }
         }
      }
   }

   @Test
   public void testRandomTree()
   {
      Random random = new Random(2342);

      for (int i = 0; i < ITERATIONS; i++)
      {
         IteratorSearchMode mode = EuclidCoreRandomTools.nextElementIn(random, IteratorSearchMode.values());
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, 50);
         RigidBodyBasics root = joints.get(0).getPredecessor();

         List<JointReadOnly> expectedList = switch (mode)
         {
            case DEPTH_FIRST_SEARCH -> collectDFSJoints(root);
            case BREADTH_FIRST_SEARCH -> collectBFSJoints(root);
            default -> throw new IllegalArgumentException("Unexpected value: " + mode);
         };

         JointIterable<JointReadOnly> jointIterable = new JointIterable<>(JointReadOnly.class, null, mode, root);
         List<JointReadOnly> actualList = jointIterable.toStream().toList();

         try
         {
            assertEquals(expectedList, actualList);
            if (mode == IteratorSearchMode.BREADTH_FIRST_SEARCH)
            {
               for (int j = 1; j < actualList.size(); j++)
               {
                  assertTrue(MultiBodySystemTools.computeDistanceToRoot(actualList.get(j - 1).getPredecessor())
                             <= MultiBodySystemTools.computeDistanceToRoot(actualList.get(j).getPredecessor()));
               }
            }
         }
         catch (Throwable e)
         {
            System.out.println("Search mode: " + mode);
            int maxNameLength = expectedList.stream().mapToInt(j -> j.getName().length()).max().getAsInt();
            List<String> expectedNames = expectedList.stream().map(j -> padRightToLength(j.getName(), maxNameLength)).collect(Collectors.toList());
            List<String> actualNames = actualList.stream().map(j -> padRightToLength(j.getName(), maxNameLength)).collect(Collectors.toList());

            for (int j = 0; j < Math.max(expectedNames.size(), actualList.size()); j++)
            {
               if (j < expectedNames.size())
                  System.out.printf("%s(%d)", expectedNames.get(j), MultiBodySystemTools.computeDistanceToRoot(expectedList.get(j).getPredecessor()));
               else
                  System.out.print("\t");

               System.out.print("\t");

               if (j < actualList.size())
                  System.out.printf("%s(%d)", actualNames.get(j), MultiBodySystemTools.computeDistanceToRoot(actualList.get(j).getPredecessor()));
               else
                  System.out.print("\t");
               System.out.println();
            }
            throw e;
         }
      }
   }

   static String padRightToLength(String input, int desiredLength)
   {
      return String.format("%" + (-desiredLength) + "s", input).replace(' ', '-');
   }

   static List<JointReadOnly> collectDFSJoints(RigidBodyReadOnly root)
   {
      return collectDFSJoints(root, new ArrayList<>());
   }

   static List<JointReadOnly> collectDFSJoints(RigidBodyReadOnly root, List<JointReadOnly> resultToPack)
   {
      for (JointReadOnly joint : root.getChildrenJoints())
      {
         resultToPack.add(joint);
         collectDFSJoints(joint.getSuccessor(), resultToPack);
      }
      return resultToPack;
   }

   static List<JointReadOnly> collectBFSJoints(RigidBodyReadOnly root)
   {
      List<JointReadOnly> result = new ArrayList<>();
      Queue<RigidBodyReadOnly> currentBodies = new ArrayDeque<>();
      currentBodies.add(root);

      while (!currentBodies.isEmpty())
      {
         RigidBodyReadOnly body = currentBodies.poll();
         result.addAll(body.getChildrenJoints());
         for (JointReadOnly child : body.getChildrenJoints())
         {
            currentBodies.add(child.getSuccessor());
         }
      }
      return result;
   }
}
