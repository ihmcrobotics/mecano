package us.ihmc.mecano.multiBodySystem.iterators;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Iterator;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

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
         JointIterable<JointReadOnly> jointIterable = new JointIterable<>(JointReadOnly.class, null, joints.get(0));
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
   }

   @Test
   public void testTreeDepth1() throws Exception
   {
      Random random = new Random(324534);

      RigidBodyBasics rootBody = new RigidBody("rootBody", ReferenceFrame.getWorldFrame());
      JointBasics rootJoint = MultiBodySystemRandomTools.nextJoint(random, "root", rootBody);
      RigidBody rootJointSuccessor = MultiBodySystemRandomTools.nextRigidBody(random, "rootJointSuccessor", rootJoint);

      int numberOfChildren = 10;

      for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
      {
         JointBasics childJoint = MultiBodySystemRandomTools.nextJoint(random, "jointDepth1", rootJointSuccessor);
         MultiBodySystemRandomTools.nextRigidBody(random, "bodyDepth1", childJoint);
      }

      JointIterable<JointReadOnly> jointIterable = new JointIterable<>(JointReadOnly.class, null, rootJoint);
      Iterator<JointReadOnly> iterator = jointIterable.iterator();

      assertTrue(iterator.hasNext());
      assertTrue(rootJoint == iterator.next());

      for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
      {
         assertTrue(iterator.hasNext());
         assertTrue(rootJointSuccessor.getChildrenJoints().get(childIndex) == iterator.next());
      }
   }

   @Test
   public void testTreeDepth2() throws Exception
   {
      Random random = new Random(324534);

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

      JointIterable<JointReadOnly> jointIterable = new JointIterable<>(JointReadOnly.class, null, rootJoint);
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
}
