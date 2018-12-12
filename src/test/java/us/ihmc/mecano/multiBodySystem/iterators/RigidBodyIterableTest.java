package us.ihmc.mecano.multiBodySystem.iterators;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

public class RigidBodyIterableTest
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
         RigidBodyReadOnly root = joints.get(0).getPredecessor();
         RigidBodyIterable<RigidBodyReadOnly> bodyIterable = new RigidBodyIterable<>(RigidBodyReadOnly.class, null, root);

         for (int j = 0; j < 2; j++) // Doing 2 calls to RigidBodyIterable.iterator() to make sure the second time the iterator is brand new.
         {
            Iterator<RigidBodyReadOnly> iterator = bodyIterable.iterator();

            assertTrue(iterator.hasNext());
            assertTrue(root == iterator.next());

            for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
            {
               assertTrue(iterator.hasNext());
               assertTrue(joints.get(jointIndex).getSuccessor() == iterator.next());
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing the filtering class with CustomRigidBodyType
         int numberOfJoints = random.nextInt(50) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         if (random.nextBoolean())
            rootBody = new CustomRigidBodyType(rootBody);

         List<RigidBodyBasics> bodies = new ArrayList<>();
         bodies.add(rootBody);

         for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
         {
            RigidBodyBasics successor = joints.get(jointIndex).getSuccessor();
            if (random.nextBoolean())
               successor = new CustomRigidBodyType(successor);
            bodies.add(successor);
         }

         RigidBodyIterable<CustomRigidBodyType> bodyIterable = new RigidBodyIterable<>(CustomRigidBodyType.class, null, rootBody);

         for (int j = 0; j < 2; j++) // Doing 2 calls to RigidBodyIterable.iterator() to make sure the second time the iterator is brand new.
         {
            Iterator<CustomRigidBodyType> iterator = bodyIterable.iterator();

            for (int bodyIndex = 0; bodyIndex < bodies.size(); bodyIndex++)
            {
               if (bodies.get(bodyIndex) instanceof CustomRigidBodyType)
               {
                  assertTrue(iterator.hasNext());
                  assertTrue(bodies.get(bodyIndex) == iterator.next());
               }
            }
         }
      }
   }

   @Test
   public void testTreeDepth1() throws Exception
   {
      Random random = new Random(324534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyBasics rootBody = new RigidBody("rootBody", ReferenceFrame.getWorldFrame());
         JointBasics rootJoint = MultiBodySystemRandomTools.nextJoint(random, "root", rootBody);
         RigidBody rootJointSuccessor = MultiBodySystemRandomTools.nextRigidBody(random, "rootJointSuccessor", rootJoint);
         int numberOfChildren = 10;

         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            JointBasics childJoint = MultiBodySystemRandomTools.nextJoint(random, "jointDepth1", rootJointSuccessor);
            MultiBodySystemRandomTools.nextRigidBody(random, "bodyDepth1", childJoint);
         }

         RigidBodyIterable<RigidBodyReadOnly> bodyIterable = new RigidBodyIterable<>(RigidBodyReadOnly.class, null, rootBody);
         Iterator<RigidBodyReadOnly> iterator = bodyIterable.iterator();
         assertTrue(iterator.hasNext());
         assertTrue(rootBody == iterator.next());
         assertTrue(iterator.hasNext());
         assertTrue(rootJointSuccessor == iterator.next());

         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            assertTrue(iterator.hasNext());
            assertTrue(rootJointSuccessor.getChildrenJoints().get(childIndex).getSuccessor() == iterator.next());
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

         RigidBodyIterable<RigidBodyReadOnly> bodyIterable = new RigidBodyIterable<>(RigidBodyReadOnly.class, null, rootBody);
         Iterator<RigidBodyReadOnly> iterator = bodyIterable.iterator();
         assertTrue(iterator.hasNext());
         assertTrue(rootBody == iterator.next());
         assertTrue(iterator.hasNext());
         assertTrue(rootJointSuccessor == iterator.next());

         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            assertTrue(iterator.hasNext());
            assertTrue(rootJointSuccessor.getChildrenJoints().get(childIndex).getSuccessor() == iterator.next());
         }

         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            JointBasics childJoint = rootJointSuccessor.getChildrenJoints().get(childIndex);

            for (int grandChildIndex = 0; grandChildIndex < numberOfGrandChildrenPerChild; grandChildIndex++)
            {
               RigidBodyBasics grandChildBody = childJoint.getSuccessor().getChildrenJoints().get(grandChildIndex).getSuccessor();
               assertTrue(iterator.hasNext());
               assertTrue(grandChildBody == iterator.next());
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Trying the filtering class for CustomRigidBodyType, the 1-DoF joints are at the depth 2.
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
               new CustomRigidBodyType(MultiBodySystemRandomTools.nextRigidBody(random, "bodyDepth2", grandChildJoint));
            }
         }

         RigidBodyIterable<CustomRigidBodyType> bodyIterable = new RigidBodyIterable<>(CustomRigidBodyType.class, null, rootBody);
         Iterator<CustomRigidBodyType> iterator = bodyIterable.iterator();

         for (int childIndex = 0; childIndex < numberOfChildren; childIndex++)
         {
            JointBasics childJoint = rootJointSuccessor.getChildrenJoints().get(childIndex);

            for (int grandChildIndex = 0; grandChildIndex < numberOfGrandChildrenPerChild; grandChildIndex++)
            {
               RigidBodyBasics grandChildBody = childJoint.getSuccessor().getChildrenJoints().get(grandChildIndex).getSuccessor();
               assertTrue(iterator.hasNext());
               CustomRigidBodyType actual = iterator.next();
               assertTrue(grandChildBody == actual);
            }
         }
      }
   }

   private static class CustomRigidBodyType implements RigidBodyBasics
   {
      private final RigidBodyBasics rigidBody;

      public CustomRigidBodyType(RigidBodyBasics rigidBody)
      {
         this.rigidBody = rigidBody;
         if (!rigidBody.isRootBody())
            rigidBody.getParentJoint().setSuccessor(this);
      }

      @Override
      public SpatialInertiaBasics getInertia()
      {
         return rigidBody.getInertia();
      }

      @Override
      public MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }

      @Override
      public JointBasics getParentJoint()
      {
         return rigidBody.getParentJoint();
      }

      @Override
      public void addChildJoint(JointBasics joint)
      {
         rigidBody.addChildJoint(joint);
      }

      @Override
      public List<? extends JointBasics> getChildrenJoints()
      {
         return rigidBody.getChildrenJoints();
      }

      @Override
      public String toString()
      {
         return rigidBody.toString();
      }

      @Override
      public String getName()
      {
         return rigidBody.getName();
      }

      @Override
      public String getNameId()
      {
         return rigidBody.getNameId();
      }
   }
}
