package us.ihmc.mecano.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.FixedJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

public class MultiBodySystemFactoriesTest
{
   private static final int NUMBER_OF_ITERATIONS = 100;
   private static final double EPSILON = 1.0e-14;

   @Test
   public void testCloneKinematicChain() throws Exception
   {
      Random random = new Random(34636);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         List<? extends JointBasics> supportJoints = MultiBodySystemRandomTools.nextJointChain(random, 5);
         RigidBodyBasics originalStart = supportJoints.get(random.nextInt(supportJoints.size())).getSuccessor();
         List<? extends JointBasics> originalJoints = MultiBodySystemRandomTools.nextJointChain(random, "original", originalStart, 5);
         { // Assert name uniqueness, so we do not have worry in the next assertions for odd edge cases.
            Set<String> jointNames = originalJoints.stream().map(JointReadOnly::getName).collect(Collectors.toSet());
            assertEquals(originalJoints.size(), jointNames.size());
         }
         List<RigidBodyReadOnly> originalSuccessors = originalJoints.stream().map(JointBasics::getSuccessor).collect(Collectors.toList());
         { // Assert name uniqueness, so we do not have worry in the next assertions for odd edge cases.
            Set<String> bodyNames = originalSuccessors.stream().map(RigidBodyReadOnly::getName).collect(Collectors.toSet());
            assertEquals(originalSuccessors.size(), bodyNames.size());
         }
         String cloneSuffix = "Test";
         List<JointBasics> cloneJoints = Arrays.asList(MultiBodySystemFactories.cloneKinematicChain(originalJoints.toArray(new JointBasics[originalJoints.size()]),
                                                                                                    cloneSuffix));
         List<RigidBodyBasics> cloneSuccessors = cloneJoints.stream().map(JointBasics::getSuccessor).collect(Collectors.toList());
         RigidBodyBasics cloneStart = MultiBodySystemTools.getRootBody(cloneSuccessors.get(0));
         { // Perform assertions on the cloneStart
           // Assert that the clone is not an appendage attach to the original multi-body system.
            assertTrue(MultiBodySystemTools.getRootBody(cloneStart) != MultiBodySystemTools.getRootBody(originalStart));
            // Assert that it is the root body
            assertTrue(cloneStart.isRootBody());
            assertNull(cloneStart.getInertia());
            assertEquals(originalStart.getName() + cloneSuffix, cloneStart.getName());
            assertTrue(originalStart.getBodyFixedFrame().getParent() == cloneStart.getBodyFixedFrame().getParent());
            EuclidCoreTestTools.assertRigidBodyTransformEquals(new RigidBodyTransform(), cloneStart.getBodyFixedFrame().getTransformToParent(), EPSILON);
         }
         assertEquals(originalJoints.size(), cloneJoints.size());
         for (int jointIndex = 0; jointIndex < originalJoints.size(); jointIndex++)
         { // Test the joint properties
            JointBasics originalJoint = originalJoints.get(jointIndex);
            JointBasics cloneJoint = cloneJoints.get(jointIndex);
            assertJointPropertiesEqual(originalJoint, cloneJoint, cloneSuffix);

            // Test successor is the correct one
            String expectedCloneSuccessorName = originalJoint.getSuccessor().getName() + cloneSuffix;
            assertEquals(expectedCloneSuccessorName, cloneJoint.getSuccessor().getName());
         }
         assertEquals(originalSuccessors.size(), cloneSuccessors.size());
         for (int bodyIndex = 0; bodyIndex < originalSuccessors.size(); bodyIndex++)
         { // Test rigid-body properties
            RigidBodyReadOnly originalBody = originalSuccessors.get(bodyIndex);
            RigidBodyReadOnly cloneBody = cloneSuccessors.get(bodyIndex);
            assertRigidBodyPropertiesEqual(originalBody, cloneBody, cloneSuffix);

            if (!originalBody.isRootBody())
            {
               // Test the parent joint is the correct one
               assertEquals(originalBody.getParentJoint().getName() + cloneSuffix, cloneBody.getParentJoint().getName());
            }

            // Same test for the children joints
            List<? extends JointReadOnly> originalChildrenJoints = originalBody.getChildrenJoints();
            List<? extends JointReadOnly> cloneChildrenJoints = cloneBody.getChildrenJoints();
            assertEquals(originalChildrenJoints.size(), cloneChildrenJoints.size());

            for (int childIndex = 0; childIndex < originalChildrenJoints.size(); childIndex++)
            {
               JointReadOnly originalChildJoint = originalChildrenJoints.get(childIndex);
               JointReadOnly cloneChildJoint = cloneChildrenJoints.get(childIndex);
               assertEquals(originalChildJoint.getName() + cloneSuffix, cloneChildJoint.getName());
            }
         }
         // Now we verify that the frames of the clone chain follows the ones of the original
         MultiBodySystemTools.getRootBody(supportJoints.get(0).getPredecessor()).updateFramesRecursively();
         originalStart.updateFramesRecursively();
         cloneStart.updateFramesRecursively();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalStart.getParentJoint().getFrameAfterJoint().getTransformToRoot(),
                                                            cloneStart.getBodyFixedFrame().getTransformToRoot(),
                                                            EPSILON);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalJoints.get(0).getFrameBeforeJoint().getTransformToRoot(),
                                                            cloneJoints.get(0).getFrameBeforeJoint().getTransformToRoot(),
                                                            EPSILON);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, supportJoints);
         MultiBodySystemTools.getRootBody(supportJoints.get(0).getPredecessor()).updateFramesRecursively();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalStart.getParentJoint().getFrameAfterJoint().getTransformToRoot(),
                                                            cloneStart.getBodyFixedFrame().getTransformToRoot(),
                                                            EPSILON);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalJoints.get(0).getFrameBeforeJoint().getTransformToRoot(),
                                                            cloneJoints.get(0).getFrameBeforeJoint().getTransformToRoot(),
                                                            EPSILON);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, originalJoints);
         MultiBodySystemTools.copyJointsState(originalJoints, cloneJoints, JointStateType.CONFIGURATION);
         originalStart.updateFramesRecursively();
         cloneStart.updateFramesRecursively();
         for (int jointIndex = 0; jointIndex < originalJoints.size(); jointIndex++)
         {
            ReferenceFrame originalFrameAfterJoint = originalJoints.get(jointIndex).getFrameAfterJoint();
            ReferenceFrame cloneFrameAfterJoint = cloneJoints.get(jointIndex).getFrameAfterJoint();
            EuclidCoreTestTools.assertRigidBodyTransformEquals(originalFrameAfterJoint.getTransformToParent(),
                                                               cloneFrameAfterJoint.getTransformToParent(),
                                                               EPSILON);
         }
         RigidBodyReadOnly originalLeaf = originalJoints.get(originalJoints.size() - 1).getSuccessor();
         RigidBodyReadOnly cloneLeaf = cloneJoints.get(cloneJoints.size() - 1).getSuccessor();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalLeaf.getBodyFixedFrame().getTransformToRoot(),
                                                            cloneLeaf.getBodyFixedFrame().getTransformToRoot(),
                                                            EPSILON);
      }
   }

   @Test
   public void testCloneSubtree() throws Exception
   {
      Random random = new Random(34636);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         List<? extends JointBasics> supportJoints = MultiBodySystemRandomTools.nextJointChain(random, 20);
         RigidBodyBasics originalStart = supportJoints.get(random.nextInt(supportJoints.size())).getSuccessor();
         List<? extends JointBasics> originalJoints = MultiBodySystemRandomTools.nextJointTree(random, "original", originalStart, 20);
         { // Assert name uniqueness, so we do not have worry in the next assertions for odd edge cases.
            Set<String> jointNames = originalJoints.stream().map(JointReadOnly::getName).collect(Collectors.toSet());
            assertEquals(originalJoints.size(), jointNames.size());
         }
         List<RigidBodyReadOnly> originalSuccessors = originalJoints.stream().map(JointBasics::getSuccessor).collect(Collectors.toList());
         { // Assert name uniqueness, so we do not have worry in the next assertions for odd edge cases.
            Set<String> bodyNames = originalSuccessors.stream().map(RigidBodyReadOnly::getName).collect(Collectors.toSet());
            assertEquals(originalSuccessors.size(), bodyNames.size());
         }

         String cloneSuffix = "Test";
         RigidBodyBasics cloneStart = MultiBodySystemFactories.cloneSubtree(originalStart, cloneSuffix);
         List<JointBasics> cloneJoints = SubtreeStreams.fromChildren(cloneStart).collect(Collectors.toList());
         List<RigidBodyBasics> cloneSuccessors = cloneJoints.stream().map(JointBasics::getSuccessor).collect(Collectors.toList());

         { // Perform assertions on the cloneStart
           // Assert that the clone is not an appendage attach to the original multi-body system.
            assertTrue(MultiBodySystemTools.getRootBody(cloneStart) != MultiBodySystemTools.getRootBody(originalStart));
            // Assert that it is the root body
            assertTrue(cloneStart.isRootBody());
            assertNull(cloneStart.getInertia());
            assertEquals(originalStart.getName() + cloneSuffix, cloneStart.getName());
            assertTrue(originalStart.getBodyFixedFrame().getParent() == cloneStart.getBodyFixedFrame().getParent());
            EuclidCoreTestTools.assertRigidBodyTransformEquals(new RigidBodyTransform(), cloneStart.getBodyFixedFrame().getTransformToParent(), EPSILON);
         }

         assertEquals(originalJoints.size(), cloneJoints.size());

         for (int jointIndex = 0; jointIndex < originalJoints.size(); jointIndex++)
         { // Test the joint properties
            JointBasics originalJoint = originalJoints.get(jointIndex);
            JointBasics cloneJoint = cloneJoints.get(jointIndex);
            assertJointPropertiesEqual(originalJoint, cloneJoint, cloneSuffix);

            // Test successor is the correct one
            String expectedCloneSuccessorName = originalJoint.getSuccessor().getName() + cloneSuffix;
            assertEquals(expectedCloneSuccessorName, cloneJoint.getSuccessor().getName());
         }

         assertEquals(originalSuccessors.size(), cloneSuccessors.size());

         for (int bodyIndex = 0; bodyIndex < originalSuccessors.size(); bodyIndex++)
         { // Test rigid-body properties
            RigidBodyReadOnly originalBody = originalSuccessors.get(bodyIndex);
            RigidBodyReadOnly cloneBody = cloneSuccessors.get(bodyIndex);
            assertRigidBodyPropertiesEqual(originalBody, cloneBody, cloneSuffix);

            if (!originalBody.isRootBody())
            {
               // Test the parent joint is the correct one
               assertEquals(originalBody.getParentJoint().getName() + cloneSuffix, cloneBody.getParentJoint().getName());
            }

            // Same test for the children joints
            List<? extends JointReadOnly> originalChildrenJoints = originalBody.getChildrenJoints();
            List<? extends JointReadOnly> cloneChildrenJoints = cloneBody.getChildrenJoints();
            assertEquals(originalChildrenJoints.size(), cloneChildrenJoints.size());

            for (int childIndex = 0; childIndex < originalChildrenJoints.size(); childIndex++)
            {
               JointReadOnly originalChildJoint = originalChildrenJoints.get(childIndex);
               JointReadOnly cloneChildJoint = cloneChildrenJoints.get(childIndex);
               assertEquals(originalChildJoint.getName() + cloneSuffix, cloneChildJoint.getName());
            }
         }

         // Now we verify that the frames of the clone chain follows the ones of the original
         MultiBodySystemTools.getRootBody(supportJoints.get(0).getPredecessor()).updateFramesRecursively();
         originalStart.updateFramesRecursively();
         cloneStart.updateFramesRecursively();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalStart.getParentJoint().getFrameAfterJoint().getTransformToRoot(),
                                                            cloneStart.getBodyFixedFrame().getTransformToRoot(),
                                                            EPSILON);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalJoints.get(0).getFrameBeforeJoint().getTransformToRoot(),
                                                            cloneJoints.get(0).getFrameBeforeJoint().getTransformToRoot(),
                                                            EPSILON);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, supportJoints);
         MultiBodySystemTools.getRootBody(supportJoints.get(0).getPredecessor()).updateFramesRecursively();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalStart.getParentJoint().getFrameAfterJoint().getTransformToRoot(),
                                                            cloneStart.getBodyFixedFrame().getTransformToRoot(),
                                                            EPSILON);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalJoints.get(0).getFrameBeforeJoint().getTransformToRoot(),
                                                            cloneJoints.get(0).getFrameBeforeJoint().getTransformToRoot(),
                                                            EPSILON);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, originalJoints);
         MultiBodySystemTools.copyJointsState(originalJoints, cloneJoints, JointStateType.CONFIGURATION);
         originalStart.updateFramesRecursively();
         cloneStart.updateFramesRecursively();

         for (int jointIndex = 0; jointIndex < originalJoints.size(); jointIndex++)
         {
            ReferenceFrame originalFrameAfterJoint = originalJoints.get(jointIndex).getFrameAfterJoint();
            ReferenceFrame cloneFrameAfterJoint = cloneJoints.get(jointIndex).getFrameAfterJoint();
            EuclidCoreTestTools.assertRigidBodyTransformEquals(originalFrameAfterJoint.getTransformToParent(),
                                                               cloneFrameAfterJoint.getTransformToParent(),
                                                               EPSILON);
         }

         RigidBodyReadOnly originalLeaf = originalJoints.get(originalJoints.size() - 1).getSuccessor();
         RigidBodyReadOnly cloneLeaf = cloneJoints.get(cloneJoints.size() - 1).getSuccessor();
         EuclidCoreTestTools.assertRigidBodyTransformEquals(originalLeaf.getBodyFixedFrame().getTransformToRoot(),
                                                            cloneLeaf.getBodyFixedFrame().getTransformToRoot(),
                                                            EPSILON);
      }
   }

   @Test
   public void testCloneMultiBodySystem() throws Exception
   {
      Random random = new Random(346);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         List<JointBasics> originalJoints = MultiBodySystemRandomTools.nextJointTree(random, 20);
         { // Assert name uniqueness, so we do not have worry in the next assertions for odd edge cases.
            Set<String> jointNames = originalJoints.stream().map(JointReadOnly::getName).collect(Collectors.toSet());
            assertEquals(originalJoints.size(), jointNames.size());
         }
         RigidBodyReadOnly originalRootBody = MultiBodySystemTools.getRootBody(originalJoints.get(0).getPredecessor());
         { // Assert name uniqueness, so we do not have worry in the next assertions for odd edge cases.
            Set<String> bodyNames = originalRootBody.subtreeStream().map(RigidBodyReadOnly::getName).collect(Collectors.toSet());
            assertEquals(originalRootBody.subtreeList().size(), bodyNames.size());
         }
         String cloneSuffix = "Test";
         RigidBodyBasics cloneRootBody = MultiBodySystemFactories.cloneMultiBodySystem(originalRootBody, ReferenceFrame.getWorldFrame(), cloneSuffix);
         List<JointBasics> cloneJoints = SubtreeStreams.from(JointBasics.class, cloneRootBody.getChildrenJoints()).collect(Collectors.toList());
         assertEquals(originalJoints.size(), cloneJoints.size());
         for (int jointIndex = 0; jointIndex < originalJoints.size(); jointIndex++)
         { // Test the joint properties
            JointBasics originalJoint = originalJoints.get(jointIndex);
            JointBasics cloneJoint = cloneJoints.get(jointIndex);
            assertJointPropertiesEqual(originalJoint, cloneJoint, cloneSuffix);

            // Test predecessor is the correct one
            String expectedClonePredecessorName = originalJoint.getPredecessor().getName() + cloneSuffix;
            assertEquals(expectedClonePredecessorName, cloneJoint.getPredecessor().getName());

            // Test successor is the correct one
            String expectedCloneSuccessorName = originalJoint.getSuccessor().getName() + cloneSuffix;
            assertEquals(expectedCloneSuccessorName, cloneJoint.getSuccessor().getName());
         }
         List<? extends RigidBodyReadOnly> originalBodies = originalRootBody.subtreeList();
         List<? extends RigidBodyBasics> cloneBodies = cloneRootBody.subtreeList();
         assertEquals(originalBodies.size(), cloneBodies.size());
         for (int bodyIndex = 0; bodyIndex < originalBodies.size(); bodyIndex++)
         { // Test rigid-body properties
            RigidBodyReadOnly originalBody = originalBodies.get(bodyIndex);
            RigidBodyReadOnly cloneBody = cloneBodies.get(bodyIndex);
            assertRigidBodyPropertiesEqual(originalBody, cloneBody, cloneSuffix);

            if (!originalBody.isRootBody())
            {
               // Test the parent joint is the correct one
               assertEquals(originalBody.getParentJoint().getName() + cloneSuffix, cloneBody.getParentJoint().getName());
            }

            // Same test for the children joints
            List<? extends JointReadOnly> originalChildrenJoints = originalBody.getChildrenJoints();
            List<? extends JointReadOnly> cloneChildrenJoints = cloneBody.getChildrenJoints();
            assertEquals(originalChildrenJoints.size(), cloneChildrenJoints.size());

            for (int childIndex = 0; childIndex < originalChildrenJoints.size(); childIndex++)
            {
               JointReadOnly originalChildJoint = originalChildrenJoints.get(childIndex);
               JointReadOnly cloneChildJoint = cloneChildrenJoints.get(childIndex);
               assertEquals(originalChildJoint.getName() + cloneSuffix, cloneChildJoint.getName());
            }
         }
      }
   }

   public void assertRigidBodyPropertiesEqual(RigidBodyReadOnly originalBody, RigidBodyReadOnly cloneBody, String cloneSuffix)
   {
      // Test clone name
      assertEquals(originalBody.getName() + cloneSuffix, cloneBody.getName());
      assertTrue(originalBody.isRootBody() == cloneBody.isRootBody());
      // This test the inertia pose
      MovingReferenceFrame originalBodyFixedFrame = originalBody.getBodyFixedFrame();
      MovingReferenceFrame cloneBodyFixedFrame = cloneBody.getBodyFixedFrame();
      EuclidCoreTestTools.assertRigidBodyTransformEquals(originalBodyFixedFrame.getTransformToParent(), cloneBodyFixedFrame.getTransformToParent(), EPSILON);

      if (originalBody.isRootBody())
      {
         // Test that is has no inertia
         assertNull(cloneBody.getInertia());
         // Test the frame name
         assertEquals(originalBodyFixedFrame.getName().replaceAll("Frame", "") + cloneSuffix + "Frame", cloneBodyFixedFrame.getName());
         // Tests that the two roots are built from the same frame
         assertTrue(originalBodyFixedFrame.getParent() == cloneBodyFixedFrame.getParent());
      }
      else
      {
         // Test frame name. Inconsistency with the root body.
         assertEquals(originalBodyFixedFrame.getName().replaceAll("CoM", "") + cloneSuffix + "CoM", cloneBodyFixedFrame.getName());

         // Test the inertia properties
         SpatialInertiaReadOnly originalInertia = originalBody.getInertia();
         SpatialInertiaReadOnly cloneInertia = cloneBody.getInertia();

         assertEquals(originalInertia.getMass(), cloneInertia.getMass(), EPSILON);
         EuclidCoreTestTools.assertMatrix3DEquals(originalInertia.getMomentOfInertia(), cloneInertia.getMomentOfInertia(), EPSILON);
      }
   }

   public void assertJointPropertiesEqual(JointReadOnly originalJoint, JointReadOnly cloneJoint, String cloneSuffix)
   {
      // Test clone name
      assertEquals(originalJoint.getName() + cloneSuffix, cloneJoint.getName());

      ReferenceFrame frameAfterOriginalJoint = originalJoint.getFrameAfterJoint();
      ReferenceFrame frameAfterCloneJoint = cloneJoint.getFrameAfterJoint();
      if (originalJoint instanceof FixedJoint && cloneJoint instanceof FixedJoint)
      {
         assertTrue(cloneJoint.getFrameAfterJoint() == cloneJoint.getFrameBeforeJoint());
      }
      else
      {
         // Test the frame name
         assertEquals(frameAfterOriginalJoint.getName() + cloneSuffix, frameAfterCloneJoint.getName());
      }

      ReferenceFrame frameBeforeOriginalJoint = originalJoint.getFrameBeforeJoint();
      ReferenceFrame frameBeforeCloneJoint = cloneJoint.getFrameBeforeJoint();
      // Test the frame name
      if (originalJoint.getPredecessor().isRootBody() || (originalJoint instanceof FixedJoint && cloneJoint instanceof FixedJoint))
         assertEquals(frameBeforeOriginalJoint.getName().replaceAll("Frame", cloneSuffix + "Frame"), frameBeforeCloneJoint.getName());
      else
         assertEquals(frameBeforeOriginalJoint.getName() + cloneSuffix, frameBeforeCloneJoint.getName());
      // Test clone joint offset
      EuclidCoreTestTools.assertRigidBodyTransformEquals(frameBeforeOriginalJoint.getTransformToParent(),
                                                         frameBeforeCloneJoint.getTransformToParent(),
                                                         EPSILON);
      // Test clone type
      assertEquals(originalJoint.getClass(), cloneJoint.getClass());

      if (originalJoint instanceof OneDoFJointReadOnly)
      { // Test joint axis for 1-DoF joints only
         OneDoFJointReadOnly originalOneDoFJoint = (OneDoFJointReadOnly) originalJoint;
         OneDoFJointReadOnly cloneOneDoFJoint = (OneDoFJointReadOnly) cloneJoint;
         EuclidCoreTestTools.assertTuple3DEquals(originalOneDoFJoint.getJointAxis(), cloneOneDoFJoint.getJointAxis(), EPSILON);
      }
   }
}
