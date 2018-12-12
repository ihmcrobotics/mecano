package us.ihmc.mecano.frames;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;

public class MovingReferenceFrameTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 5000;
   private static final double EPSILON = 1.0e-12;

   private static RandomlyChangingFrame[] nextRandomlyChangingFrameTree(Random random, int numberOfReferenceFrames)
   {
      return nextRandomlyChangingFrameTree("randomFrame", random, numberOfReferenceFrames);
   }

   private static RandomlyChangingFrame[] nextRandomlyChangingFrameTree(String frameNamePrefix, Random random, int numberOfReferenceFrames)
   {
      return nextRandomlyChangingFrameTree(frameNamePrefix, random, worldFrame, numberOfReferenceFrames);
   }

   private static RandomlyChangingFrame[] nextRandomlyChangingFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame,
                                                                        int numberOfReferenceFrames)
   {
      RandomlyChangingFrame[] referenceFrames = new RandomlyChangingFrame[numberOfReferenceFrames];
      ReferenceFrame[] referenceFramesWithRoot = new ReferenceFrame[numberOfReferenceFrames + 1];
      referenceFramesWithRoot[0] = rootFrame;

      for (int i = 0; i < numberOfReferenceFrames; i++)
      {
         int parentFrameIndex = random.nextInt(i + 1);
         ReferenceFrame parentFrame = referenceFramesWithRoot[parentFrameIndex];
         RandomlyChangingFrame randomlyChangingFrame = new RandomlyChangingFrame(frameNamePrefix + i, parentFrame, random);
         referenceFrames[i] = randomlyChangingFrame;
         referenceFramesWithRoot[i + 1] = randomlyChangingFrame;
      }

      return referenceFrames;
   }

   private static class RandomlyChangingFrame extends MovingReferenceFrame
   {
      private final Random random;
      private final RigidBodyTransform randomTransform = new RigidBodyTransform();
      private final Twist randomTwist = new Twist();

      public RandomlyChangingFrame(String frameName, ReferenceFrame parentFrame, Random random)
      {
         super(frameName, parentFrame);
         this.random = random;
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         randomTransform.set(EuclidCoreRandomTools.nextRigidBodyTransform(random));
         transformToParent.set(randomTransform);
      }

      @Override
      protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
      {
         randomTwist.setIncludingFrame(MecanoRandomTools.nextTwist(random, this, getParent(), this));
         twistRelativeToParentToPack.set(randomTwist);
      }
   }

   @Test
   public void testFixedInParentBug() throws Exception
   {
      Random random = new Random(1231);

      MovingReferenceFrame parentMovingFrame = nextRandomlyChangingFrameTree(random, 1)[0];
      MovingReferenceFrame fixedInParentOne = MovingReferenceFrame.constructFrameFixedInParent("fixed1", parentMovingFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));
      MovingReferenceFrame fixedInParentTwo = MovingReferenceFrame.constructFrameFixedInParent("fixed2", parentMovingFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));

      parentMovingFrame.update();

      Twist actualTwistFrameOne = new Twist(fixedInParentOne.getTwistOfFrame());
      Twist expectedTwistFrameOne = computeTwistRelativeToRootByClimbingTree(fixedInParentOne);

      Twist actualTwistFrameTwo = new Twist(fixedInParentTwo.getTwistOfFrame());
      Twist expectedTwistFrameTwo = computeTwistRelativeToRootByClimbingTree(fixedInParentTwo);

      MecanoTestTools.assertTwistEquals(actualTwistFrameOne, expectedTwistFrameOne, EPSILON);
      MecanoTestTools.assertTwistEquals(actualTwistFrameTwo, expectedTwistFrameTwo, EPSILON);

      parentMovingFrame.update();

      actualTwistFrameOne = new Twist(fixedInParentOne.getTwistOfFrame());
      expectedTwistFrameOne = computeTwistRelativeToRootByClimbingTree(fixedInParentOne);
      
      actualTwistFrameTwo = new Twist(fixedInParentTwo.getTwistOfFrame());
      expectedTwistFrameTwo = computeTwistRelativeToRootByClimbingTree(fixedInParentTwo);
      
      MecanoTestTools.assertTwistEquals(actualTwistFrameOne, expectedTwistFrameOne, EPSILON);
      MecanoTestTools.assertTwistEquals(actualTwistFrameTwo, expectedTwistFrameTwo, EPSILON);
   }

   @Test
   public void testTypicalExample()
   {
      Random random = new Random(87);

      for (int i = 0; i < ITERATIONS; i++)
      {
         MovingReferenceFrame[] treeFrame = MecanoRandomTools.nextMovingReferenceFrameTree(random);

         MovingReferenceFrame frameA = treeFrame[random.nextInt(treeFrame.length)];
         MovingReferenceFrame frameB = treeFrame[random.nextInt(treeFrame.length)];

         RigidBodyTransform shouldBeIdentity = new RigidBodyTransform(frameB.getTransformToDesiredFrame(frameA));
         shouldBeIdentity.multiply(frameA.getTransformToDesiredFrame(frameB));

         EuclidCoreTestTools.assertRigidBodyTransformEquals(new RigidBodyTransform(), shouldBeIdentity, EPSILON);

         Twist twistARelativeToB = new Twist();
         Twist twistBRelativeToA = new Twist();

         frameA.getTwistRelativeToOther(frameB, twistARelativeToB);
         frameB.getTwistRelativeToOther(frameA, twistBRelativeToA);

         twistBRelativeToA.invert();
         twistBRelativeToA.changeFrame(frameA);

         MecanoTestTools.assertTwistEquals(twistARelativeToB, twistBRelativeToA, EPSILON);
      }
   }

   @Test
   public void testGetTransformAndTwistToParents()
   {
      Random random = new Random(87);

      for (int i = 0; i < ITERATIONS; i++)
      {
         MovingReferenceFrame[] treeFrame = MecanoRandomTools.nextMovingReferenceFrameTree(random);

         MovingReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];

         MovingReferenceFrame parent = frame.getMovingParent();
         if (parent != null)
         {
            RigidBodyTransform transformToParentOne = frame.getTransformToParent();
            RigidBodyTransform transformToParentTwo = frame.getTransformToDesiredFrame(parent);
            EuclidCoreTestTools.assertRigidBodyTransformEquals(transformToParentOne, transformToParentTwo, EPSILON);

            Twist twistRelativeToParentOne = new Twist(frame.getTwistRelativeToParent());
            Twist twistRelativeToParentTwo = new Twist();
            frame.getTwistRelativeToOther(parent, twistRelativeToParentTwo);
            MecanoTestTools.assertTwistEquals(twistRelativeToParentOne, twistRelativeToParentTwo, EPSILON);
         }
      }
   }

   @Test
   public void testGetTransformAndTwistToRoots()
   {
      Random random = new Random(453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         MovingReferenceFrame[] treeFrame = MecanoRandomTools.nextMovingReferenceFrameTree(random);

         MovingReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(worldFrame);
         RigidBodyTransform transformToRootTwo = computeTransformToRootByClimbingTree(frame);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(transformToRootOne, transformToRootTwo, EPSILON);

         Twist twistOne = new Twist();
         frame.getTwistRelativeToOther(worldFrame, twistOne);
         Twist twistTwo = computeTwistRelativeToRootByClimbingTree(frame);
         MecanoTestTools.assertTwistEquals(twistOne, twistTwo, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame anotherRoot = ReferenceFrameTools.constructARootFrame("anotherRoot");

         MovingReferenceFrame[] treeFrame = MecanoRandomTools.nextMovingReferenceFrameTree("blop", random, anotherRoot, 20);

         MovingReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(anotherRoot);
         RigidBodyTransform transformToRootTwo = computeTransformToRootByClimbingTree(frame);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(transformToRootOne, transformToRootTwo, EPSILON);

         Twist twistOne = new Twist();
         frame.getTwistRelativeToOther(anotherRoot, twistOne);
         Twist twistTwo = computeTwistRelativeToRootByClimbingTree(frame);
         MecanoTestTools.assertTwistEquals(twistOne, twistTwo, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         RandomlyChangingFrame[] treeFrame = nextRandomlyChangingFrameTree(random, 100);

         int numberOfRandomUpdates = random.nextInt(treeFrame.length / 2) + 1;
         for (int j = 0; j < numberOfRandomUpdates; j++)
         {
            treeFrame[random.nextInt(treeFrame.length)].update();
         }

         for (RandomlyChangingFrame frame : treeFrame)
         {
            RigidBodyTransform transformToRootOne = frame.getTransformToRoot();
            RigidBodyTransform transformToRootTwo = computeTransformToRootByClimbingTree(frame);
            EuclidCoreTestTools.assertRigidBodyTransformEquals(transformToRootOne, transformToRootTwo, EPSILON);

            Twist twistOne = new Twist(frame.getTwistOfFrame());
            Twist twistTwo = computeTwistRelativeToRootByClimbingTree(frame);
            MecanoTestTools.assertTwistEquals(twistOne, twistTwo, EPSILON);
         }
      }
   }

   private RigidBodyTransform computeTransformToRootByClimbingTree(ReferenceFrame currentFrame)
   {
      if (currentFrame == null || currentFrame.isRootFrame())
         return new RigidBodyTransform();

      RigidBodyTransform transformToRoot = new RigidBodyTransform();
      if (currentFrame.getTransformToParent() != null)
         transformToRoot.set(currentFrame.getTransformToParent());
      transformToRoot.preMultiply(computeTransformToRootByClimbingTree(currentFrame.getParent()));
      return transformToRoot;
   }

   private Twist computeTwistRelativeToRootByClimbingTree(MovingReferenceFrame currentFrame)
   {
      if (currentFrame == null || currentFrame.isAStationaryFrame())
         return null;

      Twist twistRelativeToRoot = new Twist();
      if (currentFrame.getTwistRelativeToParent() != null)
         twistRelativeToRoot.setIncludingFrame(currentFrame.getTwistRelativeToParent());
      else
         twistRelativeToRoot.setToZero(currentFrame, currentFrame.getParent(), currentFrame);
      Twist parentTwist = computeTwistRelativeToRootByClimbingTree(currentFrame.getMovingParent());
      if (parentTwist != null)
      {
         parentTwist.changeFrame(currentFrame);
         twistRelativeToRoot.add(parentTwist);
      }
      else
         twistRelativeToRoot.setBaseFrame(currentFrame.getRootFrame());
      return twistRelativeToRoot;
   }
}
