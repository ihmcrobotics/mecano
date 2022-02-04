package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.mecano.tools.MecanoRandomTools;

public class SpatialImpulseTest extends SpatialImpulseBasicsTest<SpatialImpulse>
{
   @Override
   public SpatialImpulse newEmptySpatialImpulse()
   {
      return new SpatialImpulse();
   }

   @Override
   public SpatialImpulse newRandomSpatialImpulse(Random random)
   {
      return new SpatialImpulse(EuclidFrameRandomTools.nextReferenceFrame(random),
                                EuclidFrameRandomTools.nextReferenceFrame(random),
                                EuclidCoreRandomTools.nextVector3D(random),
                                EuclidCoreRandomTools.nextVector3D(random));
   }

   @Override
   public SpatialImpulse newSpatialImpulse(ReferenceFrame bodyFrame, SpatialVector spatialVector)
   {
      return new SpatialImpulse(bodyFrame, spatialVector);
   }

   @Override
   public SpatialImpulse newCopySpatialImpulse(SpatialImpulseReadOnly other)
   {
      return new SpatialImpulse(other);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-5;
   }

   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructors()
   {
      Random random = new Random(130375);

      //Test create a new spatial impulse with its components set to zero and its reference frames set to null
      SpatialImpulse spatialImpulse = new SpatialImpulse();

      assertEquals(spatialImpulse.getBodyFrame(), null);
      assertEquals(spatialImpulse.getReferenceFrame(), null);
      assertEquals(spatialImpulse.getAngularPart().getReferenceFrame(), null);
      assertEquals(spatialImpulse.getLinearPart().getReferenceFrame(), null);
      assertEquals(spatialImpulse.getAngularPartX(), 0);
      assertEquals(spatialImpulse.getAngularPartY(), 0);
      assertEquals(spatialImpulse.getAngularPartZ(), 0);
      assertEquals(spatialImpulse.getLinearPartX(), 0);
      assertEquals(spatialImpulse.getLinearPartY(), 0);
      assertEquals(spatialImpulse.getLinearPartZ(), 0);

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Test create a spatial impulse with it components set to zero and initializes its reference frames.
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         SpatialImpulse spatialImpulseByReferenceFrames = new SpatialImpulse(randomBodyFrame, randomReferenceFrame);

         assertEquals(spatialImpulseByReferenceFrames.getBodyFrame(), randomBodyFrame);
         assertEquals(spatialImpulseByReferenceFrames.getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFrames.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFrames.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFrames.getAngularPartX(), 0);
         assertEquals(spatialImpulseByReferenceFrames.getAngularPartY(), 0);
         assertEquals(spatialImpulseByReferenceFrames.getAngularPartZ(), 0);
         assertEquals(spatialImpulseByReferenceFrames.getLinearPartX(), 0);
         assertEquals(spatialImpulseByReferenceFrames.getLinearPartY(), 0);
         assertEquals(spatialImpulseByReferenceFrames.getLinearPartZ(), 0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Test create a new spatial impulse and initializes its components and reference frames.
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);

         SpatialImpulse spatialImpulseByReferenceFramesAngularPartLinearPart = new SpatialImpulse(randomBodyFrame,
                                                                                                  randomReferenceFrame,
                                                                                                  randomAngularPart,
                                                                                                  randomLinearPart);

         assertEquals(spatialImpulseByReferenceFramesAngularPartLinearPart.getBodyFrame(), randomBodyFrame);
         assertEquals(spatialImpulseByReferenceFramesAngularPartLinearPart.getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFramesAngularPartLinearPart.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFramesAngularPartLinearPart.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         EuclidCoreTestTools.assertTuple3DEquals(spatialImpulseByReferenceFramesAngularPartLinearPart.getAngularPart(), randomAngularPart, getEpsilon());
         EuclidCoreTestTools.assertTuple3DEquals(spatialImpulseByReferenceFramesAngularPartLinearPart.getLinearPart(), randomLinearPart, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Test create a new spatial impulse from the given reference frames and matrix.
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(6, 1);
         for (int row = 0; row < 6; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         SpatialImpulse spatialImpulseByReferenceFramesDMatrix = new SpatialImpulse(randomBodyFrame, randomReferenceFrame, randomMatrix);

         assertEquals(spatialImpulseByReferenceFramesDMatrix.getBodyFrame(), randomBodyFrame);
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getAngularPartX(), randomMatrix.get(0, 0));
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getAngularPartY(), randomMatrix.get(1, 0));
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getAngularPartZ(), randomMatrix.get(2, 0));
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getLinearPartX(), randomMatrix.get(3, 0));
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getLinearPartY(), randomMatrix.get(4, 0));
         assertEquals(spatialImpulseByReferenceFramesDMatrix.getLinearPartZ(), randomMatrix.get(5, 0));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Test create a new spatial impulse from the given reference frames and array.
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         double[] randomVectorArray = new double[6];
         for (int index = 0; index < 6; index++)
            randomVectorArray[index] = random.nextDouble();

         SpatialImpulse spatialImpulseByReferenceFramesArray = new SpatialImpulse(randomBodyFrame, randomReferenceFrame, randomVectorArray);

         assertEquals(spatialImpulseByReferenceFramesArray.getBodyFrame(), randomBodyFrame);
         assertEquals(spatialImpulseByReferenceFramesArray.getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFramesArray.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFramesArray.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(spatialImpulseByReferenceFramesArray.getAngularPartX(), randomVectorArray[0]);
         assertEquals(spatialImpulseByReferenceFramesArray.getAngularPartY(), randomVectorArray[1]);
         assertEquals(spatialImpulseByReferenceFramesArray.getAngularPartZ(), randomVectorArray[2]);
         assertEquals(spatialImpulseByReferenceFramesArray.getLinearPartX(), randomVectorArray[3]);
         assertEquals(spatialImpulseByReferenceFramesArray.getLinearPartY(), randomVectorArray[4]);
         assertEquals(spatialImpulseByReferenceFramesArray.getLinearPartZ(), randomVectorArray[5]);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Test create a new spatial impulse from the given bodyFrame and spatial vector.
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector randomSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);

         SpatialImpulse spatialImpulseByBodyFrameSpatialVector = new SpatialImpulse(randomBodyFrame, randomSpatialVector);

         assertEquals(spatialImpulseByBodyFrameSpatialVector.getBodyFrame(), randomBodyFrame);
         assertEquals(spatialImpulseByBodyFrameSpatialVector.getReferenceFrame(), randomSpatialVector.getReferenceFrame());
         assertEquals(spatialImpulseByBodyFrameSpatialVector.getAngularPart(), randomSpatialVector.getAngularPart());
         assertEquals(spatialImpulseByBodyFrameSpatialVector.getLinearPart(), randomSpatialVector.getLinearPart());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Test copy constructor
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector randomSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);

         SpatialImpulse spatialImpulseByBodyFrameSpatialVector = new SpatialImpulse(randomBodyFrame, randomSpatialVector);
         SpatialImpulse spatialImpulseByCopy = new SpatialImpulse(spatialImpulseByBodyFrameSpatialVector);

         assertFalse(spatialImpulseByBodyFrameSpatialVector == spatialImpulseByCopy);
         assertEquals(spatialImpulseByBodyFrameSpatialVector, spatialImpulseByCopy);
      }
   }

   @Test
   public void testSet()
   {
      Random random = new Random(21587);

      //Test Set works when Reference Frames are the same
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomOriginalAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomOriginalLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, randomReferenceFrame, randomOriginalAngularPart, randomOriginalLinearPart);

         Vector3D randomExpectedAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomExpectedLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(randomBodyFrame, randomReferenceFrame, randomExpectedAngularPart, randomExpectedLinearPart);

         assertNotEquals(originalSpatialImpulse, expectedSpatialImpulse);

         originalSpatialImpulse.set(expectedSpatialImpulse);

         assertEquals(originalSpatialImpulse, expectedSpatialImpulse);
      }

      //Test Set gives an exception when Reference Frames are the different
      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            ReferenceFrame randomOriginalBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomOriginalReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            Vector3D randomOriginalAngularPart = EuclidCoreRandomTools.nextVector3D(random);
            Vector3D randomOriginalLinearPart = EuclidCoreRandomTools.nextVector3D(random);
            SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomOriginalBodyFrame,
                                                                       randomOriginalReferenceFrame,
                                                                       randomOriginalAngularPart,
                                                                       randomOriginalLinearPart);

            ReferenceFrame randomExpectedBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomExpectedReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            Vector3D randomExpectedAngularPart = EuclidCoreRandomTools.nextVector3D(random);
            Vector3D randomExpectedLinearPart = EuclidCoreRandomTools.nextVector3D(random);
            SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(randomExpectedBodyFrame,
                                                                       randomExpectedReferenceFrame,
                                                                       randomExpectedAngularPart,
                                                                       randomExpectedLinearPart);

            originalSpatialImpulse.set(expectedSpatialImpulse);
         }, "Set should have thrown a ReferenceFrameMatchingException error when reference frame different for the vectors.");
      }
   }

   @Test
   public void testSetBodyFrame()
   {
      Random random = new Random(58741);

      //Test SetBodyFrame only changes the body frame of a Spatial Impulse
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomOriginalAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomOriginalLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, randomReferenceFrame, randomOriginalAngularPart, randomOriginalLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         ReferenceFrame expectedBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         expectedSpatialImpulse.setBodyFrame(expectedBodyFrame);

         assertEquals(expectedSpatialImpulse.getBodyFrame(), expectedBodyFrame);
         assertEquals(expectedSpatialImpulse.getReferenceFrame(), originalSpatialImpulse.getReferenceFrame());
         assertEquals(expectedSpatialImpulse.getAngularPart(), originalSpatialImpulse.getAngularPart());
         assertEquals(expectedSpatialImpulse.getLinearPart(), originalSpatialImpulse.getLinearPart());
      }
   }

   @Test
   public void testSetReferenceFrame()
   {
      Random random = new Random(212080);

      //Test SetReference Frame only changes the reference frame of a Spatial Impulse and the Angular and Linear parts
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomOriginalAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomOriginalLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, randomReferenceFrame, randomOriginalAngularPart, randomOriginalLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         ReferenceFrame expectedReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         expectedSpatialImpulse.setReferenceFrame(expectedReferenceFrame);

         assertEquals(expectedSpatialImpulse.getBodyFrame(), originalSpatialImpulse.getBodyFrame());
         assertEquals(expectedSpatialImpulse.getReferenceFrame(), expectedReferenceFrame);
         assertEquals(expectedSpatialImpulse.getAngularPart().getReferenceFrame(), expectedReferenceFrame);
         assertEquals(expectedSpatialImpulse.getLinearPart().getReferenceFrame(), expectedReferenceFrame);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getAngularPart(), originalSpatialImpulse.getAngularPart(), getEpsilon());
         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getLinearPart(), originalSpatialImpulse.getLinearPart(), getEpsilon());
      }
   }

   @Test
   public void testGets()
   {
      Random random = new Random(1005);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3DBasics randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
         FrameVector3DBasics randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

         SpatialImpulse randomSpatialImpulse = new SpatialImpulse(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart(), randomAngularPart);
         assertEquals(randomSpatialImpulse.getLinearPart(), randomLinearPart);
      }
   }

   @Test
   public void testApplyTransform()
   {
      Random random = new Random(3456L);

      for (int i = 0; i < ITERATIONS; i++)
      { //Test Rotation = 0; Translation = (random,0,0); AngularPart(Motion) = (0,random,0) and LinearPart(Force) = (0,0,random)
         RigidBodyTransform transform = new RigidBodyTransform();
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D sourceAngularPart = new Vector3D(0.0, random.nextDouble(), 0.0);
         Vector3D sourceLinearPart = new Vector3D(0.0, 0.0, random.nextDouble());
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, sourceFrame, sourceAngularPart, sourceLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         transform.getTranslation().set(random.nextDouble(), 0.0, 0.0);
         expectedSpatialImpulse.applyTransform(transform);

         assertEquals(expectedSpatialImpulse.getAngularPartX(), 0.0);
         assertEquals(expectedSpatialImpulse.getAngularPartY(), -transform.getTranslationX() * sourceLinearPart.getZ() + sourceAngularPart.getY());
         assertEquals(expectedSpatialImpulse.getAngularPartZ(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getLinearPart(), originalSpatialImpulse.getLinearPart(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { //Test Rotation = 0; Translation = (0,random,0); AngularPart(Motion) = (0,0,random) and LinearPart(Force) = (random,0,0)
         RigidBodyTransform transform = new RigidBodyTransform();
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D sourceAngularPart = new Vector3D(0.0, 0.0, random.nextDouble());
         Vector3D sourceLinearPart = new Vector3D(random.nextDouble(), 0.0, 0.0);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, sourceFrame, sourceAngularPart, sourceLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         transform.getTranslation().set(0.0, random.nextDouble(), 0.0);
         expectedSpatialImpulse.applyTransform(transform);

         assertEquals(expectedSpatialImpulse.getAngularPartX(), 0.0);
         assertEquals(expectedSpatialImpulse.getAngularPartY(), 0.0);
         assertEquals(expectedSpatialImpulse.getAngularPartZ(), -transform.getTranslationY() * sourceLinearPart.getX() + sourceAngularPart.getZ());
         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getLinearPart(), originalSpatialImpulse.getLinearPart(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { //Test Rotation = 0; Translation = (0,0,random); AngularPart(Motion) = (random,0,0) and LinearPart(Force) = (0,random,0)
         RigidBodyTransform transform = new RigidBodyTransform();
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D sourceAngularPart = new Vector3D(random.nextDouble(), 0.0, 0.0);
         Vector3D sourceLinearPart = new Vector3D(0.0, random.nextDouble(), 0.0);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, sourceFrame, sourceAngularPart, sourceLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         transform.getTranslation().set(0.0, 0.0, random.nextDouble());
         expectedSpatialImpulse.applyTransform(transform);

         assertEquals(expectedSpatialImpulse.getAngularPartX(), -transform.getTranslationZ() * sourceLinearPart.getY() + sourceAngularPart.getX());
         assertEquals(expectedSpatialImpulse.getAngularPartY(), 0.0);
         assertEquals(expectedSpatialImpulse.getAngularPartZ(), 0.0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getLinearPart(), originalSpatialImpulse.getLinearPart(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { //Test Rotation = random; Translation = (0,0,0); AngularPart(Motion) = (random,random,random) and LinearPart(Force) = (random,random,random)
         RigidBodyTransform transform = new RigidBodyTransform();
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D sourceAngularPart = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Vector3D sourceLinearPart = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, sourceFrame, sourceAngularPart, sourceLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         transform.getRotation().set(EuclidCoreRandomTools.nextOrientation3D(random));
         expectedSpatialImpulse.applyTransform(transform);

         sourceAngularPart.applyTransform(transform);
         sourceLinearPart.applyTransform(transform);

         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getAngularPart(), sourceAngularPart, getEpsilon());
         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getLinearPart(), sourceLinearPart, getEpsilon());
      }

   }

   @Test
   public void testChangeFrame()
   {
      Random random = new Random(1005);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D sourceAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         FrameVector3D sourceLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);

         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, sourceFrame, sourceAngularPart, sourceLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);
         SpatialImpulse actualSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         RigidBodyTransform transformToDesiredFrame = expectedSpatialImpulse.getReferenceFrame().getTransformToDesiredFrame(destinationFrame);
         expectedSpatialImpulse.applyTransform(transformToDesiredFrame);
         expectedSpatialImpulse.setReferenceFrame(destinationFrame);

         actualSpatialImpulse.changeFrame(destinationFrame);

         assertEquals(actualSpatialImpulse.getBodyFrame(), originalSpatialImpulse.getBodyFrame());
         assertEquals(actualSpatialImpulse.getReferenceFrame(), destinationFrame);
         assertEquals(actualSpatialImpulse.getAngularPart().getReferenceFrame(), destinationFrame);
         assertEquals(actualSpatialImpulse.getLinearPart().getReferenceFrame(), destinationFrame);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getAngularPart(), actualSpatialImpulse.getAngularPart(), getEpsilon());
         EuclidCoreTestTools.assertTuple3DEquals(expectedSpatialImpulse.getLinearPart(), actualSpatialImpulse.getLinearPart(), getEpsilon());
      }
   }

   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D sourceAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         FrameVector3D sourceLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, sourceFrame, sourceAngularPart, sourceLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         expectedSpatialImpulse.set(originalSpatialImpulse);
         assertEquals(originalSpatialImpulse, expectedSpatialImpulse);

         expectedSpatialImpulse.applyTransform(transform);
         assertNotEquals(expectedSpatialImpulse, originalSpatialImpulse);
         expectedSpatialImpulse.applyInverseTransform(transform);
         assertTrue(expectedSpatialImpulse.epsilonEquals(originalSpatialImpulse, getEpsilon()));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D sourceAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         FrameVector3D sourceLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, sourceFrame, sourceAngularPart, sourceLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         expectedSpatialImpulse.set(originalSpatialImpulse);
         assertEquals(originalSpatialImpulse, expectedSpatialImpulse);

         expectedSpatialImpulse.applyTransform(transform);
         assertNotEquals(expectedSpatialImpulse, originalSpatialImpulse);
         expectedSpatialImpulse.applyInverseTransform(transform);
         assertTrue(expectedSpatialImpulse.epsilonEquals(originalSpatialImpulse, getEpsilon()));
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D sourceAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         FrameVector3D sourceLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(randomBodyFrame, sourceFrame, sourceAngularPart, sourceLinearPart);
         SpatialImpulse expectedSpatialImpulse = new SpatialImpulse(originalSpatialImpulse);

         expectedSpatialImpulse.set(originalSpatialImpulse);
         assertEquals(originalSpatialImpulse, expectedSpatialImpulse);

         expectedSpatialImpulse.applyTransform(transform);
         assertNotEquals(expectedSpatialImpulse, originalSpatialImpulse);
         expectedSpatialImpulse.applyInverseTransform(transform);
         assertTrue(expectedSpatialImpulse.epsilonEquals(originalSpatialImpulse, getEpsilon()));
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(1250L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D originalAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         FrameVector3D originalLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, originalAngularPart, originalLinearPart);

         FrameVector3D changeAngularPart = new FrameVector3D(originalAngularPart);
         FrameVector3D changeLinearPart = new FrameVector3D(originalLinearPart);

         //Angular Part - element above epsilon
         changeAngularPart.setElement(i % 3, originalAngularPart.getElement(i % 3) - 1.01 * getEpsilon());
         SpatialImpulse changedAboveSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, changeAngularPart, originalLinearPart);
         assertFalse(changedAboveSpatialImpulse.epsilonEquals(originalSpatialImpulse, getEpsilon()));

         //Angular Part - element below epsilon
         changeAngularPart.setElement(i % 3, changeAngularPart.getElement(i % 3) + 0.99 * getEpsilon());
         SpatialImpulse changedBelowSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, changeAngularPart, originalLinearPart);
         assertTrue(changedBelowSpatialImpulse.epsilonEquals(originalSpatialImpulse, getEpsilon()));

         //Linear Part - element above epsilon
         changeLinearPart.setElement(i % 3, originalLinearPart.getElement(i % 3) - 1.01 * getEpsilon());
         changedAboveSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, originalAngularPart, changeLinearPart);
         assertFalse(changedAboveSpatialImpulse.epsilonEquals(originalSpatialImpulse, getEpsilon()));

         //Linear Part - element below epsilon
         changeLinearPart.setElement(i % 3, originalLinearPart.getElement(i % 3) + 0.99 * getEpsilon());
         changedBelowSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, originalAngularPart, changeLinearPart);
         assertTrue(changedBelowSpatialImpulse.epsilonEquals(originalSpatialImpulse, getEpsilon()));

         //Source body frame different - angular/linear parts the same
         SpatialImpulse frameDifferentSpatialImpulse = new SpatialImpulse(sourceBodyFrame,
                                                                          EuclidFrameRandomTools.nextReferenceFrame(random),
                                                                          originalAngularPart,
                                                                          originalLinearPart);
         assertFalse(originalSpatialImpulse.epsilonEquals(frameDifferentSpatialImpulse, getEpsilon()));

         //Reference frame different - angular/linear parts the same
         frameDifferentSpatialImpulse = new SpatialImpulse(EuclidFrameRandomTools.nextReferenceFrame(random),
                                                           sourceFrame,
                                                           originalAngularPart,
                                                           originalLinearPart);
         assertFalse(originalSpatialImpulse.epsilonEquals(frameDifferentSpatialImpulse, getEpsilon()));
      }
   }

   @Test
   public void testGeometricallyEquals()
   {
      Random random = new Random(130375);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D originalAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         FrameVector3D originalLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, originalAngularPart, originalLinearPart);

         FrameVector3D changeAngularPart = new FrameVector3D(originalAngularPart);
         FrameVector3D changeLinearPart = new FrameVector3D(originalLinearPart);

         //Angular Part - element above epsilon
         changeAngularPart.setElement(i % 3, originalAngularPart.getElement(i % 3) - 1.01 * getEpsilon());
         SpatialImpulse changedAboveSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, changeAngularPart, originalLinearPart);
         assertFalse(changedAboveSpatialImpulse.geometricallyEquals(originalSpatialImpulse, getEpsilon()));

         //Angular Part - element below epsilon
         changeAngularPart.setElement(i % 3, changeAngularPart.getElement(i % 3) + 0.99 * getEpsilon());
         SpatialImpulse changedBelowSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, changeAngularPart, originalLinearPart);
         assertTrue(changedBelowSpatialImpulse.geometricallyEquals(originalSpatialImpulse, getEpsilon()));

         //Linear Part - element above epsilon
         changeLinearPart.setElement(i % 3, originalLinearPart.getElement(i % 3) - 1.01 * getEpsilon());
         changedAboveSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, originalAngularPart, changeLinearPart);
         assertFalse(changedAboveSpatialImpulse.geometricallyEquals(originalSpatialImpulse, getEpsilon()));

         //Linear Part - element below epsilon
         changeLinearPart.setElement(i % 3, originalLinearPart.getElement(i % 3) + 0.99 * getEpsilon());
         changedBelowSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, originalAngularPart, changeLinearPart);
         assertTrue(changedBelowSpatialImpulse.geometricallyEquals(originalSpatialImpulse, getEpsilon()));

         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            ReferenceFrame changeFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            Vector3D angularPart = new Vector3D(originalAngularPart.getX(), originalAngularPart.getY(), originalAngularPart.getZ());
            Vector3D linearPart = new Vector3D(originalLinearPart.getX(), originalLinearPart.getY(), originalLinearPart.getZ());

            SpatialImpulse changeReferenceFrameSpatialImpulse = new SpatialImpulse(sourceBodyFrame, changeFrame, angularPart, linearPart);
            assertFalse(changeReferenceFrameSpatialImpulse.geometricallyEquals(originalSpatialImpulse, getEpsilon()));
         }, "geometricallyEquals should have thrown a ReferenceFrameMatchingException error when reference frame different for the vectors.");
      }
   }

   @Test
   public void testToString()
   {
      Random random = new Random(130375);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame sourceBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D originalAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         FrameVector3D originalLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, sourceFrame);
         SpatialImpulse originalSpatialImpulse = new SpatialImpulse(sourceBodyFrame, sourceFrame, originalAngularPart, originalLinearPart);

         assertEquals(originalSpatialImpulse.toString(), MecanoIOTools.getSpatialImpulseString(originalSpatialImpulse));
      }

   }
}
