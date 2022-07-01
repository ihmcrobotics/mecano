package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialImpulseBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTools;

public abstract class FixedFrameSpatialImpulseBasicsTest<T extends FixedFrameSpatialImpulseBasics>
{
   public abstract T newEmptySpatialImpulse();

   public abstract T newRandomSpatialImpulse(Random random);

   public abstract T newSpatialImpulse(ReferenceFrame bodyFrame, SpatialVector spatialVector);

   public abstract T newCopySpatialImpulse(SpatialImpulseReadOnly other);

   public abstract double getEpsilon();

   public abstract void testSetMatchingFrame();

   private static final int ITERATIONS = 1000;

   @Test
   public void testSetters()
   {
      Random random = new Random(21623);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            T sourceSpatialImpulse = newRandomSpatialImpulse(random);
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);

            destinationSpatialImpulse.set(sourceSpatialImpulse);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector sourceSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T sourceSpatialImpulse = newSpatialImpulse(randomBodyFrame, sourceSpatialVector);
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

            destinationSpatialImpulse.set(sourceSpatialImpulse);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         T sourceSpatialImpulse = newRandomSpatialImpulse(random);
         SpatialVector spatialVectorWithSameReferenceFrame = MecanoRandomTools.nextSpatialVector(random, sourceSpatialImpulse.getReferenceFrame());
         T destinationSpatialImpulse = newSpatialImpulse(sourceSpatialImpulse.getBodyFrame(), spatialVectorWithSameReferenceFrame);

         destinationSpatialImpulse.set(sourceSpatialImpulse);

         assertEquals(sourceSpatialImpulse, destinationSpatialImpulse);

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector sourceSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);

            destinationSpatialImpulse.set(randomBodyFrame, sourceSpatialVector);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector sourceSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

            destinationSpatialImpulse.set(randomBodyFrame, sourceSpatialVector);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         SpatialVector sourceSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

         destinationSpatialImpulse.set(randomBodyFrame, sourceSpatialVector);

         assertEquals(destinationSpatialImpulse.getReferenceFrame(), sourceSpatialVector.getReferenceFrame());
         assertEquals(destinationSpatialImpulse.getAngularPart(), sourceSpatialVector.getAngularPart());
         assertEquals(destinationSpatialImpulse.getLinearPart(), sourceSpatialVector.getLinearPart());

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);
            FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
            FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

            destinationSpatialImpulse.set(randomBodyFrame, randomAngularPart, randomLinearPart);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
            FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
            FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

            destinationSpatialImpulse.set(randomBodyFrame, randomAngularPart, randomLinearPart);

         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Body Frame the same / Reference Frame different should throw an exception
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

         destinationSpatialImpulse.set(randomBodyFrame, randomAngularPart, randomLinearPart);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), randomAngularPart.getReferenceFrame());
         assertEquals(destinationSpatialImpulse.getAngularPart(), randomAngularPart);
         assertEquals(destinationSpatialImpulse.getLinearPart(), randomLinearPart);

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);
            Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
            Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);

            destinationSpatialImpulse.set(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
            Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
            Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);

            destinationSpatialImpulse.set(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart);

         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Body Frame the same / Reference Frame different should throw an exception
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);

         destinationSpatialImpulse.set(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         EuclidCoreTestTools.assertEquals(destinationSpatialImpulse.getAngularPart(), randomAngularPart, getEpsilon());
         EuclidCoreTestTools.assertEquals(destinationSpatialImpulse.getLinearPart(), randomLinearPart, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Body Frames different
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
            Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);
            Point3D randomPointOfApplication = EuclidCoreRandomTools.nextPoint3D(random);
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
            T destinationSpatialImpulse = newSpatialImpulse(EuclidFrameRandomTools.nextReferenceFrame(random), destinationSpatialVector);

            destinationSpatialImpulse.set(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart, randomPointOfApplication);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         //Reference Frames different
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
            Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);
            Point3D randomPointOfApplication = EuclidCoreRandomTools.nextPoint3D(random);
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

            destinationSpatialImpulse.set(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart, randomPointOfApplication);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         Point3D randomPointOfApplication = EuclidCoreRandomTools.nextPoint3D(random);
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

         Vector3D expectedAngularPart = new Vector3D(randomAngularPart);
         MecanoTools.addCrossToVector(randomPointOfApplication, randomLinearPart, expectedAngularPart);

         destinationSpatialImpulse.set(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart, randomPointOfApplication);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(destinationSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(destinationSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         EuclidCoreTestTools.assertEquals(destinationSpatialImpulse.getAngularPart(), expectedAngularPart, getEpsilon());
         EuclidCoreTestTools.assertEquals(destinationSpatialImpulse.getLinearPart(), randomLinearPart, getEpsilon());
      }
   }

   @Test
   public void testAdds()
   {
      Random random = new Random(21623);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            T sourceSpatialImpulse = newRandomSpatialImpulse(random);
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);

            destinationSpatialImpulse.add(sourceSpatialImpulse);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector sourceSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T sourceSpatialImpulse = newSpatialImpulse(randomBodyFrame, sourceSpatialVector);
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

            destinationSpatialImpulse.add(sourceSpatialImpulse);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         T sourceSpatialImpulse = newRandomSpatialImpulse(random);
         SpatialVector spatialVectorWithSameReferenceFrame = MecanoRandomTools.nextSpatialVector(random, sourceSpatialImpulse.getReferenceFrame());
         T actualSpatialImpulse = newSpatialImpulse(sourceSpatialImpulse.getBodyFrame(), spatialVectorWithSameReferenceFrame);
         T destinationSpatialImpulse = newCopySpatialImpulse(actualSpatialImpulse);

         destinationSpatialImpulse.add(sourceSpatialImpulse);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), sourceSpatialImpulse.getBodyFrame());
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), sourceSpatialImpulse.getReferenceFrame());
         assertEquals(destinationSpatialImpulse.getAngularPartX(), sourceSpatialImpulse.getAngularPartX() + actualSpatialImpulse.getAngularPartX());
         assertEquals(destinationSpatialImpulse.getAngularPartY(), sourceSpatialImpulse.getAngularPartY() + actualSpatialImpulse.getAngularPartY());
         assertEquals(destinationSpatialImpulse.getAngularPartZ(), sourceSpatialImpulse.getAngularPartZ() + actualSpatialImpulse.getAngularPartZ());
         assertEquals(destinationSpatialImpulse.getLinearPartX(), sourceSpatialImpulse.getLinearPartX() + actualSpatialImpulse.getLinearPartX());
         assertEquals(destinationSpatialImpulse.getLinearPartY(), sourceSpatialImpulse.getLinearPartY() + actualSpatialImpulse.getLinearPartY());
         assertEquals(destinationSpatialImpulse.getLinearPartZ(), sourceSpatialImpulse.getLinearPartZ() + actualSpatialImpulse.getLinearPartZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector originalSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);

            destinationSpatialImpulse.add(randomBodyFrame, originalSpatialVector);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector originalSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

            destinationSpatialImpulse.add(randomBodyFrame, originalSpatialVector);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector originalSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
         T actualSpatialImpulse = newCopySpatialImpulse(destinationSpatialImpulse);

         destinationSpatialImpulse.add(randomBodyFrame, originalSpatialVector);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(destinationSpatialImpulse.getAngularPartX(), originalSpatialVector.getAngularPartX() + actualSpatialImpulse.getAngularPartX());
         assertEquals(destinationSpatialImpulse.getAngularPartY(), originalSpatialVector.getAngularPartY() + actualSpatialImpulse.getAngularPartY());
         assertEquals(destinationSpatialImpulse.getAngularPartZ(), originalSpatialVector.getAngularPartZ() + actualSpatialImpulse.getAngularPartZ());
         assertEquals(destinationSpatialImpulse.getLinearPartX(), originalSpatialVector.getLinearPartX() + actualSpatialImpulse.getLinearPartX());
         assertEquals(destinationSpatialImpulse.getLinearPartY(), originalSpatialVector.getLinearPartY() + actualSpatialImpulse.getLinearPartY());
         assertEquals(destinationSpatialImpulse.getLinearPartZ(), originalSpatialVector.getLinearPartZ() + actualSpatialImpulse.getLinearPartZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
            FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

            destinationSpatialImpulse.add(randomBodyFrame, randomAngularPart, randomLinearPart);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
            FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
            FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

            destinationSpatialImpulse.add(randomBodyFrame, randomAngularPart, randomLinearPart);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
         T actualSpatialImpulse = newCopySpatialImpulse(destinationSpatialImpulse);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

         destinationSpatialImpulse.add(randomBodyFrame, randomAngularPart, randomLinearPart);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(destinationSpatialImpulse.getAngularPartX(), randomAngularPart.getX() + actualSpatialImpulse.getAngularPartX());
         assertEquals(destinationSpatialImpulse.getAngularPartY(), randomAngularPart.getY() + actualSpatialImpulse.getAngularPartY());
         assertEquals(destinationSpatialImpulse.getAngularPartZ(), randomAngularPart.getZ() + actualSpatialImpulse.getAngularPartZ());
         assertEquals(destinationSpatialImpulse.getLinearPartX(), randomLinearPart.getX() + actualSpatialImpulse.getLinearPartX());
         assertEquals(destinationSpatialImpulse.getLinearPartY(), randomLinearPart.getY() + actualSpatialImpulse.getLinearPartY());
         assertEquals(destinationSpatialImpulse.getLinearPartZ(), randomLinearPart.getZ() + actualSpatialImpulse.getLinearPartZ());
      }
   }

   @Test
   public void testSubs()
   {
      Random random = new Random(21623);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            T sourceSpatialImpulse = newRandomSpatialImpulse(random);
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);

            destinationSpatialImpulse.sub(sourceSpatialImpulse);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector sourceSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T sourceSpatialImpulse = newSpatialImpulse(randomBodyFrame, sourceSpatialVector);
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

            destinationSpatialImpulse.sub(sourceSpatialImpulse);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         T sourceSpatialImpulse = newRandomSpatialImpulse(random);
         SpatialVector spatialVectorWithSameReferenceFrame = MecanoRandomTools.nextSpatialVector(random, sourceSpatialImpulse.getReferenceFrame());
         T actualSpatialImpulse = newSpatialImpulse(sourceSpatialImpulse.getBodyFrame(), spatialVectorWithSameReferenceFrame);
         T destinationSpatialImpulse = newCopySpatialImpulse(actualSpatialImpulse);

         destinationSpatialImpulse.sub(sourceSpatialImpulse);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), sourceSpatialImpulse.getBodyFrame());
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), sourceSpatialImpulse.getReferenceFrame());
         assertEquals(destinationSpatialImpulse.getAngularPartX(), actualSpatialImpulse.getAngularPartX() - sourceSpatialImpulse.getAngularPartX());
         assertEquals(destinationSpatialImpulse.getAngularPartY(), actualSpatialImpulse.getAngularPartY() - sourceSpatialImpulse.getAngularPartY());
         assertEquals(destinationSpatialImpulse.getAngularPartZ(), actualSpatialImpulse.getAngularPartZ() - sourceSpatialImpulse.getAngularPartZ());
         assertEquals(destinationSpatialImpulse.getLinearPartX(), actualSpatialImpulse.getLinearPartX() - sourceSpatialImpulse.getLinearPartX());
         assertEquals(destinationSpatialImpulse.getLinearPartY(), actualSpatialImpulse.getLinearPartY() - sourceSpatialImpulse.getLinearPartY());
         assertEquals(destinationSpatialImpulse.getLinearPartZ(), actualSpatialImpulse.getLinearPartZ() - sourceSpatialImpulse.getLinearPartZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector originalSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);

            destinationSpatialImpulse.sub(randomBodyFrame, originalSpatialVector);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector originalSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);

            destinationSpatialImpulse.sub(randomBodyFrame, originalSpatialVector);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector originalSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
         T actualSpatialImpulse = newCopySpatialImpulse(destinationSpatialImpulse);

         destinationSpatialImpulse.sub(randomBodyFrame, originalSpatialVector);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(destinationSpatialImpulse.getAngularPartX(), actualSpatialImpulse.getAngularPartX() - originalSpatialVector.getAngularPartX());
         assertEquals(destinationSpatialImpulse.getAngularPartY(), actualSpatialImpulse.getAngularPartY() - originalSpatialVector.getAngularPartY());
         assertEquals(destinationSpatialImpulse.getAngularPartZ(), actualSpatialImpulse.getAngularPartZ() - originalSpatialVector.getAngularPartZ());
         assertEquals(destinationSpatialImpulse.getLinearPartX(), actualSpatialImpulse.getLinearPartX() - originalSpatialVector.getLinearPartX());
         assertEquals(destinationSpatialImpulse.getLinearPartY(), actualSpatialImpulse.getLinearPartY() - originalSpatialVector.getLinearPartY());
         assertEquals(destinationSpatialImpulse.getLinearPartZ(), actualSpatialImpulse.getLinearPartZ() - originalSpatialVector.getLinearPartZ());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
            FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

            destinationSpatialImpulse.sub(randomBodyFrame, randomAngularPart, randomLinearPart);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body Frame the same / Reference Frame different should throw an exception
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
            FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
            FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

            destinationSpatialImpulse.sub(randomBodyFrame, randomAngularPart, randomLinearPart);
         }, "Set should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
         T actualSpatialImpulse = newCopySpatialImpulse(destinationSpatialImpulse);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

         destinationSpatialImpulse.sub(randomBodyFrame, randomAngularPart, randomLinearPart);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(destinationSpatialImpulse.getAngularPartX(), actualSpatialImpulse.getAngularPartX() - randomAngularPart.getX());
         assertEquals(destinationSpatialImpulse.getAngularPartY(), actualSpatialImpulse.getAngularPartY() - randomAngularPart.getY());
         assertEquals(destinationSpatialImpulse.getAngularPartZ(), actualSpatialImpulse.getAngularPartZ() - randomAngularPart.getZ());
         assertEquals(destinationSpatialImpulse.getLinearPartX(), actualSpatialImpulse.getLinearPartX() - randomLinearPart.getX());
         assertEquals(destinationSpatialImpulse.getLinearPartY(), actualSpatialImpulse.getLinearPartY() - randomLinearPart.getY());
         assertEquals(destinationSpatialImpulse.getLinearPartZ(), actualSpatialImpulse.getLinearPartZ() - randomLinearPart.getZ());
      }
   }
}
