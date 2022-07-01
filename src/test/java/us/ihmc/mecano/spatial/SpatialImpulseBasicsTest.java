package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialImpulseBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTools;

public abstract class SpatialImpulseBasicsTest<T extends SpatialImpulseBasics> extends FixedFrameSpatialImpulseBasicsTest<FixedFrameSpatialImpulseBasics>
{
   public abstract T newEmptySpatialImpulse();

   public abstract T newRandomSpatialImpulse(Random random);

   public abstract T newSpatialImpulse(ReferenceFrame bodyFrame, SpatialVector spatialVector);

   public abstract T newCopySpatialImpulse(SpatialImpulseReadOnly other);

   public abstract double getEpsilon();

   private static final int ITERATIONS = 1000;
   
   @Test
   public void testSetMatchingFrame()
   {
      Random random = new Random(21623);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            //Body frame different should throw an exception
            T sourceSpatialImpulse = newRandomSpatialImpulse(random);
            T destinationSpatialImpulse = newRandomSpatialImpulse(random);

            destinationSpatialImpulse.setMatchingFrame(sourceSpatialImpulse);
         }, "SetMatchingFrame should have thrown a ReferenceFrameMatchingException error when any of the reference frames different for the SpatialImpulses.");

      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         //Body frame same, and reference frames different
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector sourceSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
         SpatialVector destinationSpatialVector = MecanoRandomTools.nextSpatialVector(random, EuclidFrameRandomTools.nextReferenceFrame(random));
         T sourceSpatialImpulse = newSpatialImpulse(randomBodyFrame, sourceSpatialVector);
         T destinationSpatialImpulse = newSpatialImpulse(randomBodyFrame, destinationSpatialVector);
         T expectedSpatialVector = destinationSpatialImpulse;
         
         expectedSpatialVector.setIncludingFrame(sourceSpatialImpulse);
         expectedSpatialVector.changeFrame(destinationSpatialImpulse.getReferenceFrame());

         destinationSpatialImpulse.setMatchingFrame(sourceSpatialImpulse);

         assertEquals(destinationSpatialImpulse.getBodyFrame(), sourceSpatialImpulse.getBodyFrame());
         assertEquals(destinationSpatialImpulse.getReferenceFrame(), sourceSpatialImpulse.getReferenceFrame());
         assertEquals(destinationSpatialImpulse.getAngularPart(), expectedSpatialVector.getAngularPart());
         assertEquals(destinationSpatialImpulse.getLinearPart(), expectedSpatialVector.getLinearPart());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         T sourceSpatialImpulse = newRandomSpatialImpulse(random);
         SpatialVector spatialVectorWithSameReferenceFrame = MecanoRandomTools.nextSpatialVector(random, sourceSpatialImpulse.getReferenceFrame());
         T destinationSpatialImpulse = newSpatialImpulse(sourceSpatialImpulse.getBodyFrame(), spatialVectorWithSameReferenceFrame);

         destinationSpatialImpulse.setMatchingFrame(sourceSpatialImpulse);
         
         assertEquals(sourceSpatialImpulse, destinationSpatialImpulse);

      }
   }

   @Test
   public void testSetBodyFrame()
   {

      Random random = new Random(21587);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialImpulse = newRandomSpatialImpulse(random);
         T expectedSpatialImpulse = newCopySpatialImpulse(originalSpatialImpulse);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         expectedSpatialImpulse.setBodyFrame(randomBodyFrame);

         //verify only the body frame changed and the rest of the SpatialImpulseBasics components stayed the same as the original
         assertEquals(expectedSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(expectedSpatialImpulse.getReferenceFrame(), originalSpatialImpulse.getReferenceFrame());
         assertEquals(expectedSpatialImpulse.getAngularPart(), originalSpatialImpulse.getAngularPart());
         assertEquals(expectedSpatialImpulse.getLinearPart(), originalSpatialImpulse.getLinearPart());
      }
   }

   @Test
   public void testToZero()
   {

      Random random = new Random(19547);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         randomSpatialImpulse.setToZero(randomBodyFrame, randomReferenceFrame);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPartX(), 0);
         assertEquals(randomSpatialImpulse.getAngularPartY(), 0);
         assertEquals(randomSpatialImpulse.getAngularPartX(), 0);
         assertEquals(randomSpatialImpulse.getLinearPartX(), 0);
         assertEquals(randomSpatialImpulse.getLinearPartY(), 0);
         assertEquals(randomSpatialImpulse.getLinearPartZ(), 0);
      }
   }

   @Test
   public void testToNaN()
   {

      Random random = new Random(19547);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         randomSpatialImpulse.setToNaN(randomBodyFrame, randomReferenceFrame);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(randomSpatialImpulse.getAngularPart());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(randomSpatialImpulse.getLinearPart());
      }
   }

   @Test
   public void testSetIncludingFrame()
   {

      Random random = new Random(25414);

      //SetIncludingFrame using a Spatial Vector
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialImpulse = newRandomSpatialImpulse(random);
         T expectedSpatialImpulse = newCopySpatialImpulse(originalSpatialImpulse);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector randomSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);

         expectedSpatialImpulse.setIncludingFrame(randomSpatialVector);

         assertEquals(expectedSpatialImpulse.getBodyFrame(), originalSpatialImpulse.getBodyFrame());
         assertEquals(expectedSpatialImpulse.getReferenceFrame(), randomSpatialVector.getReferenceFrame());
         assertEquals(expectedSpatialImpulse.getAngularPart().getReferenceFrame(), randomSpatialVector.getReferenceFrame());
         assertEquals(expectedSpatialImpulse.getLinearPart().getReferenceFrame(), randomSpatialVector.getReferenceFrame());
         assertEquals(expectedSpatialImpulse.getAngularPart(), randomSpatialVector.getAngularPart());
         assertEquals(expectedSpatialImpulse.getLinearPart(), randomSpatialVector.getLinearPart());
      }

      //SetIncludingFrame using a body frame and a Spatial Vector
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector randomSpatialVector = MecanoRandomTools.nextSpatialVector(random, randomReferenceFrame);

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomSpatialVector);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomSpatialVector.getReferenceFrame());
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomSpatialVector.getReferenceFrame());
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomSpatialVector.getReferenceFrame());
         assertEquals(randomSpatialImpulse.getAngularPart(), randomSpatialVector.getAngularPart());
         assertEquals(randomSpatialImpulse.getLinearPart(), randomSpatialVector.getLinearPart());
      }

      //SetIncludingFrame using a body frame and angular and linear frame vectors where the reference frames of the vectors are equal
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomAngularPart, randomLinearPart);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomAngularPart.getReferenceFrame());
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomAngularPart.getReferenceFrame());
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomLinearPart.getReferenceFrame());
         assertEquals(randomSpatialImpulse.getAngularPart(), randomAngularPart);
         assertEquals(randomSpatialImpulse.getLinearPart(), randomLinearPart);
      }

      //SetIncludingFrame using a body frame and angular and linear frame vectors where the reference frames of the vectors are different - throws ReferenceFrameMismatchException
      for (int i = 0; i < ITERATIONS; i++)
      {

         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            T randomSpatialImpulse = newRandomSpatialImpulse(random);
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrameAngular = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomReferenceFrameLinear = EuclidFrameRandomTools.nextReferenceFrame(random);
            FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrameAngular);
            FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrameLinear);

            randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomAngularPart, randomLinearPart);
         }, "SetIncludingFrame should have thrown a ReferenceFrameMatchingException error when reference frame different for the vectors.");
      }

      //SetIncludingFrame using a body frame, a reference frame and angular and linear 3D vectors
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         EuclidCoreTestTools.assertEquals(randomSpatialImpulse.getAngularPart(), randomAngularPart, getEpsilon());
         EuclidCoreTestTools.assertEquals(randomSpatialImpulse.getLinearPart(), randomLinearPart, getEpsilon());
      }

      //SetIncludingFrame using a body frame, a reference frame and a double array. The components are read in the following order: .angularPartX, angularPartY,
      //angularPartZ, linearPartX, linearPartY, linearPartZ.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         double[] randomVectorArray = new double[6];
         for (int index = 0; index < 6; index++)
            randomVectorArray[index] = random.nextDouble();

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, randomVectorArray);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPartX(), randomVectorArray[0]);
         assertEquals(randomSpatialImpulse.getAngularPartY(), randomVectorArray[1]);
         assertEquals(randomSpatialImpulse.getAngularPartZ(), randomVectorArray[2]);
         assertEquals(randomSpatialImpulse.getLinearPartX(), randomVectorArray[3]);
         assertEquals(randomSpatialImpulse.getLinearPartY(), randomVectorArray[4]);
         assertEquals(randomSpatialImpulse.getLinearPartZ(), randomVectorArray[5]);
      }

      //SetIncludingFrame using a body frame, a reference frame and a double array from a starting index. The components are read in the following order: .angularPartX, angularPartY,
      //angularPartZ, linearPartX, linearPartY, linearPartZ.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         double[] randomVectorArray = new double[24];
         for (int index = 0; index < 24; index++)
            randomVectorArray[index] = random.nextDouble();

         int startingArrayIndex = i % 18;

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, startingArrayIndex, randomVectorArray);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPartX(), randomVectorArray[startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getAngularPartY(), randomVectorArray[++startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getAngularPartZ(), randomVectorArray[++startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getLinearPartX(), randomVectorArray[++startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getLinearPartY(), randomVectorArray[++startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getLinearPartZ(), randomVectorArray[++startingArrayIndex]);
      }

      //SetIncludingFrame using a body frame, a reference frame and a float array. The components are read in the following order: .angularPartX, angularPartY,
      //angularPartZ, linearPartX, linearPartY, linearPartZ.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         float[] randomVectorArray = new float[6];
         for (int index = 0; index < 6; index++)
            randomVectorArray[index] = random.nextFloat();

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, randomVectorArray);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPartX(), randomVectorArray[0], getEpsilon());
         assertEquals(randomSpatialImpulse.getAngularPartY(), randomVectorArray[1], getEpsilon());
         assertEquals(randomSpatialImpulse.getAngularPartZ(), randomVectorArray[2], getEpsilon());
         assertEquals(randomSpatialImpulse.getLinearPartX(), randomVectorArray[3], getEpsilon());
         assertEquals(randomSpatialImpulse.getLinearPartY(), randomVectorArray[4], getEpsilon());
         assertEquals(randomSpatialImpulse.getLinearPartZ(), randomVectorArray[5], getEpsilon());
      }

      //SetIncludingFrame using a body frame, a reference frame and a float array from a starting point. The components are read in the following order: .angularPartX, angularPartY,
      //angularPartZ, linearPartX, linearPartY, linearPartZ.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         float[] randomVectorArray = new float[24];
         for (int index = 0; index < 24; index++)
            randomVectorArray[index] = random.nextFloat();

         int startingArrayIndex = i % 18;

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, startingArrayIndex, randomVectorArray);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPartX(), randomVectorArray[startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getAngularPartY(), randomVectorArray[++startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getAngularPartZ(), randomVectorArray[++startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getLinearPartX(), randomVectorArray[++startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getLinearPartY(), randomVectorArray[++startingArrayIndex]);
         assertEquals(randomSpatialImpulse.getLinearPartZ(), randomVectorArray[++startingArrayIndex]);
      }

      //SetIncludingFrame using a body frame, a reference frame and a DMatrix. The components are read in the following order: .angularPartX, angularPartY,
      //angularPartZ, linearPartX, linearPartY, linearPartZ.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(6, 1);
         for (int row = 0; row < 6; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, randomMatrix);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPartX(), randomMatrix.get(0, 0));
         assertEquals(randomSpatialImpulse.getAngularPartY(), randomMatrix.get(1, 0));
         assertEquals(randomSpatialImpulse.getAngularPartZ(), randomMatrix.get(2, 0));
         assertEquals(randomSpatialImpulse.getLinearPartX(), randomMatrix.get(3, 0));
         assertEquals(randomSpatialImpulse.getLinearPartY(), randomMatrix.get(4, 0));
         assertEquals(randomSpatialImpulse.getLinearPartZ(), randomMatrix.get(5, 0));
      }

      //SetIncludingFrame using a body frame, a reference frame and a DMatrix from a starting row. The components are read in the following order: .angularPartX, angularPartY,
      //angularPartZ, linearPartX, linearPartY, linearPartZ.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24, 1);
         for (int row = 0; row < 24; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         int startingArrayIndex = i % 18;

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, startingArrayIndex, randomMatrix);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPartX(), randomMatrix.get(startingArrayIndex, 0));
         assertEquals(randomSpatialImpulse.getAngularPartY(), randomMatrix.get(++startingArrayIndex, 0));
         assertEquals(randomSpatialImpulse.getAngularPartZ(), randomMatrix.get(++startingArrayIndex, 0));
         assertEquals(randomSpatialImpulse.getLinearPartX(), randomMatrix.get(++startingArrayIndex, 0));
         assertEquals(randomSpatialImpulse.getLinearPartY(), randomMatrix.get(++startingArrayIndex, 0));
         assertEquals(randomSpatialImpulse.getLinearPartZ(), randomMatrix.get(++startingArrayIndex, 0));
      }

      //SetIncludingFrame using a body frame, a reference frame and a DMatrix from a starting row and starting column. The components are read in the following order: .angularPartX, angularPartY,
      //angularPartZ, linearPartX, linearPartY, linearPartZ.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24, 24);
         for (int row = 0; row < 24; row++)
         {
            for (int column = 0; column < 24; column++)
            {
               randomMatrix.set(row, column, random.nextDouble());
            }
         }

         int startingArrayIndex = i % 18;
         int startingMatrixColumn = i % 18;

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, startingArrayIndex, startingMatrixColumn, randomMatrix);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPartX(), randomMatrix.get(startingArrayIndex, startingMatrixColumn));
         assertEquals(randomSpatialImpulse.getAngularPartY(), randomMatrix.get(++startingArrayIndex, startingMatrixColumn));
         assertEquals(randomSpatialImpulse.getAngularPartZ(), randomMatrix.get(++startingArrayIndex, startingMatrixColumn));
         assertEquals(randomSpatialImpulse.getLinearPartX(), randomMatrix.get(++startingArrayIndex, startingMatrixColumn));
         assertEquals(randomSpatialImpulse.getLinearPartY(), randomMatrix.get(++startingArrayIndex, startingMatrixColumn));
         assertEquals(randomSpatialImpulse.getLinearPartZ(), randomMatrix.get(++startingArrayIndex, startingMatrixColumn));
      }

      //SetIncludingFrame using a body frame, a reference frame and sets the spatial impulse given a 3D moment and 3D force that are exerted at
      //pointOfApplication and updates this vector reference frame.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         Point3D randomPointOfApplication = EuclidCoreRandomTools.nextPoint3D(random);

         Vector3D expectedAngularPart = new Vector3D(randomAngularPart);
         MecanoTools.addCrossToVector(randomPointOfApplication, randomLinearPart, expectedAngularPart);

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomReferenceFrame, randomAngularPart, randomLinearPart, randomPointOfApplication);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         EuclidCoreTestTools.assertEquals(randomSpatialImpulse.getAngularPart(), expectedAngularPart, getEpsilon());
         EuclidCoreTestTools.assertEquals(randomSpatialImpulse.getLinearPart(), randomLinearPart, getEpsilon());
      }

      //SetIncludingFrame using a body frame, a angular and linear Frame Vector and sets the spatial impulse given a 3D moment and 3D force that are exerted at
      //pointOfApplication and updates this vector reference frame.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialImpulse = newRandomSpatialImpulse(random);
         ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame randomReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomReferenceFrame);
         FramePoint3D randomPointOfApplication = EuclidFrameRandomTools.nextFramePoint3D(random, randomReferenceFrame);

         FrameVector3D expectedAngularPart = new FrameVector3D(randomAngularPart);
         MecanoTools.addCrossToVector(randomPointOfApplication, randomLinearPart, expectedAngularPart);

         randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomAngularPart, randomLinearPart, randomPointOfApplication);

         assertEquals(randomSpatialImpulse.getBodyFrame(), randomBodyFrame);
         assertEquals(randomSpatialImpulse.getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getAngularPart().getReferenceFrame(), randomReferenceFrame);
         assertEquals(randomSpatialImpulse.getLinearPart().getReferenceFrame(), randomReferenceFrame);
         EuclidCoreTestTools.assertEquals(randomSpatialImpulse.getAngularPart(), expectedAngularPart, getEpsilon());
         EuclidCoreTestTools.assertEquals(randomSpatialImpulse.getLinearPart(), randomLinearPart, getEpsilon());
      }

      //SetIncludingFrame using a body frame, a angular and linear Frame Vector and reference frames are different should cause a ReferenceFrameMismatchException.
      for (int i = 0; i < ITERATIONS; i++)
      {

         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            T randomSpatialImpulse = newRandomSpatialImpulse(random);
            ReferenceFrame randomBodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomAnularReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            ReferenceFrame randomLinearReferenceFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomAnularReferenceFrame);
            FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomLinearReferenceFrame);
            FramePoint3D randomPointOfApplication = EuclidFrameRandomTools.nextFramePoint3D(random, randomAnularReferenceFrame);

            FrameVector3D expectedAngularPart = new FrameVector3D(randomAngularPart);
            MecanoTools.addCrossToVector(randomPointOfApplication, randomLinearPart, expectedAngularPart);

            randomSpatialImpulse.setIncludingFrame(randomBodyFrame, randomAngularPart, randomLinearPart, randomPointOfApplication);
         }, "SetIncludingFrame should have thrown a ReferenceFrameMatchingException error when reference frame different for the vectors.");
      }
   }
}
