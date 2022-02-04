package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import java.util.Random;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoTestTools;

public abstract class SpatialVectorBasicsTest<T extends SpatialVectorBasics> extends FixedFrameSpatialVectorBasicsTest<FixedFrameSpatialVectorBasics>
{
   public abstract T newEmptySpatialVector();

   public abstract T newRandomSpatialVector(Random random);

   public abstract T newSpatialVector(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart);

   public abstract T newCopySpatialVector(SpatialVectorReadOnly other);

   public abstract double getEpsilon();

   private static final int ITERATIONS = 1000;

   @Test
   public void testSetMatchingFrame()
   {

      Random random = new Random(21587);

      //Test setMatchingFrame of one Spatial Vector using another Spatial Vector 
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random),
                                                    EuclidCoreRandomTools.nextVector3D(random),
                                                    EuclidCoreRandomTools.nextVector3D(random));
         T actualSpatialVector = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random),
                                                  EuclidCoreRandomTools.nextVector3D(random),
                                                  EuclidCoreRandomTools.nextVector3D(random));
         T expectedSpatialVector = actualSpatialVector;

         expectedSpatialVector.setIncludingFrame(originalSpatialVector);
         expectedSpatialVector.changeFrame(actualSpatialVector.getReferenceFrame());
         actualSpatialVector.setMatchingFrame(originalSpatialVector);

         MecanoTestTools.assertSpatialVectorEquals(actualSpatialVector, expectedSpatialVector, getEpsilon());
      }

      //Test setMatchingFrame from an angular / linear part where the parts have the same Reference Frame
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random),
                                                    EuclidCoreRandomTools.nextVector3D(random),
                                                    EuclidCoreRandomTools.nextVector3D(random));
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         T actualSpatialVector = newSpatialVector(randomAngularPart.getReferenceFrame(), randomAngularPart, randomLinearPart);
         T expectedSpatialVector = actualSpatialVector;

         expectedSpatialVector.setIncludingFrame(originalSpatialVector);
         expectedSpatialVector.changeFrame(actualSpatialVector.getReferenceFrame());
         actualSpatialVector.setMatchingFrame(randomAngularPart, randomLinearPart);

         MecanoTestTools.assertSpatialVectorEquals(actualSpatialVector, expectedSpatialVector, getEpsilon());

         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            randomLinearPart.setReferenceFrame(ReferenceFrameTools.getWorldFrame());

            originalSpatialVector.setMatchingFrame(randomAngularPart, randomLinearPart);
         });

      }
   }

   @Test
   public void testSettersIncludingFrame()
   {
      //Test setIncludingFrame methods

      Random random = new Random(21623);

      //Test setIncludingFrame - sets a spatial vector equal to another spatial vector
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newRandomSpatialVector(random);

         assertNotEquals(originalSpatialVector, expectedSpatialVector);
         originalSpatialVector.setIncludingFrame(expectedSpatialVector);
         MecanoTestTools.assertSpatialVectorEquals(originalSpatialVector, expectedSpatialVector, getEpsilon());
      }

      //Test setIncludingFrame - given an angular part and linear part, updates the frame of this vector
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);

         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);

         //Setting AngularPart/LinearPart where the Reference Frames are the same
         assertNotEquals(originalSpatialVector.getAngularPart(), randomAngularPart);
         assertNotEquals(originalSpatialVector.getLinearPart(), randomLinearPart);

         originalSpatialVector.setIncludingFrame(randomAngularPart, randomLinearPart);

         assertEquals(originalSpatialVector.getAngularPart(), randomAngularPart);
         assertEquals(originalSpatialVector.getLinearPart(), randomLinearPart);
         assertEquals(originalSpatialVector.getReferenceFrame(), randomFrame);

         //Test setting AngularPart/LinearPart where the Reference Frames are not the same fails 
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            randomLinearPart.setReferenceFrame(ReferenceFrameTools.getWorldFrame());

            originalSpatialVector.setIncludingFrame(randomAngularPart, randomLinearPart);
         });
      }

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, and two Vector3Ds 
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);

         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);

         assertNotEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertNotEquals(randomSpatialVector.getAngularPart(), randomAngularPart);
         assertNotEquals(randomSpatialVector.getAngularPart(), randomLinearPart);

         randomSpatialVector.setIncludingFrame(randomFrame, randomAngularPart, randomLinearPart);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomSpatialVector.getAngularPart().getReferenceFrame(), randomFrame);
         assertEquals(randomSpatialVector.getLinearPart().getReferenceFrame(), randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(randomSpatialVector.getAngularPart(), randomAngularPart, getEpsilon());
         EuclidCoreTestTools.assertTuple3DEquals(randomSpatialVector.getLinearPart(), randomLinearPart, getEpsilon());
      }

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from an array. 
      //The components of the array are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         double[] randomVectorArray = new double[6];
         for (int index = 0; index < 6; index++)
            randomVectorArray[index] = random.nextDouble();

         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));

         randomSpatialVector.setIncludingFrame(randomFrame, randomVectorArray);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomVectorArray[0], randomSpatialVector.getAngularPartX());
         assertEquals(randomVectorArray[1], randomSpatialVector.getAngularPartY());
         assertEquals(randomVectorArray[2], randomSpatialVector.getAngularPartZ());
         assertEquals(randomVectorArray[3], randomSpatialVector.getLinearPartX());
         assertEquals(randomVectorArray[4], randomSpatialVector.getLinearPartY());
         assertEquals(randomVectorArray[5], randomSpatialVector.getLinearPartZ());
      }

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from an array with a starting point in the array. 
      //The components of the array are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         double[] randomVectorArray = new double[24];
         for (int index = 0; index < 24; index++)
            randomVectorArray[index] = random.nextDouble();

         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));

         int startingArrayIndex = i % 18;

         randomSpatialVector.setIncludingFrame(randomFrame, startingArrayIndex, randomVectorArray);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomVectorArray[startingArrayIndex], randomSpatialVector.getAngularPartX());
         assertEquals(randomVectorArray[startingArrayIndex + 1], randomSpatialVector.getAngularPartY());
         assertEquals(randomVectorArray[startingArrayIndex + 2], randomSpatialVector.getAngularPartZ());
         assertEquals(randomVectorArray[startingArrayIndex + 3], randomSpatialVector.getLinearPartX());
         assertEquals(randomVectorArray[startingArrayIndex + 4], randomSpatialVector.getLinearPartY());
         assertEquals(randomVectorArray[startingArrayIndex + 5], randomSpatialVector.getLinearPartZ());
      }

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from an float array. 
      //The components of the array are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         float[] randomVectorArray = new float[24];
         for (int index = 0; index < 6; index++)
            randomVectorArray[index] = random.nextFloat();

         assertNotEquals(randomSpatialVector.getReferenceFrame(), randomFrame);

         randomSpatialVector.setIncludingFrame(randomFrame, randomVectorArray);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomVectorArray[0], randomSpatialVector.getAngularPartX());
         assertEquals(randomVectorArray[1], randomSpatialVector.getAngularPartY());
         assertEquals(randomVectorArray[2], randomSpatialVector.getAngularPartZ());
         assertEquals(randomVectorArray[3], randomSpatialVector.getLinearPartX());
         assertEquals(randomVectorArray[4], randomSpatialVector.getLinearPartY());
         assertEquals(randomVectorArray[5], randomSpatialVector.getLinearPartZ());
      }

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from a float array with a starting point in the array. 
      //The components of the array are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         float[] randomVectorArray = new float[24];
         for (int index = 0; index < 24; index++)
            randomVectorArray[index] = random.nextFloat();

         assertNotEquals(randomSpatialVector.getReferenceFrame(), randomFrame);

         int startingArrayIndex = i % 18;

         randomSpatialVector.setIncludingFrame(randomFrame, startingArrayIndex, randomVectorArray);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomVectorArray[startingArrayIndex], randomSpatialVector.getAngularPartX());
         assertEquals(randomVectorArray[startingArrayIndex + 1], randomSpatialVector.getAngularPartY());
         assertEquals(randomVectorArray[startingArrayIndex + 2], randomSpatialVector.getAngularPartZ());
         assertEquals(randomVectorArray[startingArrayIndex + 3], randomSpatialVector.getLinearPartX());
         assertEquals(randomVectorArray[startingArrayIndex + 4], randomSpatialVector.getLinearPartY());
         assertEquals(randomVectorArray[startingArrayIndex + 5], randomSpatialVector.getLinearPartZ());
      }

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from a column DMatrix. 
      //The components of the DMatrix are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(6, 1);
         for (int row = 0; row < 6; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         assertNotEquals(randomSpatialVector.getReferenceFrame(), randomFrame);

         randomSpatialVector.setIncludingFrame(randomFrame, randomMatrix);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomMatrix.get(0, 0), randomSpatialVector.getAngularPartX());
         assertEquals(randomMatrix.get(1, 0), randomSpatialVector.getAngularPartY());
         assertEquals(randomMatrix.get(2, 0), randomSpatialVector.getAngularPartZ());
         assertEquals(randomMatrix.get(3, 0), randomSpatialVector.getLinearPartX());
         assertEquals(randomMatrix.get(4, 0), randomSpatialVector.getLinearPartY());
         assertEquals(randomMatrix.get(5, 0), randomSpatialVector.getLinearPartZ());
      }

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from a column DMatrix starting
      //at a specific row. The components of the DMatrix are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24, 1);
         for (int row = 0; row < 24; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         assertNotEquals(randomSpatialVector.getReferenceFrame(), randomFrame);

         int startingMatrixRow = i % 18;

         randomSpatialVector.setIncludingFrame(randomFrame, startingMatrixRow, randomMatrix);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomMatrix.get(startingMatrixRow, 0), randomSpatialVector.getAngularPartX());
         assertEquals(randomMatrix.get(startingMatrixRow + 1, 0), randomSpatialVector.getAngularPartY());
         assertEquals(randomMatrix.get(startingMatrixRow + 2, 0), randomSpatialVector.getAngularPartZ());
         assertEquals(randomMatrix.get(startingMatrixRow + 3, 0), randomSpatialVector.getLinearPartX());
         assertEquals(randomMatrix.get(startingMatrixRow + 4, 0), randomSpatialVector.getLinearPartY());
         assertEquals(randomMatrix.get(startingMatrixRow + 5, 0), randomSpatialVector.getLinearPartZ());
      }

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from a 
      //starting column/row of a DMatrix. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24, 24);
         for (int row = 0; row < 24; row++)
         {
            for (int column = 0; column < 24; column++)
            {
               randomMatrix.set(row, column, random.nextDouble());
            }
         }

         assertNotEquals(randomSpatialVector.getReferenceFrame(), randomFrame);

         int startingMatrixRow = i % 18;
         int startingMatrixColumn = i % 18;

         randomSpatialVector.setIncludingFrame(randomFrame, startingMatrixRow, startingMatrixColumn, randomMatrix);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomMatrix.get(startingMatrixRow, startingMatrixColumn), randomSpatialVector.getAngularPartX());
         assertEquals(randomMatrix.get(startingMatrixRow + 1, startingMatrixColumn), randomSpatialVector.getAngularPartY());
         assertEquals(randomMatrix.get(startingMatrixRow + 2, startingMatrixColumn), randomSpatialVector.getAngularPartZ());
         assertEquals(randomMatrix.get(startingMatrixRow + 3, startingMatrixColumn), randomSpatialVector.getLinearPartX());
         assertEquals(randomMatrix.get(startingMatrixRow + 4, startingMatrixColumn), randomSpatialVector.getLinearPartY());
         assertEquals(randomMatrix.get(startingMatrixRow + 5, startingMatrixColumn), randomSpatialVector.getLinearPartZ());
      }

   }

   @Test
   public void testSetReferenceFrame()
   {
      Random random = new Random(24523);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T spatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         assertNotEquals(spatialVector.getReferenceFrame(), randomFrame);

         spatialVector.setReferenceFrame(randomFrame);

         assertEquals(spatialVector.getReferenceFrame(), randomFrame);
         assertEquals(spatialVector.getAngularPart().getReferenceFrame(), randomFrame);
         assertEquals(spatialVector.getLinearPart().getReferenceFrame(), randomFrame);
      }
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(21623);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         assertNotEquals(randomSpatialVector.getReferenceFrame(), randomFrame);

         randomSpatialVector.setToZero(randomFrame);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomSpatialVector.getAngularPart().getReferenceFrame(), randomFrame);
         assertEquals(randomSpatialVector.getLinearPart().getReferenceFrame(), randomFrame);
         assertEquals(randomSpatialVector.getAngularPartX(), 0);
         assertEquals(randomSpatialVector.getAngularPartY(), 0);
         assertEquals(randomSpatialVector.getAngularPartZ(), 0);
         assertEquals(randomSpatialVector.getLinearPartX(), 0);
         assertEquals(randomSpatialVector.getLinearPartY(), 0);
         assertEquals(randomSpatialVector.getLinearPartZ(), 0);
      }
   }

   @Test
   public void testSetToNaN()
   {
      //Test SetToNan - sets all the components of this vector to NaN and sets its reference frame
      Random random = new Random(31843);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         assertNotEquals(randomSpatialVector.getReferenceFrame(), randomFrame);

         randomSpatialVector.setToNaN(randomFrame);

         assertEquals(randomSpatialVector.getReferenceFrame(), randomFrame);
         assertEquals(randomSpatialVector.getAngularPart().getReferenceFrame(), randomFrame);
         assertEquals(randomSpatialVector.getLinearPart().getReferenceFrame(), randomFrame);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(randomSpatialVector.getAngularPart());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(randomSpatialVector.getLinearPart());
      }
   }

}