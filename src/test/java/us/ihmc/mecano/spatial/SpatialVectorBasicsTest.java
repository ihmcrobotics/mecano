package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import java.util.Random;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

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
         T originalSpatialVector = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         T actualSpatialVector = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         T expectedSpatialVector = actualSpatialVector;

         expectedSpatialVector.setIncludingFrame(originalSpatialVector);
         expectedSpatialVector.changeFrame(actualSpatialVector.getReferenceFrame());
         
         actualSpatialVector.setMatchingFrame(originalSpatialVector);
         
         assertTrue(actualSpatialVector.equals(expectedSpatialVector));
      }
      
      //Test setMatchingFrame from an angular / linear part where the parts have the same Reference Frame
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);       
         T actualSpatialVector = newSpatialVector(randomAngularPart.getReferenceFrame(), randomAngularPart, randomLinearPart);
         T expectedSpatialVector = actualSpatialVector;

         expectedSpatialVector.setIncludingFrame(originalSpatialVector);
         expectedSpatialVector.changeFrame(actualSpatialVector.getReferenceFrame());
         
         actualSpatialVector.setMatchingFrame(randomAngularPart, randomLinearPart);
         
         assertTrue(actualSpatialVector.equals(expectedSpatialVector));
         
         //Test setMatchingFrame when AngularPart/LinearPart have different Reference Frames fails 
         try
         {
            randomLinearPart.setReferenceFrame(ReferenceFrameTools.getWorldFrame());
            
            originalSpatialVector.setMatchingFrame(randomAngularPart, randomLinearPart);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
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

         assertFalse(originalSpatialVector.equals(expectedSpatialVector));
         originalSpatialVector.setIncludingFrame(expectedSpatialVector);
         assertTrue(originalSpatialVector.equals(expectedSpatialVector));
      }
      
      //Test setIncludingFrame - given an angular part and linear part, updates the frame of this vector
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);

         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);

         //Setting AngularPart/LinearPart where the Reference Frames are the same
         assertFalse(originalSpatialVector.getAngularPart().equals(randomAngularPart));
         assertFalse(originalSpatialVector.getLinearPart().equals(randomLinearPart));
         originalSpatialVector.setIncludingFrame(randomAngularPart,randomLinearPart);
         assertTrue(originalSpatialVector.getAngularPart().equals(randomAngularPart));
         assertTrue(originalSpatialVector.getLinearPart().equals(randomLinearPart));
         assertTrue(originalSpatialVector.getReferenceFrame().equals(randomFrame));
         
         //Test setting AngularPart/LinearPart where the Reference Frames are not the same fails 
         try
         {
            randomLinearPart.setReferenceFrame(ReferenceFrameTools.getWorldFrame());
            
            originalSpatialVector.setIncludingFrame(randomAngularPart, randomLinearPart);
            assertTrue(originalSpatialVector.getReferenceFrame().equals(randomLinearPart.getReferenceFrame()));
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
  
      }
      
      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, and two Vector3Ds 
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);

         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);

         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertFalse(randomSpatialVector.getAngularPart().equals(randomAngularPart));
         assertFalse(randomSpatialVector.getLinearPart().equals(randomLinearPart));
         randomSpatialVector.setIncludingFrame(randomFrame, randomAngularPart, randomLinearPart);
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(randomFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(randomFrame));
         assertTrue(randomSpatialVector.getAngularPart().equals(randomAngularPart));
         assertTrue(randomSpatialVector.getLinearPart().equals(randomLinearPart));
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
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomVectorArray[0] == randomSpatialVector.getAngularPartX());
         assertTrue(randomVectorArray[1] == randomSpatialVector.getAngularPartY());
         assertTrue(randomVectorArray[2] == randomSpatialVector.getAngularPartZ());
         assertTrue(randomVectorArray[3] == randomSpatialVector.getLinearPartX());
         assertTrue(randomVectorArray[4] == randomSpatialVector.getLinearPartY());
         assertTrue(randomVectorArray[5] == randomSpatialVector.getLinearPartZ());
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
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomVectorArray[startingArrayIndex] == randomSpatialVector.getAngularPartX());
         assertTrue(randomVectorArray[startingArrayIndex+1] == randomSpatialVector.getAngularPartY());
         assertTrue(randomVectorArray[startingArrayIndex+2] == randomSpatialVector.getAngularPartZ());
         assertTrue(randomVectorArray[startingArrayIndex+3] == randomSpatialVector.getLinearPartX());
         assertTrue(randomVectorArray[startingArrayIndex+4] == randomSpatialVector.getLinearPartY());
         assertTrue(randomVectorArray[startingArrayIndex+5] == randomSpatialVector.getLinearPartZ());
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

         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));

         randomSpatialVector.setIncludingFrame(randomFrame, randomVectorArray);
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomVectorArray[0] == randomSpatialVector.getAngularPartX());
         assertTrue(randomVectorArray[1] == randomSpatialVector.getAngularPartY());
         assertTrue(randomVectorArray[2] == randomSpatialVector.getAngularPartZ());
         assertTrue(randomVectorArray[3] == randomSpatialVector.getLinearPartX());
         assertTrue(randomVectorArray[4] == randomSpatialVector.getLinearPartY());
         assertTrue(randomVectorArray[5] == randomSpatialVector.getLinearPartZ());
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
         
         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));

         int startingArrayIndex = i % 18;
         
         randomSpatialVector.setIncludingFrame(randomFrame, startingArrayIndex, randomVectorArray);
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomVectorArray[startingArrayIndex] == randomSpatialVector.getAngularPartX());
         assertTrue(randomVectorArray[startingArrayIndex+1] == randomSpatialVector.getAngularPartY());
         assertTrue(randomVectorArray[startingArrayIndex+2] == randomSpatialVector.getAngularPartZ());
         assertTrue(randomVectorArray[startingArrayIndex+3] == randomSpatialVector.getLinearPartX());
         assertTrue(randomVectorArray[startingArrayIndex+4] == randomSpatialVector.getLinearPartY());
         assertTrue(randomVectorArray[startingArrayIndex+5] == randomSpatialVector.getLinearPartZ());
      }
      
      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from a column DMatrix. 
      //The components of the DMatrix are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(6,1); 
         for (int row = 0; row < 6; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));

         randomSpatialVector.setIncludingFrame(randomFrame, randomMatrix);
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomMatrix.get(0,0) == randomSpatialVector.getAngularPartX());
         assertTrue(randomMatrix.get(1,0) == randomSpatialVector.getAngularPartY());
         assertTrue(randomMatrix.get(2,0) == randomSpatialVector.getAngularPartZ());
         assertTrue(randomMatrix.get(3,0) == randomSpatialVector.getLinearPartX());
         assertTrue(randomMatrix.get(4,0) == randomSpatialVector.getLinearPartY());
         assertTrue(randomMatrix.get(5,0) == randomSpatialVector.getLinearPartZ());
      }  
      
      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from a column DMatrix starting
      //at a specific row. The components of the DMatrix are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24,1); 
         for (int row = 0; row < 24; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));

         int startingMatrixRow = i % 18;

         randomSpatialVector.setIncludingFrame(randomFrame, startingMatrixRow, randomMatrix);
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomMatrix.get(startingMatrixRow,0) == randomSpatialVector.getAngularPartX());
         assertTrue(randomMatrix.get(startingMatrixRow+1,0) == randomSpatialVector.getAngularPartY());
         assertTrue(randomMatrix.get(startingMatrixRow+2,0) == randomSpatialVector.getAngularPartZ());
         assertTrue(randomMatrix.get(startingMatrixRow+3,0) == randomSpatialVector.getLinearPartX());
         assertTrue(randomMatrix.get(startingMatrixRow+4,0) == randomSpatialVector.getLinearPartY());
         assertTrue(randomMatrix.get(startingMatrixRow+5,0) == randomSpatialVector.getLinearPartZ());
      }  

      //Test setIncludingFrame - sets a spatial vector equal to a ReferenceFrame, sets the angular/linear parts from a 
      //starting column/row of a DMatrix. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24,24); 
         for (int row = 0; row < 24; row++)
         {
            for (int column = 0; column < 24; column++)
            {
               randomMatrix.set(row, column, random.nextDouble());
            }
         }

         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));

         int startingMatrixRow = i % 18;
         int startingMatrixColumn = i % 18;
         
         randomSpatialVector.setIncludingFrame(randomFrame, startingMatrixRow, startingMatrixColumn, randomMatrix);
         
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomMatrix.get(startingMatrixRow,startingMatrixColumn) == randomSpatialVector.getAngularPartX());
         assertTrue(randomMatrix.get(startingMatrixRow+1,startingMatrixColumn) == randomSpatialVector.getAngularPartY());
         assertTrue(randomMatrix.get(startingMatrixRow+2,startingMatrixColumn) == randomSpatialVector.getAngularPartZ());
         assertTrue(randomMatrix.get(startingMatrixRow+3,startingMatrixColumn) == randomSpatialVector.getLinearPartX());
         assertTrue(randomMatrix.get(startingMatrixRow+4,startingMatrixColumn) == randomSpatialVector.getLinearPartY());
         assertTrue(randomMatrix.get(startingMatrixRow+5,startingMatrixColumn) == randomSpatialVector.getLinearPartZ());
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
         
         assertFalse(spatialVector.getReferenceFrame().equals(randomFrame));
         spatialVector.setReferenceFrame(randomFrame);
         assertTrue(spatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(spatialVector.getAngularPart().getReferenceFrame().equals(randomFrame));
         assertTrue(spatialVector.getLinearPart().getReferenceFrame().equals(randomFrame));
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
         
         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         
         randomSpatialVector.setToZero(randomFrame);
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(randomFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(randomFrame));
         assertTrue(randomSpatialVector.getAngularPartX() == 0); 
         assertTrue(randomSpatialVector.getAngularPartY() == 0);
         assertTrue(randomSpatialVector.getAngularPartZ() == 0);
         assertTrue(randomSpatialVector.getLinearPartX() == 0);
         assertTrue(randomSpatialVector.getLinearPartY() == 0);
         assertTrue(randomSpatialVector.getLinearPartZ() == 0);
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
         
         assertFalse(randomSpatialVector.getReferenceFrame().equals(randomFrame));

         randomSpatialVector.setToNaN(randomFrame);
         
         assertTrue(randomSpatialVector.getReferenceFrame().equals(randomFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(randomFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(randomFrame));
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(randomSpatialVector.getAngularPart());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(randomSpatialVector.getLinearPart());
      }         
   }
   
}