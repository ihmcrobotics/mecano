package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

public abstract class FixedFrameSpatialVectorBasicsTest<T extends FixedFrameSpatialVectorBasics> extends SpatialVectorReadOnlyTest<SpatialVectorReadOnly>
{
   public abstract T newEmptySpatialVector();
   
   public abstract T newRandomSpatialVector(Random random);
   
   public abstract T newSpatialVector(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart);
   
   public abstract T newCopySpatialVector(SpatialVectorReadOnly other);
   
   public abstract void testSetMatchingFrame();
   
   public abstract double getEpsilon();
   
   private static final int ITERATIONS = 1000;
   
   @Test
   public void testSetToZeroFixedFrame()
   {
      Random random = new Random(21623);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();
         
         randomSpatialVector.setToZero();
         
         //verify Angular and Linear parts are set to zero and verify Reference Frame did not change
         assertTrue(randomSpatialVector.getAngularPartX() == 0); 
         assertTrue(randomSpatialVector.getAngularPartY() == 0);
         assertTrue(randomSpatialVector.getAngularPartZ() == 0);
         assertTrue(randomSpatialVector.getLinearPartX() == 0);
         assertTrue(randomSpatialVector.getLinearPartY() == 0);
         assertTrue(randomSpatialVector.getLinearPartZ() == 0);
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      }      
   }
   
   @Test
   public void testSetToNaNFixedFrame()
   {
      //Test SetToNan - sets all the components of both vectors to NaN 
      Random random = new Random(31843);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();
         
         randomSpatialVector.setToNaN();
         
         //verify Angular and Linear parts are set to NaN and verify Reference Frame did not change
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(randomSpatialVector.getAngularPart());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(randomSpatialVector.getLinearPart());
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      }         
   }
   
   @Test
   public void testContainsNaN() throws Exception
   {
      //Test SetToNan - sets all the components of both vectors to NaN 
      Random random = new Random(31843);
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         
         FixedFrameVector3DBasics originalAngularPart = randomSpatialVector.getAngularPart();
         FixedFrameVector3DBasics originalLinearPart =  randomSpatialVector.getAngularPart();
      
         assertFalse(randomSpatialVector.containsNaN());
    
         //Test containsNan method
         randomSpatialVector.setAngularPartX(Double.NaN);
         assertTrue(randomSpatialVector.containsNaN());
      
         randomSpatialVector.setAngularPartX(originalAngularPart.getX());
         randomSpatialVector.setAngularPartY(Double.NaN);
         assertTrue(randomSpatialVector.containsNaN());

         randomSpatialVector.setAngularPartY(originalAngularPart.getY());
         randomSpatialVector.setAngularPartZ(Double.NaN);
         assertTrue(randomSpatialVector.containsNaN());
      
         randomSpatialVector.setAngularPartZ(originalAngularPart.getZ());
         randomSpatialVector.setLinearPartX(Double.NaN);
         assertTrue(randomSpatialVector.containsNaN());
      
         randomSpatialVector.setLinearPartX(originalLinearPart.getX());
         randomSpatialVector.setLinearPartY(Double.NaN);
         assertTrue(randomSpatialVector.containsNaN());

         randomSpatialVector.setLinearPartY(originalLinearPart.getY());
         randomSpatialVector.setLinearPartZ(Double.NaN);
         assertTrue(randomSpatialVector.containsNaN());
      }
   }
   
   @Test
   public void testSetElement() throws Exception
   {
      Random random = new Random(31843);
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         
         double randomDouble = random.nextDouble();
         randomSpatialVector.setElement(0, randomDouble);
         assertTrue(randomSpatialVector.getAngularPartX() == randomDouble);
         
         randomDouble = random.nextDouble();
         randomSpatialVector.setElement(1, randomDouble);
         assertTrue(randomSpatialVector.getAngularPartY() == randomDouble);
         
         randomDouble = random.nextDouble();
         randomSpatialVector.setElement(2, randomDouble);
         assertTrue(randomSpatialVector.getAngularPartZ() == randomDouble);

         randomDouble = random.nextDouble();
         randomSpatialVector.setElement(3, randomDouble);
         assertTrue(randomSpatialVector.getLinearPartX() == randomDouble);

         randomDouble = random.nextDouble();
         randomSpatialVector.setElement(4, randomDouble);
         assertTrue(randomSpatialVector.getLinearPartY() == randomDouble);
         
         randomDouble = random.nextDouble();
         randomSpatialVector.setElement(5, randomDouble);
         assertTrue(randomSpatialVector.getLinearPartZ() == randomDouble);
         
         //Test setElement with an index greater than 5 fails
         try
         {
            randomSpatialVector.setElement(i+6, randomDouble);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }

      }
   }
   
   @Test
   public void testSettersFixedFrame()
   {
      //Test set methods
      
      Random random = new Random(21623);
      
      //Test set the angular/linear parts from an array. 
      //The components of the array are read in the following order: angularPartX, angularPartY, angularPartZ, linearPartX, 
      //linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();
         
         double[] randomVectorArray = new double[6]; 
         for (int index = 0; index < 6; index++)
            randomVectorArray[index] = random.nextDouble();

         randomSpatialVector.set(randomVectorArray);
         assertTrue(randomVectorArray[0] == randomSpatialVector.getAngularPartX());
         assertTrue(randomVectorArray[1] == randomSpatialVector.getAngularPartY());
         assertTrue(randomVectorArray[2] == randomSpatialVector.getAngularPartZ());
         assertTrue(randomVectorArray[3] == randomSpatialVector.getLinearPartX());
         assertTrue(randomVectorArray[4] == randomSpatialVector.getLinearPartY());
         assertTrue(randomVectorArray[5] == randomSpatialVector.getLinearPartZ());
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      }
      
      //Test sets the angular/linear parts from an array with a starting point in the array. The components of the array are read in the following 
      //order: angularPartX, angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();

         double[] randomVectorArray = new double[24]; 
         for (int index = 0; index < 24; index++)
            randomVectorArray[index] = random.nextDouble();
         
         int startingArrayIndex = i % 18;
         
         randomSpatialVector.set(startingArrayIndex, randomVectorArray);
         assertTrue(randomVectorArray[startingArrayIndex] == randomSpatialVector.getAngularPartX());
         assertTrue(randomVectorArray[startingArrayIndex+1] == randomSpatialVector.getAngularPartY());
         assertTrue(randomVectorArray[startingArrayIndex+2] == randomSpatialVector.getAngularPartZ());
         assertTrue(randomVectorArray[startingArrayIndex+3] == randomSpatialVector.getLinearPartX());
         assertTrue(randomVectorArray[startingArrayIndex+4] == randomSpatialVector.getLinearPartY());
         assertTrue(randomVectorArray[startingArrayIndex+5] == randomSpatialVector.getLinearPartZ());
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      }
      
      //Test sets the angular/linear parts from an float array. The components of the array are read in the following order: angularPartX, angularPartY, 
      //angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();

         float[] randomVectorArray = new float[24]; 
         for (int index = 0; index < 6; index++)
            randomVectorArray[index] = random.nextFloat();

         randomSpatialVector.set(randomVectorArray);
         assertTrue(randomVectorArray[0] == randomSpatialVector.getAngularPartX());
         assertTrue(randomVectorArray[1] == randomSpatialVector.getAngularPartY());
         assertTrue(randomVectorArray[2] == randomSpatialVector.getAngularPartZ());
         assertTrue(randomVectorArray[3] == randomSpatialVector.getLinearPartX());
         assertTrue(randomVectorArray[4] == randomSpatialVector.getLinearPartY());
         assertTrue(randomVectorArray[5] == randomSpatialVector.getLinearPartZ());
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      } 
      
      //Test sets the angular/linear parts from a float array with a starting point in the array. The components of the array are read in the following 
      //order: angularPartX, angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();
 
         float[] randomVectorArray = new float[24]; 
         for (int index = 0; index < 24; index++)
            randomVectorArray[index] = random.nextFloat();
         
          int startingArrayIndex = i % 18;
         
         randomSpatialVector.set(startingArrayIndex, randomVectorArray);
         assertTrue(randomVectorArray[startingArrayIndex] == randomSpatialVector.getAngularPartX());
         assertTrue(randomVectorArray[startingArrayIndex+1] == randomSpatialVector.getAngularPartY());
         assertTrue(randomVectorArray[startingArrayIndex+2] == randomSpatialVector.getAngularPartZ());
         assertTrue(randomVectorArray[startingArrayIndex+3] == randomSpatialVector.getLinearPartX());
         assertTrue(randomVectorArray[startingArrayIndex+4] == randomSpatialVector.getLinearPartY());
         assertTrue(randomVectorArray[startingArrayIndex+5] == randomSpatialVector.getLinearPartZ());
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      }

      //Test sets the angular/linear parts from a column DMatrix. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();

         DMatrixRMaj randomMatrix = new DMatrixRMaj(6,1); 
         for (int row = 0; row < 6; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         randomSpatialVector.set(randomMatrix);
         assertTrue(randomMatrix.get(0,0) == randomSpatialVector.getAngularPartX());
         assertTrue(randomMatrix.get(1,0) == randomSpatialVector.getAngularPartY());
         assertTrue(randomMatrix.get(2,0) == randomSpatialVector.getAngularPartZ());
         assertTrue(randomMatrix.get(3,0) == randomSpatialVector.getLinearPartX());
         assertTrue(randomMatrix.get(4,0) == randomSpatialVector.getLinearPartY());
         assertTrue(randomMatrix.get(5,0) == randomSpatialVector.getLinearPartZ());
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      }
      
      //Test sets the angular/linear parts from a column DMatrix starting at a specific row. The components of the DMatrix are read in the following 
      //order: angularPartX, angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();

         DMatrixRMaj randomMatrix = new DMatrixRMaj(24,1); 
         for (int row = 0; row < 24; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         int startingMatrixRow = i % 18;

         randomSpatialVector.set(startingMatrixRow, randomMatrix);
         assertTrue(randomMatrix.get(startingMatrixRow,0) == randomSpatialVector.getAngularPartX());
         assertTrue(randomMatrix.get(startingMatrixRow+1,0) == randomSpatialVector.getAngularPartY());
         assertTrue(randomMatrix.get(startingMatrixRow+2,0) == randomSpatialVector.getAngularPartZ());
         assertTrue(randomMatrix.get(startingMatrixRow+3,0) == randomSpatialVector.getLinearPartX());
         assertTrue(randomMatrix.get(startingMatrixRow+4,0) == randomSpatialVector.getLinearPartY());
         assertTrue(randomMatrix.get(startingMatrixRow+5,0) == randomSpatialVector.getLinearPartZ());
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      }  
      
      //Test sets the angular/linear parts from a starting column/row of a DMatrix. The components of the DMatrix are read in the following order: 
      //angularPartX, angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = randomSpatialVector.getReferenceFrame();

         DMatrixRMaj randomMatrix = new DMatrixRMaj(24,24); 
         for (int row = 0; row < 24; row++)
         {
            for (int column = 0; column < 24; column++)
            {
               randomMatrix.set(row, column, random.nextDouble());
            }
         }

         int startingMatrixRow = i % 18;
         int startingMatrixColumn = Math.abs(random.nextInt()) % 18;
         
         randomSpatialVector.set(startingMatrixRow, startingMatrixColumn, randomMatrix);
         
         assertTrue(randomMatrix.get(startingMatrixRow,startingMatrixColumn) == randomSpatialVector.getAngularPartX());
         assertTrue(randomMatrix.get(startingMatrixRow+1,startingMatrixColumn) == randomSpatialVector.getAngularPartY());
         assertTrue(randomMatrix.get(startingMatrixRow+2,startingMatrixColumn) == randomSpatialVector.getAngularPartZ());
         assertTrue(randomMatrix.get(startingMatrixRow+3,startingMatrixColumn) == randomSpatialVector.getLinearPartX());
         assertTrue(randomMatrix.get(startingMatrixRow+4,startingMatrixColumn) == randomSpatialVector.getLinearPartY());
         assertTrue(randomMatrix.get(startingMatrixRow+5,startingMatrixColumn) == randomSpatialVector.getLinearPartZ());
         assertTrue(randomSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(randomSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      } 
      
      //Test set given an angular part and linear part, updates the frame of this vector
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         ReferenceFrame saveReferenceFrame = originalSpatialVector.getReferenceFrame();
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);

         originalSpatialVector.set(randomAngularPart,randomLinearPart);
         
         assertTrue(originalSpatialVector.getAngularPart().equals(randomAngularPart));
         assertTrue(originalSpatialVector.getLinearPart().equals(randomLinearPart));
         assertTrue(originalSpatialVector.getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(originalSpatialVector.getAngularPart().getReferenceFrame().equals(saveReferenceFrame));
         assertTrue(originalSpatialVector.getLinearPart().getReferenceFrame().equals(saveReferenceFrame));
      }
      
      //Test sets a spatial vector equal to another spatial vector; however, they both have to have the same Reference Frame or an exception is thrown.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T randomSpatialVectorDifferentReferenceFrame = newRandomSpatialVector(random);

         assertFalse(originalSpatialVector.equals(randomSpatialVectorDifferentReferenceFrame));
         
         //Different Reference Frames throws an exception
         try
         {
            originalSpatialVector.set(randomSpatialVectorDifferentReferenceFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
         
         //Same Reference Frame different angular/linear parts
         T randomSpatialVectorSameReferenceFrame = newSpatialVector(originalSpatialVector.getReferenceFrame(),randomSpatialVectorDifferentReferenceFrame.getAngularPart(), randomSpatialVectorDifferentReferenceFrame.getLinearPart());
         originalSpatialVector.set(randomSpatialVectorSameReferenceFrame);
         assertTrue(originalSpatialVector.equals(randomSpatialVectorSameReferenceFrame));
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, originalSpatialVector.getReferenceFrame());
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, originalSpatialVector.getReferenceFrame());
         
         originalSpatialVector.set(randomAngularPart, randomLinearPart);
         assertTrue(originalSpatialVector.getAngularPart().equals(randomAngularPart));
         assertTrue(originalSpatialVector.getLinearPart().equals(randomLinearPart));

         
         //Different Reference Frames throws an exception
         try
         {
            ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            randomAngularPart.setReferenceFrame(randomFrame);
            originalSpatialVector.set(randomAngularPart, randomLinearPart);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
      
      }
      
      //Test set with a Reference Frame, angular part, and linear part. The reference frame has to be the same as the original
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, originalSpatialVector.getReferenceFrame());
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, originalSpatialVector.getReferenceFrame());
         
         originalSpatialVector.set(originalSpatialVector.getReferenceFrame(), randomAngularPart, randomLinearPart);
         assertTrue(originalSpatialVector.getAngularPart().equals(randomAngularPart));
         assertTrue(originalSpatialVector.getLinearPart().equals(randomLinearPart));

         
         //Different Reference Frames throws an exception
         try
         {
            ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
            randomAngularPart.setReferenceFrame(randomFrame);
            randomLinearPart.setReferenceFrame(randomFrame);
            originalSpatialVector.set(randomFrame, randomAngularPart, randomLinearPart);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
      
      }
      
   }
   
   @Test
   public void testScale()
   {
      Random random = new Random(21623);
  
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         double randomDouble = random.nextDouble();

         assertTrue(originalSpatialVector.equals(expectedSpatialVector));
         expectedSpatialVector.scale(randomDouble);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() * randomDouble, getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() * randomDouble, getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() * randomDouble, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() * randomDouble, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() * randomDouble, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() * randomDouble, getEpsilon());
      }
   }
   
   @Test
   public void testNegate()
   {
      Random random = new Random(21623);
  
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         assertTrue(originalSpatialVector.equals(expectedSpatialVector));
         expectedSpatialVector.negate();;
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() * -1, getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() * -1, getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() * -1, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() * -1, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() * -1, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() * -1, getEpsilon());
      }
   }
   
   @Test
   public void testNormalize()
   {
      //Test normalizing a vector such that its magnitude is equal to 1 after calling this method and its
      // direction remains unchanged.
      Random random = new Random(21623);
  
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         double expectedScaleAfterNormalization = 1.0 / originalSpatialVector.length();

         assertTrue(expectedSpatialVector.equals(expectedSpatialVector));
         
         expectedSpatialVector.normalize();

         //After normalization the magnitude should equal to 1 and each x, y, z is scaled to 1 / length of the Spatial Vector
         assertEquals(expectedSpatialVector.length(),1,getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() * expectedScaleAfterNormalization, getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() * expectedScaleAfterNormalization, getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() * expectedScaleAfterNormalization, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() * expectedScaleAfterNormalization, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() * expectedScaleAfterNormalization, getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() * expectedScaleAfterNormalization, getEpsilon());
      }
   }
   
   @Test
   public void testAdds()
   {
      Random random = new Random(21723);
      
      //Test add given vector to a vector and throws an exception if reference frames are different
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T addSpatialVectorSameReferenceFrame = newSpatialVector(originalSpatialVector.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         expectedSpatialVector.add(addSpatialVectorSameReferenceFrame);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() + addSpatialVectorSameReferenceFrame.getAngularPartX(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() + addSpatialVectorSameReferenceFrame.getAngularPartY(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() + addSpatialVectorSameReferenceFrame.getAngularPartZ(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() + addSpatialVectorSameReferenceFrame.getLinearPartX(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() + addSpatialVectorSameReferenceFrame.getLinearPartY(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() + addSpatialVectorSameReferenceFrame.getLinearPartZ(), getEpsilon());

         //Verify Different Reference Frame throws an exception
         try
         {
            T addSpatialVectorDifferentReferenceFrame = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));

            expectedSpatialVector.add(addSpatialVectorDifferentReferenceFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
         
      }
      
      //Test setting a vector to the sum of two given vectors and throws an exception if reference frames are different
      for (int i = 0; i < ITERATIONS; i++)
      {
         T firstSpatialVector = newRandomSpatialVector(random);
         T secondSpatialVectorSameReferenceFrame = newSpatialVector(firstSpatialVector.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         T newSpatialVector = newSpatialVector(firstSpatialVector.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         
         newSpatialVector.add(firstSpatialVector, secondSpatialVectorSameReferenceFrame);
         
         assertEquals(newSpatialVector.getAngularPartX(), firstSpatialVector.getAngularPartX() + secondSpatialVectorSameReferenceFrame.getAngularPartX(), getEpsilon());
         assertEquals(newSpatialVector.getAngularPartY(), firstSpatialVector.getAngularPartY() + secondSpatialVectorSameReferenceFrame.getAngularPartY(), getEpsilon());
         assertEquals(newSpatialVector.getAngularPartZ(), firstSpatialVector.getAngularPartZ() + secondSpatialVectorSameReferenceFrame.getAngularPartZ(), getEpsilon());
         assertEquals(newSpatialVector.getLinearPartX(), firstSpatialVector.getLinearPartX() + secondSpatialVectorSameReferenceFrame.getLinearPartX(), getEpsilon());
         assertEquals(newSpatialVector.getLinearPartY(), firstSpatialVector.getLinearPartY() + secondSpatialVectorSameReferenceFrame.getLinearPartY(), getEpsilon());
         assertEquals(newSpatialVector.getLinearPartZ(), firstSpatialVector.getLinearPartZ() + secondSpatialVectorSameReferenceFrame.getLinearPartZ(), getEpsilon());

         //Verify Different Reference Frame throws an exception
         try
         {
            T secondSpatialVectorDifferentReferenceFrame = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));

            newSpatialVector.add(firstSpatialVector, secondSpatialVectorDifferentReferenceFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
         
      }
      
      //Test adding a column matrix to a vector. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(6,1); 
         for (int row = 0; row < 6; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }
         
         expectedSpatialVector.add(randomMatrix);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() + randomMatrix.get(0,0), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() + randomMatrix.get(1,0), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() + randomMatrix.get(2,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() + randomMatrix.get(3,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() + randomMatrix.get(4,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() + randomMatrix.get(5,0), getEpsilon());
      }
      
      //Test adding a column matrix to a vector starting at a specific row. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24,1); 
         for (int row = 0; row < 24; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         int startingMatrixRow = i % 18;
         
         expectedSpatialVector.add(startingMatrixRow, randomMatrix);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() + randomMatrix.get(startingMatrixRow,0), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() + randomMatrix.get(startingMatrixRow + 1,0), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() + randomMatrix.get(startingMatrixRow + 2,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() + randomMatrix.get(startingMatrixRow + 3,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() + randomMatrix.get(startingMatrixRow + 4,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() + randomMatrix.get(startingMatrixRow + 5,0), getEpsilon());
      }
      
      //Test adding a column matrix to a vector starting at a specific row/column. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24,24); 
         for (int row = 0; row < 24; row++)
         {
            for (int column = 0; column < 24; column++)
            {
               randomMatrix.set(row, column, random.nextDouble());
            }
         }

         int startingMatrixRow = i % 18;
         int startingMatrixColumn = i % 18;
         
         expectedSpatialVector.add(startingMatrixRow, startingMatrixColumn, randomMatrix);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() + randomMatrix.get(startingMatrixRow,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() + randomMatrix.get(startingMatrixRow + 1,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() + randomMatrix.get(startingMatrixRow + 2,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() + randomMatrix.get(startingMatrixRow + 3,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() + randomMatrix.get(startingMatrixRow + 4,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() + randomMatrix.get(startingMatrixRow + 5,startingMatrixColumn), getEpsilon());
      }
      
      //Test adding FrameVectors to a spatial vector.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         FrameVector3D randomAngularPartSameReferenceFrame = EuclidFrameRandomTools.nextFrameVector3D(random, originalSpatialVector.getReferenceFrame());
         FrameVector3D randomLinearPartSameReferenceFrame = EuclidFrameRandomTools.nextFrameVector3D(random, originalSpatialVector.getReferenceFrame());
         
         expectedSpatialVector.add(randomAngularPartSameReferenceFrame, randomLinearPartSameReferenceFrame);

         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() + randomAngularPartSameReferenceFrame.getX(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() + randomAngularPartSameReferenceFrame.getY(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() + randomAngularPartSameReferenceFrame.getZ(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() + randomLinearPartSameReferenceFrame.getX(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() + randomLinearPartSameReferenceFrame.getY(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() + randomLinearPartSameReferenceFrame.getZ(), getEpsilon());
         
         //Verify Different Reference Frame throws an exception
         try
         {
            FrameVector3D randomAngularPartDifferentReferenceFrame = EuclidFrameRandomTools.nextFrameVector3D(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            FrameVector3D randomLinearPartDifferentReferenceFrame = EuclidFrameRandomTools.nextFrameVector3D(random, EuclidFrameRandomTools.nextReferenceFrame(random));

            expectedSpatialVector.add(randomAngularPartDifferentReferenceFrame, randomLinearPartDifferentReferenceFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
      }
   }
   
   @Test
   public void testSubs()
   {
      Random random = new Random(21723);
      
      //Test add given vector to a vector and throws an exception if reference frames are different
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T subSpatialVectorSameReferenceFrame = newSpatialVector(originalSpatialVector.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         expectedSpatialVector.sub(subSpatialVectorSameReferenceFrame);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() - subSpatialVectorSameReferenceFrame.getAngularPartX(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() - subSpatialVectorSameReferenceFrame.getAngularPartY(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() - subSpatialVectorSameReferenceFrame.getAngularPartZ(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() - subSpatialVectorSameReferenceFrame.getLinearPartX(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() - subSpatialVectorSameReferenceFrame.getLinearPartY(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() - subSpatialVectorSameReferenceFrame.getLinearPartZ(), getEpsilon());

         //Verify Different Reference Frame throws an exception
         try
         {
            T subSpatialVectorDifferentReferenceFrame = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));

            expectedSpatialVector.sub(subSpatialVectorDifferentReferenceFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
         
      }
      
      //Test setting a vector to the sum of two given vectors and throws an exception if reference frames are different
      for (int i = 0; i < ITERATIONS; i++)
      {
         T firstSpatialVector = newRandomSpatialVector(random);
         T secondSpatialVectorSameReferenceFrame = newSpatialVector(firstSpatialVector.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         T newSpatialVector = newSpatialVector(firstSpatialVector.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         
         newSpatialVector.sub(firstSpatialVector, secondSpatialVectorSameReferenceFrame);
         
         assertEquals(newSpatialVector.getAngularPartX(), firstSpatialVector.getAngularPartX() - secondSpatialVectorSameReferenceFrame.getAngularPartX(), getEpsilon());
         assertEquals(newSpatialVector.getAngularPartY(), firstSpatialVector.getAngularPartY() - secondSpatialVectorSameReferenceFrame.getAngularPartY(), getEpsilon());
         assertEquals(newSpatialVector.getAngularPartZ(), firstSpatialVector.getAngularPartZ() - secondSpatialVectorSameReferenceFrame.getAngularPartZ(), getEpsilon());
         assertEquals(newSpatialVector.getLinearPartX(), firstSpatialVector.getLinearPartX() - secondSpatialVectorSameReferenceFrame.getLinearPartX(), getEpsilon());
         assertEquals(newSpatialVector.getLinearPartY(), firstSpatialVector.getLinearPartY() - secondSpatialVectorSameReferenceFrame.getLinearPartY(), getEpsilon());
         assertEquals(newSpatialVector.getLinearPartZ(), firstSpatialVector.getLinearPartZ() - secondSpatialVectorSameReferenceFrame.getLinearPartZ(), getEpsilon());

         //Verify Different Reference Frame throws an exception
         try
         {
            T secondSpatialVectorDifferentReferenceFrame = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));

            newSpatialVector.sub(firstSpatialVector, secondSpatialVectorDifferentReferenceFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
         
      }
      
      //Test adding a column matrix to a vector. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(6,1); 
         for (int row = 0; row < 6; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }
         
         expectedSpatialVector.sub(randomMatrix);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() - randomMatrix.get(0,0), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() - randomMatrix.get(1,0), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() - randomMatrix.get(2,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() - randomMatrix.get(3,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() - randomMatrix.get(4,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() - randomMatrix.get(5,0), getEpsilon());
      }
      
      //Test adding a column matrix to a vector starting at a specific row. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24,1); 
         for (int row = 0; row < 24; row++)
         {
            randomMatrix.set(row, 0, random.nextDouble());
         }

         int startingMatrixRow = i % 18;
         
         expectedSpatialVector.sub(startingMatrixRow, randomMatrix);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() - randomMatrix.get(startingMatrixRow,0), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() - randomMatrix.get(startingMatrixRow + 1,0), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() - randomMatrix.get(startingMatrixRow + 2,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() - randomMatrix.get(startingMatrixRow + 3,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() - randomMatrix.get(startingMatrixRow + 4,0), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() - randomMatrix.get(startingMatrixRow + 5,0), getEpsilon());
      }
      
      //Test adding a column matrix to a vector starting at a specific row/column. The components of the DMatrix are read in the following order: angularPartX, 
      //angularPartY, angularPartZ, linearPartX, linearPartY, linearPartZ
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         DMatrixRMaj randomMatrix = new DMatrixRMaj(24,24); 
         for (int row = 0; row < 24; row++)
         {
            for (int column = 0; column < 24; column++)
            {
               randomMatrix.set(row, column, random.nextDouble());
            }
         }

         int startingMatrixRow = i % 18;
         int startingMatrixColumn = i % 18;
         
         expectedSpatialVector.sub(startingMatrixRow, startingMatrixColumn, randomMatrix);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() - randomMatrix.get(startingMatrixRow,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() - randomMatrix.get(startingMatrixRow + 1,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() - randomMatrix.get(startingMatrixRow + 2,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() - randomMatrix.get(startingMatrixRow + 3,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() - randomMatrix.get(startingMatrixRow + 4,startingMatrixColumn), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() - randomMatrix.get(startingMatrixRow + 5,startingMatrixColumn), getEpsilon());
      }
      
      //Test adding FrameVectors to a spatial vector.
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         FrameVector3D randomAngularPartSameReferenceFrame = EuclidFrameRandomTools.nextFrameVector3D(random, originalSpatialVector.getReferenceFrame());
         FrameVector3D randomLinearPartSameReferenceFrame = EuclidFrameRandomTools.nextFrameVector3D(random, originalSpatialVector.getReferenceFrame());
         
         expectedSpatialVector.sub(randomAngularPartSameReferenceFrame, randomLinearPartSameReferenceFrame);

         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() - randomAngularPartSameReferenceFrame.getX(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() - randomAngularPartSameReferenceFrame.getY(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() - randomAngularPartSameReferenceFrame.getZ(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() - randomLinearPartSameReferenceFrame.getX(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() - randomLinearPartSameReferenceFrame.getY(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() - randomLinearPartSameReferenceFrame.getZ(), getEpsilon());
         
         //Verify Different Reference Frame throws an exception
         try
         {
            FrameVector3D randomAngularPartDifferentReferenceFrame = EuclidFrameRandomTools.nextFrameVector3D(random, EuclidFrameRandomTools.nextReferenceFrame(random));
            FrameVector3D randomLinearPartDifferentReferenceFrame = EuclidFrameRandomTools.nextFrameVector3D(random, EuclidFrameRandomTools.nextReferenceFrame(random));

            expectedSpatialVector.sub(randomAngularPartDifferentReferenceFrame, randomLinearPartDifferentReferenceFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
      }
   }
   
   @Test
   public void testInterpolate()
   {
      Random random = new Random(21723);
      
      //Test linear interpolation from this vector to given percentage. Result should be this = (1.0 - alpha) * this + alpha * other
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T interpolateSpatialVectorSameReferenceFrame = newSpatialVector(originalSpatialVector.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         double randomDouble = random.nextDouble();
         
         expectedSpatialVector.interpolate(interpolateSpatialVectorSameReferenceFrame, randomDouble);

         assertEquals(expectedSpatialVector.getAngularPartX(), (1.0 - randomDouble) * originalSpatialVector.getAngularPartX() + randomDouble * interpolateSpatialVectorSameReferenceFrame.getAngularPartX(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), (1.0 - randomDouble) * originalSpatialVector.getAngularPartY() + randomDouble * interpolateSpatialVectorSameReferenceFrame.getAngularPartY(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), (1.0 - randomDouble) * originalSpatialVector.getAngularPartZ() + randomDouble * interpolateSpatialVectorSameReferenceFrame.getAngularPartZ(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), (1.0 - randomDouble) * originalSpatialVector.getLinearPartX() + randomDouble * interpolateSpatialVectorSameReferenceFrame.getLinearPartX(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), (1.0 - randomDouble) * originalSpatialVector.getLinearPartY() + randomDouble * interpolateSpatialVectorSameReferenceFrame.getLinearPartY(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), (1.0 - randomDouble) * originalSpatialVector.getLinearPartZ() + randomDouble * interpolateSpatialVectorSameReferenceFrame.getLinearPartZ(), getEpsilon());

         //Verify Different Reference Frame throws an exception
         try
         {
            T interpolateSpatialVectorDifferentReferenceFrame = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));

            expectedSpatialVector.interpolate(interpolateSpatialVectorDifferentReferenceFrame,randomDouble);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
      }  
      
      //Test linear interpolation from 2 vectors to given percentage. Result should be this = (1.0 - alpha) * vector1 + alpha * vector2
      for (int i = 0; i < ITERATIONS; i++)
      {
         T firstInterpolateSpatialVectorSameReferenceFrame = newRandomSpatialVector(random);
         T secondInterpolateSpatialVectorSameReferenceFrame = newSpatialVector(firstInterpolateSpatialVectorSameReferenceFrame.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         T expectedSpatialVector = newSpatialVector(firstInterpolateSpatialVectorSameReferenceFrame.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         double randomDouble = random.nextDouble();
         
         expectedSpatialVector.interpolate(firstInterpolateSpatialVectorSameReferenceFrame, secondInterpolateSpatialVectorSameReferenceFrame, randomDouble);

         assertEquals(expectedSpatialVector.getAngularPartX(), (1.0 - randomDouble) * firstInterpolateSpatialVectorSameReferenceFrame.getAngularPartX() + randomDouble * secondInterpolateSpatialVectorSameReferenceFrame.getAngularPartX(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), (1.0 - randomDouble) * firstInterpolateSpatialVectorSameReferenceFrame.getAngularPartY() + randomDouble * secondInterpolateSpatialVectorSameReferenceFrame.getAngularPartY(), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), (1.0 - randomDouble) * firstInterpolateSpatialVectorSameReferenceFrame.getAngularPartZ() + randomDouble * secondInterpolateSpatialVectorSameReferenceFrame.getAngularPartZ(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartX(), (1.0 - randomDouble) * firstInterpolateSpatialVectorSameReferenceFrame.getLinearPartX() + randomDouble * secondInterpolateSpatialVectorSameReferenceFrame.getLinearPartX(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), (1.0 - randomDouble) * firstInterpolateSpatialVectorSameReferenceFrame.getLinearPartY() + randomDouble * secondInterpolateSpatialVectorSameReferenceFrame.getLinearPartY(), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), (1.0 - randomDouble) * firstInterpolateSpatialVectorSameReferenceFrame.getLinearPartZ() + randomDouble * secondInterpolateSpatialVectorSameReferenceFrame.getLinearPartZ(), getEpsilon());

         //Verify Different Reference Frame throws an exception
         try
         {
            T interpolateSpatialVectorDifferentReferenceFrame = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));

            expectedSpatialVector.interpolate(interpolateSpatialVectorDifferentReferenceFrame, secondInterpolateSpatialVectorSameReferenceFrame, randomDouble);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
      }  
   }
   
   @Test
   public void testAddCrossToAngularPart()
   {
      Random random = new Random(21723);
      
      //Test cross product of tuple1 and tuple2 and adds result to vector's angular part. 
      //Result should be this.angularPart + tuple1 <cross> tuple2
      // The cross product is as follows:
      // x = tuple1.getY() * tuple2.getZ() - tuple1.getZ() * tuple2.getY();
      // y = tuple1.getZ() * tuple2.getX() - tuple1.getX() * tuple2.getZ();
      // z = tuple1.getX() * tuple2.getY() - tuple1.getY() * tuple2.getX();
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         Tuple3DReadOnly firstTuple3D = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly secondTuple3D = EuclidCoreRandomTools.nextPoint3D(random);
         
         expectedSpatialVector.addCrossToAngularPart(firstTuple3D, secondTuple3D);
         
         assertEquals(expectedSpatialVector.getAngularPartX(), originalSpatialVector.getAngularPartX() + (firstTuple3D.getY() * secondTuple3D.getZ() - firstTuple3D.getZ() * secondTuple3D.getY()), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartY(), originalSpatialVector.getAngularPartY() + (firstTuple3D.getZ() * secondTuple3D.getX() - firstTuple3D.getX() * secondTuple3D.getZ()), getEpsilon());
         assertEquals(expectedSpatialVector.getAngularPartZ(), originalSpatialVector.getAngularPartZ() + (firstTuple3D.getX() * secondTuple3D.getY() - firstTuple3D.getY() * secondTuple3D.getX()), getEpsilon());
      }
   }
   
   @Test
   public void testAddCrossToLinearPart()
   {
      Random random = new Random(24213);
      
      //Test cross product of tuple1 and tuple2 and adds result to vector's angular part. 
      //Result should be this.angularPart + tuple1 <cross> tuple2
      // The cross product is as follows:
      // x = tuple1.getY() * tuple2.getZ() - tuple1.getZ() * tuple2.getY();
      // y = tuple1.getZ() * tuple2.getX() - tuple1.getX() * tuple2.getZ();
      // z = tuple1.getX() * tuple2.getY() - tuple1.getY() * tuple2.getX();
      for (int i = 0; i < ITERATIONS; i++)
      {
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         Tuple3DReadOnly firstTuple3D = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly secondTuple3D = EuclidCoreRandomTools.nextPoint3D(random);
         
         expectedSpatialVector.addCrossToLinearPart(firstTuple3D, secondTuple3D);
         
         assertEquals(expectedSpatialVector.getLinearPartX(), originalSpatialVector.getLinearPartX() + (firstTuple3D.getY() * secondTuple3D.getZ() - firstTuple3D.getZ() * secondTuple3D.getY()), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartY(), originalSpatialVector.getLinearPartY() + (firstTuple3D.getZ() * secondTuple3D.getX() - firstTuple3D.getX() * secondTuple3D.getZ()), getEpsilon());
         assertEquals(expectedSpatialVector.getLinearPartZ(), originalSpatialVector.getLinearPartZ() + (firstTuple3D.getX() * secondTuple3D.getY() - firstTuple3D.getY() * secondTuple3D.getX()), getEpsilon());
      }
   }
   
   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(25613L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         assertEquals(expectedSpatialVector, originalSpatialVector);

         transform.transform(expectedSpatialVector.getAngularPart());
         transform.transform(expectedSpatialVector.getLinearPart());

         assertFalse(expectedSpatialVector.equals(originalSpatialVector));

         originalSpatialVector.applyTransform(transform);
         assertEquals(expectedSpatialVector, originalSpatialVector);
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         assertEquals(expectedSpatialVector, originalSpatialVector);

         transform.transform(expectedSpatialVector.getAngularPart());
         transform.transform(expectedSpatialVector.getLinearPart());
         assertFalse(expectedSpatialVector.equals(originalSpatialVector));

         originalSpatialVector.applyTransform(transform);
         assertEquals(expectedSpatialVector, originalSpatialVector);
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         expectedSpatialVector.set(originalSpatialVector);
         assertEquals(expectedSpatialVector, originalSpatialVector);

         transform.transform(expectedSpatialVector.getAngularPart());
         transform.transform(expectedSpatialVector.getLinearPart());
         assertFalse(expectedSpatialVector.equals(originalSpatialVector));

         originalSpatialVector.applyTransform(transform);
         assertEquals(expectedSpatialVector, originalSpatialVector);
      }
   }
   
   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         expectedSpatialVector.set(originalSpatialVector);
         assertEquals(originalSpatialVector, expectedSpatialVector);
         
         expectedSpatialVector.applyTransform(transform);
         assertFalse(expectedSpatialVector.equals(originalSpatialVector));
         expectedSpatialVector.applyInverseTransform(transform);
         assertTrue(expectedSpatialVector.epsilonEquals(originalSpatialVector, getEpsilon()));
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         expectedSpatialVector.set(originalSpatialVector);
         assertEquals(originalSpatialVector, expectedSpatialVector);
         
         expectedSpatialVector.applyTransform(transform);
         assertFalse(expectedSpatialVector.equals(originalSpatialVector));
         expectedSpatialVector.applyInverseTransform(transform);
         assertTrue(expectedSpatialVector.epsilonEquals(originalSpatialVector, getEpsilon()));
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T originalSpatialVector = newRandomSpatialVector(random);
         T expectedSpatialVector = newCopySpatialVector(originalSpatialVector);
         
         expectedSpatialVector.set(originalSpatialVector);
         assertEquals(originalSpatialVector, expectedSpatialVector);
         
         expectedSpatialVector.applyTransform(transform);
         assertFalse(expectedSpatialVector.equals(originalSpatialVector));
         expectedSpatialVector.applyInverseTransform(transform);
         assertTrue(expectedSpatialVector.epsilonEquals(originalSpatialVector, getEpsilon()));
      }
   }   
}



















