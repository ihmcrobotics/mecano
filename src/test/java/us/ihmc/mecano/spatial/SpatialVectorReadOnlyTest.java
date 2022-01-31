package us.ihmc.mecano.spatial;

import java.util.Random;
import static org.junit.jupiter.api.Assertions.*;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

public abstract class SpatialVectorReadOnlyTest<T extends SpatialVectorReadOnly>
{
   public abstract T newEmptySpatialVector();
   
   public abstract T newRandomSpatialVector(Random random);
   
   public abstract T newSpatialVector(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart);
   
   public abstract T newCopySpatialVector(SpatialVectorReadOnly other);
   
   public abstract double getEpsilon();
   
   private static final int ITERATIONS = 1000;
   
   @Test
   public void testGetElement()
   {
      
      Random random = new Random(29843);
 
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         
         assertTrue(randomSpatialVector.getElement(0) == randomSpatialVector.getAngularPartX());
         assertTrue(randomSpatialVector.getElement(1) == randomSpatialVector.getAngularPartY());
         assertTrue(randomSpatialVector.getElement(2) == randomSpatialVector.getAngularPartZ());
         assertTrue(randomSpatialVector.getElement(3) == randomSpatialVector.getLinearPartX());
         assertTrue(randomSpatialVector.getElement(4) == randomSpatialVector.getLinearPartY());
         assertTrue(randomSpatialVector.getElement(5) == randomSpatialVector.getLinearPartZ());
         
         //Test setElement with an index greater than 5 fails
         try
         {
            randomSpatialVector.getElement(i+6);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
      }
   }
   
   @Test
   public void testGets()
   {
      Random random = new Random(29843);
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         double[] vectorArray = new double[6];
         
         randomSpatialVector.get(vectorArray);
         
         assertTrue(vectorArray[0] == randomSpatialVector.getAngularPartX());
         assertTrue(vectorArray[1] == randomSpatialVector.getAngularPartY());
         assertTrue(vectorArray[2] == randomSpatialVector.getAngularPartZ());
         assertTrue(vectorArray[3] == randomSpatialVector.getLinearPartX());
         assertTrue(vectorArray[4] == randomSpatialVector.getLinearPartY());
         assertTrue(vectorArray[5] == randomSpatialVector.getLinearPartZ());
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         double[] vectorArray = new double[24];
         
         int startingArrayIndex = i % 18;
         
         randomSpatialVector.get(startingArrayIndex, vectorArray);
         
         assertTrue(vectorArray[startingArrayIndex] == randomSpatialVector.getAngularPartX());
         assertTrue(vectorArray[startingArrayIndex + 1] == randomSpatialVector.getAngularPartY());
         assertTrue(vectorArray[startingArrayIndex + 2] == randomSpatialVector.getAngularPartZ());
         assertTrue(vectorArray[startingArrayIndex + 3] == randomSpatialVector.getLinearPartX());
         assertTrue(vectorArray[startingArrayIndex + 4] == randomSpatialVector.getLinearPartY());
         assertTrue(vectorArray[startingArrayIndex + 5] == randomSpatialVector.getLinearPartZ());
      }
      

      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         float[] vectorArray = new float[6];
         
         randomSpatialVector.get(vectorArray);
         
         assertEquals(vectorArray[0], randomSpatialVector.getAngularPartX(), getEpsilon());
         assertEquals(vectorArray[1], randomSpatialVector.getAngularPartY(), getEpsilon());
         assertEquals(vectorArray[2], randomSpatialVector.getAngularPartZ(), getEpsilon());
         assertEquals(vectorArray[3], randomSpatialVector.getLinearPartX(), getEpsilon());
         assertEquals(vectorArray[4], randomSpatialVector.getLinearPartY(), getEpsilon());
         assertEquals(vectorArray[5], randomSpatialVector.getLinearPartZ(), getEpsilon());
      }  
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         float[] vectorArray = new float[24];
         
         int startingArrayIndex = i % 18;
         
         randomSpatialVector.get(startingArrayIndex, vectorArray);
         
         assertEquals(vectorArray[startingArrayIndex], randomSpatialVector.getAngularPartX(), getEpsilon());
         assertEquals(vectorArray[startingArrayIndex + 1], randomSpatialVector.getAngularPartY(), getEpsilon());
         assertEquals(vectorArray[startingArrayIndex + 2], randomSpatialVector.getAngularPartZ(), getEpsilon());
         assertEquals(vectorArray[startingArrayIndex + 3], randomSpatialVector.getLinearPartX(), getEpsilon());
         assertEquals(vectorArray[startingArrayIndex + 4], randomSpatialVector.getLinearPartY(), getEpsilon());
         assertEquals(vectorArray[startingArrayIndex + 5], randomSpatialVector.getLinearPartZ(), getEpsilon());
      }
    
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         DMatrix vectorMatrix = new DMatrixRMaj(6,1); 
         
         randomSpatialVector.get(vectorMatrix);
         
         assertTrue(vectorMatrix.get(0,0) == randomSpatialVector.getAngularPartX());
         assertTrue(vectorMatrix.get(1,0) == randomSpatialVector.getAngularPartY());
         assertTrue(vectorMatrix.get(2,0) == randomSpatialVector.getAngularPartZ());
         assertTrue(vectorMatrix.get(3,0) == randomSpatialVector.getLinearPartX());
         assertTrue(vectorMatrix.get(4,0) == randomSpatialVector.getLinearPartY());
         assertTrue(vectorMatrix.get(5,0) == randomSpatialVector.getLinearPartZ());
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         DMatrix vectorMatrix = new DMatrixRMaj(24,1); 
         
         int startingArrayIndex = i % 18;
         
         randomSpatialVector.get(startingArrayIndex, vectorMatrix);
         
         assertTrue(vectorMatrix.get(startingArrayIndex,0) == randomSpatialVector.getAngularPartX());
         assertTrue(vectorMatrix.get(startingArrayIndex + 1,0) == randomSpatialVector.getAngularPartY());
         assertTrue(vectorMatrix.get(startingArrayIndex + 2,0) == randomSpatialVector.getAngularPartZ());
         assertTrue(vectorMatrix.get(startingArrayIndex + 3,0) == randomSpatialVector.getLinearPartX());
         assertTrue(vectorMatrix.get(startingArrayIndex + 4,0) == randomSpatialVector.getLinearPartY());
         assertTrue(vectorMatrix.get(startingArrayIndex + 5,0) == randomSpatialVector.getLinearPartZ());
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         DMatrix vectorMatrix = new DMatrixRMaj(24,24); 
         
         int startingArrayRow = i % 18;
         int startingArrayColumn = Math.abs(random.nextInt()) % 18;
         
         randomSpatialVector.get(startingArrayRow, startingArrayColumn, vectorMatrix);
         
         assertTrue(vectorMatrix.get(startingArrayRow,startingArrayColumn) == randomSpatialVector.getAngularPartX());
         assertTrue(vectorMatrix.get(startingArrayRow + 1,startingArrayColumn) == randomSpatialVector.getAngularPartY());
         assertTrue(vectorMatrix.get(startingArrayRow + 2,startingArrayColumn) == randomSpatialVector.getAngularPartZ());
         assertTrue(vectorMatrix.get(startingArrayRow + 3,startingArrayColumn) == randomSpatialVector.getLinearPartX());
         assertTrue(vectorMatrix.get(startingArrayRow + 4,startingArrayColumn) == randomSpatialVector.getLinearPartY());
         assertTrue(vectorMatrix.get(startingArrayRow + 5,startingArrayColumn) == randomSpatialVector.getLinearPartZ());
      }

   }
   
   @Test
   public void testDot()
   {
      Random random = new Random(65841);
      double testDot;
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         T randomSpatialVector = newRandomSpatialVector(random);
         T randomSpatialVectorSameReferenceFrame = newSpatialVector(randomSpatialVector.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));

         testDot = randomSpatialVector.dot(randomSpatialVectorSameReferenceFrame);
      
         assertEquals(testDot, randomSpatialVector.getAngularPartX() * randomSpatialVectorSameReferenceFrame.getAngularPartX() + randomSpatialVector.getAngularPartY() * randomSpatialVectorSameReferenceFrame.getAngularPartY() + randomSpatialVector.getAngularPartZ() * randomSpatialVectorSameReferenceFrame.getAngularPartZ()
                             + randomSpatialVector.getLinearPartX() * randomSpatialVectorSameReferenceFrame.getLinearPartX() + randomSpatialVector.getLinearPartY() * randomSpatialVectorSameReferenceFrame.getLinearPartY() + randomSpatialVector.getLinearPartZ() * randomSpatialVectorSameReferenceFrame.getLinearPartZ(), getEpsilon());
         
         //Test setElement with an index greater than 5 fails
         try
         {
            T randomSpatialVectorDifferentReferenceFrame = newRandomSpatialVector(random);

            testDot = randomSpatialVector.dot(randomSpatialVectorDifferentReferenceFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            //good 
         }
      }
   }
   
   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(1250L);
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3DBasics originalAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3DBasics originalLinearPart = EuclidCoreRandomTools.nextVector3D(random);

         Vector3DBasics changeAngularPart = originalAngularPart;
         Vector3DBasics changeLinearPart = originalLinearPart;

         T originalSpatialVector = newSpatialVector(ReferenceFrameTools.getWorldFrame(), originalAngularPart, originalLinearPart);
         
         //Angular Part - element above epsilon
         changeAngularPart.setElement(i % 3, originalAngularPart.getElement(i % 3) - 1.01 * getEpsilon());
         T changedAboveSpatialVector = newSpatialVector(ReferenceFrameTools.getWorldFrame(), changeAngularPart, originalLinearPart);
         assertFalse(changedAboveSpatialVector.epsilonEquals(originalSpatialVector, getEpsilon()));
         
         //Angular Part - element below epsilon
         changeAngularPart.setElement(i % 3, changeAngularPart.getElement(i % 3) + 0.99 * getEpsilon());
         T changedBelowSpatialVector = newSpatialVector(ReferenceFrameTools.getWorldFrame(), changeAngularPart, originalLinearPart);
         assertTrue(changedBelowSpatialVector.epsilonEquals(originalSpatialVector, getEpsilon()));
         
         //Linear Part - element above epsilon
         changeLinearPart.setElement(i % 3, originalLinearPart.getElement(i % 3) - 1.01 * getEpsilon());
         changedAboveSpatialVector = newSpatialVector(ReferenceFrameTools.getWorldFrame(), originalAngularPart, changeLinearPart);
         assertFalse(changedAboveSpatialVector.epsilonEquals(originalSpatialVector, getEpsilon()));
         
         //Angular Part - element below epsilon
         changeLinearPart.setElement(i % 3, originalLinearPart.getElement(i % 3) + 0.99 * getEpsilon());
         changedBelowSpatialVector = newSpatialVector(ReferenceFrameTools.getWorldFrame(), originalAngularPart, changeLinearPart);
         assertTrue(changedBelowSpatialVector.epsilonEquals(originalSpatialVector, getEpsilon()));
         
         //Reference Frame different - angular/linear parts the same
         T frameDifferentSpatialVector = newSpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), originalAngularPart, originalLinearPart);
         assertFalse(originalSpatialVector.epsilonEquals(frameDifferentSpatialVector, getEpsilon()));
      }
   }
}
