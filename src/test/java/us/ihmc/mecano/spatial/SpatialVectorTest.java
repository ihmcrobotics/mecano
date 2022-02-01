package us.ihmc.mecano.spatial;

import java.util.Random;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.mecano.tools.MecanoTestTools;

public class SpatialVectorTest extends SpatialVectorBasicsTest<SpatialVector>
{
   @Override
   public SpatialVector newEmptySpatialVector()
   {
      // Return new empty Triangle3D()
      return new SpatialVector();
   }

   @Override
   public SpatialVector newRandomSpatialVector(Random random)
   {
      return new SpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
   }

   @Override
   public SpatialVector newSpatialVector(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      // Return new SpatialVector with reference frame, angular part, and linear part
      return new SpatialVector(expressedInFrame, angularPart, linearPart);
   }

   @Override
   public SpatialVector newCopySpatialVector(SpatialVectorReadOnly other)
   {
      // Return copy SpatialVector 
      return new SpatialVector(other);
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
      
      /*
       * Test constructor - public SpatialVector()
       * Creates a new spatial vector with its components set to zero and its reference frame set to
       * getWorldFrame()
       */
      SpatialVector spatialVector = new SpatialVector();

      assertEquals(spatialVector.getReferenceFrame(), ReferenceFrameTools.getWorldFrame());
      assertEquals(spatialVector.getAngularPart().getReferenceFrame(), ReferenceFrameTools.getWorldFrame());
      assertEquals(spatialVector.getLinearPart().getReferenceFrame(), ReferenceFrameTools.getWorldFrame());
      assertEquals(spatialVector.getAngularPartX(), 0); 
      assertEquals(spatialVector.getAngularPartY(), 0);
      assertEquals(spatialVector.getAngularPartZ(), 0);
      assertEquals(spatialVector.getLinearPartX(), 0);
      assertEquals(spatialVector.getLinearPartY(), 0);
      assertEquals(spatialVector.getLinearPartZ(), 0);
      
      /*
       * Test constructor - public SpatialVector(ReferenceFrame expressedInFrame) 
       * Creates a new spatial vector with its components set to zero and initializes its reference frame.
       *
       * @param expressedInFrame the initial frame in which this vector is expressed.
       */
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialVector spatialVectorByReferenceFrame = new SpatialVector(randomFrame);
         
         assertEquals(spatialVectorByReferenceFrame.getReferenceFrame(), randomFrame);
         assertEquals(spatialVectorByReferenceFrame.getAngularPart().getReferenceFrame(), randomFrame);
         assertEquals(spatialVectorByReferenceFrame.getLinearPart().getReferenceFrame(), randomFrame);
         assertEquals(spatialVectorByReferenceFrame.getAngularPartX(), 0); 
         assertEquals(spatialVectorByReferenceFrame.getAngularPartY(), 0);
         assertEquals(spatialVectorByReferenceFrame.getAngularPartZ(), 0);
         assertEquals(spatialVectorByReferenceFrame.getLinearPartX(), 0);
         assertEquals(spatialVectorByReferenceFrame.getLinearPartY(), 0);
         assertEquals(spatialVectorByReferenceFrame.getLinearPartZ(), 0);
      }
      
      /*
       * Test constructor - public SpatialVector(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
       * Creates a new spatial vector and initializes its components and the reference frame it is
       * expressed in.
       *
       * @param expressedInFrame the initial frame in which this vector is expressed.
       * @param angularPart      the vector holding the values for the angular part. Not modified.
       * @param linearPart       the vector holding the values for the linear part. Not modified.
       */
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         
         SpatialVector spatialVectorByReferenceFrameAngularPartLinearPart = new SpatialVector(randomFrame, randomAngularPart, randomLinearPart);
         
         assertEquals(spatialVectorByReferenceFrameAngularPartLinearPart.getReferenceFrame(), randomFrame);
         assertEquals(spatialVectorByReferenceFrameAngularPartLinearPart.getAngularPart().getReferenceFrame(), randomFrame);
         assertEquals(spatialVectorByReferenceFrameAngularPartLinearPart.getLinearPart().getReferenceFrame(), randomFrame);
         EuclidCoreTestTools.assertTuple3DEquals(spatialVectorByReferenceFrameAngularPartLinearPart.getAngularPart(), randomAngularPart, getEpsilon());
         EuclidCoreTestTools.assertTuple3DEquals(spatialVectorByReferenceFrameAngularPartLinearPart.getLinearPart(), randomLinearPart, getEpsilon());
      }
    
      /*
       * Test constructor public SpatialVector(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
       * Creates a new spatial vector and initializes its components and the reference frame it is
       * expressed in.
       *
       * @param angularPart the vector holding the new values for the angular part. Not modified.
       * @param linearPart  the vector holding the new values for the linear part. Not modified.
       * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
       *                                         frame.
       */
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FrameVector3D randomAngularPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         FrameVector3D randomLinearPart = EuclidFrameRandomTools.nextFrameVector3D(random, randomFrame);
         
         SpatialVector spatialVectorByAngularPartLinearPart = new SpatialVector(randomAngularPart, randomLinearPart);
         
         assertEquals(spatialVectorByAngularPartLinearPart.getReferenceFrame(), randomFrame);
         assertEquals(spatialVectorByAngularPartLinearPart.getAngularPart().getReferenceFrame(), randomFrame);
         assertEquals(spatialVectorByAngularPartLinearPart.getLinearPart().getReferenceFrame(), randomFrame);        
         EuclidCoreTestTools.assertTuple3DEquals(spatialVectorByAngularPartLinearPart.getAngularPart(), randomAngularPart, getEpsilon());
         EuclidCoreTestTools.assertTuple3DEquals(spatialVectorByAngularPartLinearPart.getLinearPart(), randomLinearPart, getEpsilon());
         
         ReferenceFrame randomDifferentFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         randomLinearPart.setReferenceFrame(randomDifferentFrame);
            
         Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
         {
            @SuppressWarnings("unused")
            SpatialVector spatialVectorByAngularPartLinearPartDifferent = new SpatialVector(randomAngularPart, randomLinearPart);
         });
      }
      
      /*
       * Test create copy constructor - SpatialVector(SpatialVectorReadOnly other)
       *
       * @param other the other spatial vector to copy. Not modified.
       */
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         
         SpatialVector spatialVectorByReferenceFrameAngularPartLinearPart = new SpatialVector(randomFrame, randomAngularPart, randomLinearPart);
         
         SpatialVector spatialVectorByCopy = new SpatialVector(spatialVectorByReferenceFrameAngularPartLinearPart);
         
         assertFalse(spatialVectorByCopy == spatialVectorByReferenceFrameAngularPartLinearPart);
         MecanoTestTools.assertSpatialVectorEquals(spatialVectorByReferenceFrameAngularPartLinearPart, spatialVectorByCopy, getEpsilon());
      }      
   } 
   
   @Test
   public void testSetReferenceFrame()
   {
      Random random = new Random(15875);
      
      ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      SpatialVector spatialVector = new SpatialVector();
      
      assertEquals(spatialVector.getReferenceFrame(), ReferenceFrameTools.getWorldFrame());
      spatialVector.setReferenceFrame(randomFrame);
      assertEquals(spatialVector.getReferenceFrame(), randomFrame);
   }
   
   @Test
   public void testEquals()
   {
      Random random = new Random(130375);
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         //Test equals() method
         SpatialVector originalSpatialVector = new SpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         SpatialVector copySpatialVector = originalSpatialVector;
         
         assertTrue(originalSpatialVector.equals(copySpatialVector));
         
         SpatialVector differentSpatialVector = new SpatialVector(EuclidFrameRandomTools.nextReferenceFrame(random), EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
        
         assertFalse(originalSpatialVector.equals(differentSpatialVector));
         
         SpatialVector emptySpatialVector = newEmptySpatialVector();
         
         assertFalse(originalSpatialVector.equals(emptySpatialVector));
      }
      
   }

   @Test
   public void testToString()
   {
      Random random = new Random(130375);
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Vector3D randomAngularPart = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D randomLinearPart = EuclidCoreRandomTools.nextVector3D(random);
         
         SpatialVector spatialVectorByReferenceFrameAngularPartLinearPart = new SpatialVector(randomFrame, randomAngularPart, randomLinearPart);
         assertEquals(spatialVectorByReferenceFrameAngularPartLinearPart.toString(), MecanoIOTools.getSpatialVectorString(spatialVectorByReferenceFrameAngularPartLinearPart));
      } 
      
   }
}


