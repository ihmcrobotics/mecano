package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameNameRestrictionLevel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.tools.MecanoRandomTools;

public class ArticulatedBodyInertiaTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @BeforeEach
   public void disableNameRestriction()
   {
      ReferenceFrame.getWorldFrame().setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);
   }

   @Test
   public void testApplyTransform()
   {
      Random random = new Random(2552);

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's compare applyTransform against SpatialInertia
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame expressedInFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialInertia spatialInertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);
         ArticulatedBodyInertia articulatedBodyInertia = new ArticulatedBodyInertia();
         articulatedBodyInertia.setIncludingFrame(spatialInertia);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         spatialInertia.applyTransform(transform);
         articulatedBodyInertia.applyTransform(transform);

         transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         spatialInertia.applyTransform(transform);
         articulatedBodyInertia.applyTransform(transform);

         DMatrixRMaj expected = new DMatrixRMaj(6, 6);
         DMatrixRMaj actual = new DMatrixRMaj(6, 6);

         spatialInertia.get(expected);
         articulatedBodyInertia.get(actual);

         boolean areEqual = MatrixFeatures_DDRM.isEquals(expected, actual, EPSILON);

         if (!areEqual)
         {
            LogTools.info("expected: " + expected);
            LogTools.info("actual: " + actual);
         }

         assertTrue(areEqual);
      }
   }

   @Test
   public void testApplyInverseTransform()
   {
      Random random = new Random(2552);

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's compare applyInverseTransform against SpatialInertia
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame expressedInFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialInertia spatialInertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);
         ArticulatedBodyInertia articulatedBodyInertia = new ArticulatedBodyInertia();
         articulatedBodyInertia.setIncludingFrame(spatialInertia);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         spatialInertia.applyInverseTransform(transform);
         articulatedBodyInertia.applyInverseTransform(transform);

         transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         spatialInertia.applyInverseTransform(transform);
         articulatedBodyInertia.applyInverseTransform(transform);

         DMatrixRMaj expected = new DMatrixRMaj(6, 6);
         DMatrixRMaj actual = new DMatrixRMaj(6, 6);

         spatialInertia.get(expected);
         articulatedBodyInertia.get(actual);

         boolean areEqual = MatrixFeatures_DDRM.isEquals(expected, actual, EPSILON);

         if (!areEqual)
         {
            LogTools.info("expected: " + expected);
            LogTools.info("actual: " + actual);
         }

         assertTrue(areEqual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's compare applyInverseTransform against applyTransform
         ArticulatedBodyInertia articulatedBodyInertia = new ArticulatedBodyInertia();
         articulatedBodyInertia.getAngularInertia().set(MecanoRandomTools.nextSymmetricPositiveDefiniteMatrix3D(random));
         articulatedBodyInertia.getLinearInertia().set(MecanoRandomTools.nextSymmetricPositiveDefiniteMatrix3D(random));
         articulatedBodyInertia.getCrossInertia().set(EuclidCoreRandomTools.nextMatrix3D(random));

         DMatrixRMaj expected = new DMatrixRMaj(6, 6);
         DMatrixRMaj actual = new DMatrixRMaj(6, 6);

         articulatedBodyInertia.get(expected);

         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         articulatedBodyInertia.applyTransform(transform);
         articulatedBodyInertia.applyInverseTransform(transform);

         articulatedBodyInertia.get(actual);

         boolean areEqual = MatrixFeatures_DDRM.isEquals(expected, actual, EPSILON);

         if (!areEqual)
         {
            LogTools.info("expected: " + expected);
            LogTools.info("actual: " + actual);
         }

         assertTrue(areEqual);
      }
   }
}
