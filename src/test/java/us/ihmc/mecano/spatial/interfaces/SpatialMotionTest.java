package us.ihmc.mecano.spatial.interfaces;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public abstract class SpatialMotionTest<T extends SpatialMotionBasics>
{
   protected Random random = new Random(100L);

   protected ReferenceFrame frameA;
   protected ReferenceFrame frameB;
   protected ReferenceFrame frameC;
   protected ReferenceFrame frameD;

   public abstract T createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
                                               Vector3DReadOnly angularPart, Vector3DReadOnly linearPart);

   public abstract T createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DMatrixRMaj matrix);

   @BeforeEach
   public void setUp() throws Exception
   {
      frameA = ReferenceFrameTools.constructARootFrame("A");
      frameB = new ReferenceFrame("B", frameA)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(1.0, 2.0, 3.0);
            RigidBodyTransform translation = new RigidBodyTransform();
            translation.getTranslation().set(new Vector3D(3.0, 4.0, 5.0));
            transformToParent.multiply(translation);
         }
      };

      frameC = new ReferenceFrame("C", frameB)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(1.0, 2.0, 3.0);
            RigidBodyTransform translation = new RigidBodyTransform();
            translation.getTranslation().set(new Vector3D(3.0, 4.0, 5.0));
            transformToParent.multiply(translation);
         }
      };

      frameD = ReferenceFrameTools.constructARootFrame("D");

      frameB.update();
      frameC.update();
   }

   /**
    * Test inverting a twist
    */

   @Test
   public void testInvert()
   {
      Vector3D linearPart = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D angularPart = EuclidCoreRandomTools.nextVector3D(random);

      Vector3D linearPartInverse = new Vector3D(linearPart);
      linearPartInverse.scale(-1.0);

      Vector3D angularPartInverse = new Vector3D(angularPart);
      angularPartInverse.scale(-1.0);

      T twist1 = createSpatialMotionVector(frameB, frameA, frameA, angularPart, linearPart);
      twist1.invert();

      double epsilon = 1e-10;
      EuclidCoreTestTools.assertEquals(angularPartInverse, twist1.getAngularPart(), epsilon);
      EuclidCoreTestTools.assertEquals(linearPartInverse, twist1.getLinearPart(), epsilon);
      assertEquals(frameA, twist1.getReferenceFrame());
      assertEquals(frameB, twist1.getBaseFrame());
      assertEquals(frameA, twist1.getBodyFrame());

      T twist2 = createSpatialMotionVector(frameB, frameA, frameB, angularPart, linearPart);
      twist2.invert();
      EuclidCoreTestTools.assertEquals(angularPartInverse, twist2.getAngularPart(), epsilon);
      EuclidCoreTestTools.assertEquals(linearPartInverse, twist2.getLinearPart(), epsilon);
      assertEquals(frameB, twist2.getReferenceFrame());
      assertEquals(frameB, twist2.getBaseFrame());
      assertEquals(frameA, twist2.getBodyFrame());
   }

   /**
    * Constructing using a matrix
    */

   @Test
   public void testConstructUsingMatrix()
   {
      DMatrixRMaj matrix = RandomMatrices_DDRM.rectangle(SpatialVectorReadOnly.SIZE, 1, random);
      T spatialMotionVector = createSpatialMotionVector(frameC, frameD, frameA, matrix);
      DMatrixRMaj matrixBack = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
      spatialMotionVector.get(matrixBack);
      EjmlUnitTests.assertEquals(matrix, matrixBack, 0.0);
   }

   @Test
   public void testConstructUsingMatrixTooSmall()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         DMatrixRMaj matrix = new DMatrixRMaj(SpatialVectorReadOnly.SIZE - 1, 1);
         createSpatialMotionVector(frameC, frameD, frameA, matrix);
      });
   }
}
