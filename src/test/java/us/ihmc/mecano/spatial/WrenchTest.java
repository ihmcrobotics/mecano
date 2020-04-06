package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.NormOps;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;

public class WrenchTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;
   private ReferenceFrame frameA;
   private ReferenceFrame frameB;
   private ReferenceFrame frameC;

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

      frameB.update();
      frameC.update();
   }

   @Test
   public void testApplyInverseTransform()
   {
      Random random = new Random(325456);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Wrench expected = MecanoRandomTools.nextWrench(random, frameA, frameA);
         RigidBodyTransform randomTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         Wrench actual = new Wrench(expected);
         actual.applyTransform(randomTransform);
         actual.applyInverseTransform(randomTransform);
         MecanoTestTools.assertWrenchEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testChangeExpressedInWhatReferenceFrame()
   {
      // create random twists and random wrenches, transform both to other frames,
      // and check that the instantaneous power returned by dot() remains the same
      double epsilon = 1e-10;
      Random random = new Random(102L);
      int nTests = 10;
      for (int i = 0; i < nTests; i++)
      {
         Vector3D angularVelocity = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Vector3D linearVelocity = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Twist twist = new Twist(frameC, frameA, frameA, angularVelocity, linearVelocity);

         Vector3D torque = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Vector3D force = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Wrench wrench = new Wrench(frameC, frameA, force, torque); // baseFrame doesn't matter

         double power1 = twist.dot(wrench);

         twist.changeFrame(frameB);
         wrench.changeFrame(frameB);

         double power2 = twist.dot(wrench);

         assertEquals(power1, power2, epsilon);
      }
   }

   @Test
   public void testDefaultConstructor()
   {
      Wrench wrench = new Wrench();
      assertNull(wrench.getBodyFrame());
      assertNull(wrench.getReferenceFrame());
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), wrench.getAngularPart(), 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), wrench.getLinearPart(), 0.0);
   }

   @Test
   public void testConstructUsingMatrix()
   {
      Random random = new Random(167L);
      DenseMatrix64F matrix = RandomMatrices.createRandom(Wrench.SIZE, 1, random);
      Wrench wrench = new Wrench(frameA, frameB, matrix);
      DenseMatrix64F matrixBack = new DenseMatrix64F(Wrench.SIZE, 1);
      wrench.get(matrixBack);
      EjmlUnitTests.assertEquals(matrix, matrixBack, 0.0);

      Vector3D torque = new Vector3D();
      Vector3D force = new Vector3D();
      torque.set(matrix);
      force.set(3, matrix);
      EuclidCoreTestTools.assertTuple3DEquals(torque, wrench.getAngularPart(), 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(force, wrench.getLinearPart(), 0.0);
   }

   @Test
   public void testConstructUsingMatrixTooSmall()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         Random random = new Random(12342L);
         DenseMatrix64F matrix = RandomMatrices.createRandom(Wrench.SIZE - 1, 1, random);
         new Wrench(frameA, frameB, matrix);
      });
   }

   @Test
   public void testConstructUsingDoubleArray()
   {
      Random random = new Random(1234L);
      int vectorDimension = 3;
      double[] angularArray = new double[vectorDimension];
      double[] linearArray = new double[vectorDimension];
      double[] spatialArray = new double[2 * vectorDimension];

      for (int i = 0; i < vectorDimension; i++)
      {
         angularArray[i] = random.nextDouble();
         linearArray[i] = random.nextDouble();
         spatialArray[i] = angularArray[i];
         spatialArray[i + vectorDimension] = linearArray[i];
      }

      Wrench wrench = new Wrench(frameC, frameA, spatialArray);
      assertEquals(wrench.getBodyFrame(), frameC);
      assertEquals(wrench.getReferenceFrame(), frameA);
      double epsilon = 0.0;
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(angularArray), wrench.getAngularPart(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(linearArray), wrench.getLinearPart(), epsilon);
   }

   @Test
   public void testConstructUsingDoubleArrayTooSmall()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         new Wrench(frameA, frameB, new double[Wrench.SIZE - 1]);
      });
   }

   @Test
   public void testAddNotAllowed()
   {
      Wrench wrench1 = new Wrench(frameB, frameA);
      Wrench wrench2 = new Wrench(frameC, frameA);
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         wrench1.add(wrench2);
      });
   }

   @Test
   public void testAddNotAllowed2()
   {
      Wrench wrench1 = new Wrench(frameB, frameA);
      Wrench wrench2 = new Wrench(frameB, frameB);
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         wrench1.add(wrench2);
      });
   }

   @Test
   public void testAdd()
   {
      Random random = new Random(187L);
      Wrench wrench1 = new Wrench(frameA, frameB, RandomMatrices.createRandom(Wrench.SIZE, 1, random));
      Wrench wrench2 = new Wrench(frameA, frameB, RandomMatrices.createRandom(Wrench.SIZE, 1, random));
      Wrench wrench3 = new Wrench(wrench1);
      wrench3.add(wrench2);

      Vector3DBasics linearPart = wrench1.getLinearPart();
      linearPart.add(wrench2.getLinearPart());

      Vector3DBasics angularPart = wrench1.getAngularPart();
      angularPart.add(wrench2.getAngularPart());

      EuclidCoreTestTools.assertTuple3DEquals(wrench3.getLinearPart(), linearPart, 1e-24);
      EuclidCoreTestTools.assertTuple3DEquals(wrench3.getAngularPart(), angularPart, 1e-24);
   }

   @Test
   public void testSubNotAllowed()
   {
      Wrench wrench1 = new Wrench(frameB, frameA);
      Wrench wrench2 = new Wrench(frameC, frameA);
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         wrench1.sub(wrench2);
      });
   }

   @Test
   public void testSubNotAllowed2()
   {
      Wrench wrench1 = new Wrench(frameB, frameA);
      Wrench wrench2 = new Wrench(frameB, frameB);
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         wrench1.sub(wrench2);
      });
   }

   @Test
   public void testSub()
   {
      Random random = new Random(187L);
      Wrench wrench1 = new Wrench(frameA, frameB, RandomMatrices.createRandom(Wrench.SIZE, 1, random));
      Wrench wrench2 = new Wrench(frameA, frameB, RandomMatrices.createRandom(Wrench.SIZE, 1, random));
      Wrench wrench3 = new Wrench(wrench1);
      wrench3.sub(wrench2);

      Vector3DBasics linearPart = wrench1.getLinearPart();
      linearPart.sub(wrench2.getLinearPart());

      Vector3DBasics angularPart = wrench1.getAngularPart();
      angularPart.sub(wrench2.getAngularPart());

      EuclidCoreTestTools.assertTuple3DEquals(wrench3.getLinearPart(), linearPart, 1e-24);
      EuclidCoreTestTools.assertTuple3DEquals(wrench3.getAngularPart(), angularPart, 1e-24);
   }

   @Test
   public void testCheckAndSetNotAllowed1()
   {
      Wrench wrench1 = new Wrench(frameA, frameB);
      Wrench wrench2 = new Wrench(frameA, frameC);
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         wrench2.set(wrench1);
      });
   }

   @Test
   public void testCheckAndSetNotAllowed2()
   {
      Wrench wrench1 = new Wrench(frameA, frameB);
      Wrench wrench2 = new Wrench(frameC, frameB);
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         wrench2.set(wrench1);
      });
   }

   @Test
   public void testDotProduct()
   {
      testDotProduct(frameA, frameB, frameC);
   }

   @Test
   public void testDotProductNotAllowed1()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         testDotProductNotAllowed1(frameA, frameB, frameC);
      });
   }

   @Test
   public void testDotProductNotAllowed2()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         testDotProductNotAllowed2(frameA, frameB, frameC);
      });
   }

   public static void testDotProduct(ReferenceFrame frameA, ReferenceFrame frameB, ReferenceFrame frameC)
   {
      Random random = new Random(187L);
      DenseMatrix64F twistMatrix = RandomMatrices.createRandom(Twist.SIZE, 1, random);
      DenseMatrix64F wrenchMatrix = RandomMatrices.createRandom(Wrench.SIZE, 1, random);
      Twist twist = new Twist(frameA, frameB, frameC, twistMatrix);
      Wrench wrench = new Wrench(frameA, frameC, wrenchMatrix);
      DenseMatrix64F c = new DenseMatrix64F(1, 1);
      CommonOps.multTransA(twistMatrix, wrenchMatrix, c);
      assertEquals(c.get(0, 0), wrench.dot(twist), 1e-12);
   }

   public static void testDotProductNotAllowed1(ReferenceFrame frameA, ReferenceFrame frameB, ReferenceFrame frameC)
   {
      Wrench wrench = null;
      Twist twist = null;
      try
      {
         wrench = new Wrench(frameA, frameB);
         twist = new Twist(frameA, frameB, frameC);
      }
      catch (RuntimeException e)
      {
         fail();
      }

      wrench.dot(twist);
   }

   public static void testDotProductNotAllowed2(ReferenceFrame frameA, ReferenceFrame frameB, ReferenceFrame frameC)
   {
      Wrench wrench = null;
      Twist twist = null;
      try
      {
         wrench = new Wrench(frameB, frameB);
         twist = new Twist(frameA, frameB, frameB);
      }
      catch (RuntimeException e)
      {
         fail();
      }

      wrench.dot(twist);
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(71243L);
      Wrench wrench = new Wrench(frameA, frameB, RandomMatrices.createRandom(Wrench.SIZE, 1, random));
      wrench.setToZero(frameC, frameA);
      assertEquals(frameC, wrench.getBodyFrame());
      assertEquals(frameA, wrench.getReferenceFrame());
      DenseMatrix64F matrix = RandomMatrices.createRandom(Wrench.SIZE, 1, random);
      double epsilon = 1e-12;
      assertTrue(NormOps.normP2(matrix) > epsilon);
      wrench.get(matrix);
      assertTrue(NormOps.normP2(matrix) == 0.0);
   }

   @Test
   public void testGetAngularPartAt()
   {
      Random random = new Random(4788);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame expressedInFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameAtObserverPosition = EuclidFrameRandomTools.nextReferenceFrame(random);
         Wrench wrench = MecanoRandomTools.nextWrench(random, bodyFrame, expressedInFrame);

         FramePoint3D observerPosition = new FramePoint3D(frameAtObserverPosition);
         observerPosition.changeFrame(expressedInFrame);
         FrameVector3D actualAngularPart = new FrameVector3D();
         wrench.getAngularPartAt(observerPosition, actualAngularPart);

         FrameVector3D expectedAngularPart = new FrameVector3D();
         wrench.changeFrame(frameAtObserverPosition);
         expectedAngularPart.setIncludingFrame(wrench.getAngularPart());
         expectedAngularPart.changeFrame(expressedInFrame);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedAngularPart, actualAngularPart, EPSILON);

         assertThrows(ReferenceFrameMismatchException.class,
                      () -> wrench.getAngularPartAt(new FramePoint3D(EuclidFrameRandomTools.nextReferenceFrame(random)), new FrameVector3D()));
      }
   }
}
