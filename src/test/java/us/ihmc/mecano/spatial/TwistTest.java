package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialMotionTest;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MecanoTools;

public class TwistTest extends SpatialMotionTest<Twist>
{
   @Override
   public Twist createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart,
                                          Vector3DReadOnly linearPart)
   {
      return new Twist(bodyFrame, baseFrame, expressedInFrame, angularPart, linearPart);
   }

   @Override
   public Twist createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      return new Twist(bodyFrame, baseFrame, expressedInFrame, matrix);
   }

   /**
    * Basic test of constructor, getters and setters
    */

   @Test
   public void testConstructionAndGettersAndSetters()
   {
      // construct a random twist that expresses the motion of frame A with respect to B, expressed in frame C
      Vector3D angularVelocity = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist = new Twist(frameB, frameA, frameC, angularVelocity, linearVelocity);

      // test getters
      double epsilon = 1e-14;
      Vector3DBasics angularVelocity2 = twist.getAngularPart();
      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity, angularVelocity2, epsilon);

      Vector3DBasics linearVelocity2 = twist.getLinearPart();
      EuclidCoreTestTools.assertTuple3DEquals(linearVelocity, linearVelocity2, epsilon);

      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity, angularVelocity2, epsilon); // make sure the linear velocity setter didn't change the angular velocity

      // test setters
      Vector3D angularVelocity3 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      twist.getAngularPart().set(angularVelocity3);

      Vector3D linearVelocity3 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      twist.getLinearPart().set(linearVelocity3);

      Vector3DBasics angularVelocity4 = twist.getAngularPart();
      Vector3DBasics linearVelocity4 = twist.getLinearPart();

      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity3, angularVelocity4, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(linearVelocity3, linearVelocity4, epsilon);
   }

   /**
    * default constructor
    */

   @Test
   public void testDefaultConstructor()
   {
      Twist twist = new Twist();
      assertNull(twist.getBaseFrame());
      assertNull(twist.getBodyFrame());
      assertNull(twist.getReferenceFrame());
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), twist.getAngularPart(), 0.0);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), twist.getLinearPart(), 0.0);
   }

   /**
    * Constructing using a double array
    */

   @Test
   public void testConstructUsingArray()
   {
      double[] array = new double[Twist.SIZE];
      for (int i = 0; i < array.length; i++)
      {
         array[i] = random.nextDouble();
      }

      Twist twist = new Twist(frameC, frameD, frameA, array);
      DenseMatrix64F matrixBack = new DenseMatrix64F(Twist.SIZE, 1);
      twist.get(matrixBack);
      double[] arrayBack = matrixBack.getData();
      assertArrayEquals(array, arrayBack);
   }

   @Test
   public void testConstructUsingArrayTooSmall()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         double[] array = new double[Twist.SIZE - 1];
         new Twist(frameC, frameD, frameA, array);
      });
   }

   /**
    * Copy constructor
    */

   @Test
   public void testCopyConstructor()
   {
      DenseMatrix64F inputMatrix = RandomMatrices.createRandom(Twist.SIZE, 1, random);
      Twist twist = new Twist(frameC, frameD, frameA, inputMatrix);
      Twist twistCopy = new Twist(twist);

      DenseMatrix64F twistMatrix = new DenseMatrix64F(Twist.SIZE, 1);
      twist.get(twistMatrix);

      DenseMatrix64F twistCopyMatrix = new DenseMatrix64F(Twist.SIZE, 1);
      twistCopy.get(twistCopyMatrix);

      // test that they're the same
      EjmlUnitTests.assertEquals(twistMatrix, twistCopyMatrix, 0.0);
      assertEquals(twist.getBodyFrame(), twistCopy.getBodyFrame());
      assertEquals(twist.getReferenceFrame(), twistCopy.getReferenceFrame());
      assertEquals(twist.getBaseFrame(), twistCopy.getBaseFrame());

      // test that we're actually copying, not just using references
      inputMatrix = RandomMatrices.createRandom(Twist.SIZE, 1, random);
      twist.setIncludingFrame(frameD, frameA, frameC, inputMatrix);
      twist.get(twistMatrix);
      twistCopy.get(twistCopyMatrix);

      for (int i = 0; i < twistMatrix.getNumElements(); i++)
      {
         if (twistMatrix.get(i) == twistCopyMatrix.get(i))
            fail();
      }

      assertNotSame(twist.getBodyFrame(), twistCopy.getBodyFrame());
      assertNotSame(twist.getReferenceFrame(), twistCopy.getReferenceFrame());
      assertNotSame(twist.getBaseFrame(), twistCopy.getBaseFrame());
   }

   /**
    * Dot product
    */

   @Test
   public void testDotProduct()
   {
      WrenchTest.testDotProduct(frameA, frameB, frameC);
   }

   @Test
   public void testDotProductNotAllowed1()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         WrenchTest.testDotProductNotAllowed1(frameA, frameB, frameC);
      });
   }

   @Test
   public void testDotProductNotAllowed2()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         WrenchTest.testDotProductNotAllowed2(frameA, frameB, frameC);
      });
   }

   /**
    * You shouldn't be able to add two twists expressed in different frames
    */

   @Test
   public void testAddExpressedInDifferentFrames()
   {
      Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
      {
         Twist twist1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());
         Twist twist2 = createSpatialMotionVector(frameB, frameA, frameA, new Vector3D(), new Vector3D());

         twist1.add(twist2);
      });
   }

   /**
    * You shouldn't be able to add two twists if the second is not relative to the first
    */

   @Test
   public void testAddNotRelative()
   {
      Assertions.assertThrows(ReferenceFrameMismatchException.class, () ->
      {
         Twist twist1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());
         Twist twist2 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());

         twist1.add(twist2);
      });
   }

   /**
    * Test adding two twists, both expressed in the same reference frame, and the second relative to
    * the first (which is allowed)
    */

   @Test
   public void testAdd()
   {
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameD, angularVelocity1, linearVelocity1);

      Vector3D angularVelocity2 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity2 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist2 = new Twist(frameC, frameB, frameD, angularVelocity2, linearVelocity2);

      twist1.add(twist2);

      assertEquals(frameD, twist1.getReferenceFrame());
      assertEquals(frameA, twist1.getBaseFrame());
      assertEquals(frameC, twist1.getBodyFrame());

      angularVelocity1.add(angularVelocity2);
      linearVelocity1.add(linearVelocity2);

      double epsilon = 1e-14;
      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity1, twist1.getAngularPart(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(linearVelocity1, twist1.getLinearPart(), epsilon);

      // Should throw exception if try it the other way:

      twist1 = new Twist(frameB, frameA, frameD, angularVelocity1, linearVelocity1);
      twist2 = new Twist(frameC, frameB, frameD, angularVelocity2, linearVelocity2);

      try
      {
         twist2.add(twist1);

         throw new RuntimeException("Should not be able to add in this direction");
      }
      catch (Exception e)
      {
      }

      twist1 = new Twist(frameB, frameA, frameD, angularVelocity1, linearVelocity1);

      try
      {
         twist1.add(twist1);

         throw new RuntimeException("Should not be able to add in this direction");
      }
      catch (Exception e)
      {
      }
   }

   @Test
   public void testSub()
   {
      Random random = new Random(3454L);
      Twist twist1 = new Twist(frameB, frameA, frameD, EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
      Twist twist2 = new Twist(frameC, frameB, frameD, EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
      Twist twist3 = new Twist(twist1);
      twist3.add(twist2);

      double epsilon = 1e-15;

      Twist twist2Back = new Twist(twist3);
      twist2Back.sub(twist1);
      MecanoTestTools.assertTwistEquals(twist2, twist2Back, epsilon);

      Twist twist1Back = new Twist(twist3);
      twist1Back.sub(twist2);
      MecanoTestTools.assertTwistEquals(twist1, twist1Back, epsilon);
   }

   @Test
   public void testSubWrongExpressedInFrame()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         Twist twist1 = new Twist(frameB, frameA, frameD);
         Twist twist2 = new Twist(frameB, frameC, frameC);
         twist1.sub(twist2);
      });
   }

   @Test
   public void testSubFramesDontMatchUp()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         Twist twist1 = new Twist(frameD, frameA, frameC);
         Twist twist2 = new Twist(frameB, frameC, frameC);
         twist1.sub(twist2);
      });
   }

   /**
    * Test changing frames by comparing the results of changeExpressedInWhatReferenceFrame() with the
    * results of the 'tilde' formula from Duindam, Port-Based Modeling and Control for Efficient
    * Bipedal Walking Robots, page 25, lemma 2.8 (b)
    */

   @Test
   public void testChangeFrame()
   {
      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Twist twist1 = new Twist(frameB, frameA, frameA, angularVelocity1, linearVelocity1);

         // first transform using 2.8 (b)
         DenseMatrix64F twist1TildeInA = toTildeForm(twist1);
         RigidBodyTransform transformFromAToC = frameA.getTransformToDesiredFrame(frameC);
         DenseMatrix64F transformFromAToCMatrix = new DenseMatrix64F(4, 4);
         transformFromAToC.get(transformFromAToCMatrix);
         DenseMatrix64F transformFromAToCMatrixInv = new DenseMatrix64F(transformFromAToCMatrix);
         CommonOps.invert(transformFromAToCMatrixInv);

         DenseMatrix64F twist1TildeInC = new DenseMatrix64F(transformFromAToCMatrix);
         DenseMatrix64F temp = new DenseMatrix64F(transformFromAToCMatrix);
         CommonOps.mult(twist1TildeInC, twist1TildeInA, temp);
         CommonOps.mult(temp, transformFromAToCMatrixInv, twist1TildeInC);

         Vector3D omega1InC = new Vector3D();
         Vector3D v1InC = new Vector3D();
         fromTildeForm(twist1TildeInC, omega1InC, v1InC);

         // then transform using 2.8 (c)
         twist1.changeFrame(frameC);
         assertEquals(frameC, twist1.getReferenceFrame());

         Vector3DBasics omega1InC2 = twist1.getAngularPart();
         Vector3DBasics v1InC2 = twist1.getLinearPart();

         double epsilon = 1e-8;
         EuclidCoreTestTools.assertTuple3DEquals(omega1InC, omega1InC2, epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(v1InC, v1InC2, epsilon);
      }
   }

   @Test
   public void testChangeFrameSameFrame()
   {
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameA, angularVelocity1, linearVelocity1);
      Twist twist2 = new Twist(twist1);
      twist1.changeFrame(twist1.getReferenceFrame());

      DenseMatrix64F twist1Matrix = new DenseMatrix64F(Twist.SIZE, 1);
      twist1.get(twist1Matrix);

      DenseMatrix64F twist2Matrix = new DenseMatrix64F(Twist.SIZE, 1);
      twist2.get(twist2Matrix);

      // test that they're the same
      EjmlUnitTests.assertEquals(twist1Matrix, twist2Matrix, 0.0);
      assertEquals(twist1.getBodyFrame(), twist2.getBodyFrame());
      assertEquals(twist1.getReferenceFrame(), twist2.getReferenceFrame());
      assertEquals(twist1.getBaseFrame(), twist2.getBaseFrame());
   }

   @Test
   public void testGetMatrix()
   {
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Twist twist1 = new Twist(frameB, frameA, frameA, angularVelocity1, linearVelocity1);

      DenseMatrix64F twistMatrix = new DenseMatrix64F(6, 1);
      twist1.get(twistMatrix);

      double epsilon = 1e-14;

      assertEquals(angularVelocity1.getX(), twistMatrix.get(0, 0), epsilon);
      assertEquals(angularVelocity1.getY(), twistMatrix.get(1, 0), epsilon);
      assertEquals(angularVelocity1.getZ(), twistMatrix.get(2, 0), epsilon);
      assertEquals(linearVelocity1.getX(), twistMatrix.get(3, 0), epsilon);
      assertEquals(linearVelocity1.getY(), twistMatrix.get(4, 0), epsilon);
      assertEquals(linearVelocity1.getZ(), twistMatrix.get(5, 0), epsilon);
   }

   /**
    * This test is used to prove that the reference frame in which the linear velocity of a body fixed
    * point in computed in does not matter.
    */
   @Test
   public void testGetLinearVelocityOfPointFixedInBodyFrameComputedInDifferentFrames() throws Exception
   {
      Random random = new Random(4354L);

      for (int i = 0; i < 1000; i++)
      {
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         ReferenceFrame baseFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("baseFrame",
                                                                                                      worldFrame,
                                                                                                      EuclidCoreRandomTools.nextRigidBodyTransform(random));
         ReferenceFrame bodyFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("bodyFrame",
                                                                                                      worldFrame,
                                                                                                      EuclidCoreRandomTools.nextRigidBodyTransform(random));
         Vector3D linearVelocity = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);

         Twist twist = new Twist(bodyFrame, baseFrame, bodyFrame, angularVelocity, linearVelocity);

         FramePoint3D pointFixedInBodyFrame = new FramePoint3D(bodyFrame, EuclidCoreRandomTools.nextPoint3D(random, 1.0));
         FrameVector3D bodyFixedPointLinearVelocityInBody = new FrameVector3D();
         FrameVector3D bodyFixedPointLinearVelocityInBase = new FrameVector3D();

         // Compute the linear velocity while in bodyFrame
         pointFixedInBodyFrame.changeFrame(bodyFrame);
         twist.changeFrame(bodyFrame);
         twist.getLinearVelocityAt(pointFixedInBodyFrame, bodyFixedPointLinearVelocityInBody);

         // Compute the linear velocity while in baseFrame
         pointFixedInBodyFrame.changeFrame(baseFrame);
         twist.changeFrame(baseFrame);
         twist.getLinearVelocityAt(pointFixedInBodyFrame, bodyFixedPointLinearVelocityInBase);

         // Verify that they are the same
         bodyFixedPointLinearVelocityInBody.changeFrame(baseFrame);
         EuclidCoreTestTools.assertTuple3DEquals(bodyFixedPointLinearVelocityInBase, bodyFixedPointLinearVelocityInBody, 1.0e-12);
      }
   }

   /**
    * Converts the twist to 'tilde' form, i.e.: [tilde(omega), v; 0, 0];
    */
   private static DenseMatrix64F toTildeForm(Twist twist)
   {
      Vector3DBasics angularVelocity = twist.getAngularPart();
      Vector3DBasics linearVelocity = twist.getLinearPart();

      DenseMatrix64F ret = new DenseMatrix64F(4, 4);
      MecanoTools.toTildeForm(angularVelocity, 0, 0, ret);
      ret.set(0, 3, linearVelocity.getX());
      ret.set(1, 3, linearVelocity.getY());
      ret.set(2, 3, linearVelocity.getZ());
      return ret;
   }

   /**
    * Converts a twist in tilde form back to twist coordinates (angular velocity and linear velocity)
    */
   private static void fromTildeForm(DenseMatrix64F twistTilde, Vector3D angularVelocityToPack, Vector3D linearVelocityToPack)
   {
      linearVelocityToPack.set(twistTilde.get(0, 3), twistTilde.get(1, 3), twistTilde.get(2, 3));
      angularVelocityToPack.set(twistTilde.get(2, 1), twistTilde.get(0, 2), twistTilde.get(1, 0));
   }
}
