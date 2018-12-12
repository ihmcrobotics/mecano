package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialMotionTest;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;

public class SpatialAccelerationTest extends SpatialMotionTest<SpatialAcceleration>
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Override
   public SpatialAcceleration createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
                                                        Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      return new SpatialAcceleration(bodyFrame, baseFrame, expressedInFrame, angularPart, linearPart);
   }

   @Override
   public SpatialAcceleration createSpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
                                                        DenseMatrix64F matrix)
   {
      return new SpatialAcceleration(bodyFrame, baseFrame, expressedInFrame, matrix);
   }

   @Test
   public void testChangeFrameUsingNumericalDifferentiationVersusAnalytical()
   {
      double epsilon = 1e-3; // needs to be pretty high, but if you decrease deltaT, you can go lower
      double deltaT = 1e-6;

      double[] linearAmplitudes = {1.0, 2.0, 3.0};
      double[] angularAmplitudes = {4.0, 5.0, 6.0};
      double[] linearFrequencies = {1.0, 2.0, 3.0};
      double[] angularFrequencies = {4.0, 5.0, 6.0};

      Vector3D previousLinearVelocity = new Vector3D();
      Vector3D previousAngularVelocity = new Vector3D();
      double tMax = 1.0;

      for (double t = 0.0; t < tMax; t += deltaT)
      {
         Vector3D linearVelocity = getSinusoidalVelocity(linearAmplitudes, linearFrequencies, t);
         Vector3D angularVelocity = getSinusoidalVelocity(angularAmplitudes, angularFrequencies, t);

         Vector3D linearAcceleration = getSinusoidalAcceleration(linearAmplitudes, linearFrequencies, t);
         Vector3D angularAcceleration = getSinusoidalAcceleration(angularAmplitudes, angularFrequencies, t);

         Twist twistInB = new Twist(frameB, frameA, frameB, angularVelocity, linearVelocity);
         Twist twistInA = new Twist(twistInB);
         twistInA.changeFrame(frameA);

         SpatialAcceleration acceleration = new SpatialAcceleration(frameB, frameA, frameB, angularAcceleration, linearAcceleration);
         acceleration.changeFrame(frameA, twistInB, twistInB);

         if (t > deltaT / 2.0) // numerically differentiating, so don't do the first step
         {
            Vector3D linearAccelerationNewFrameNumeric = numericallyDifferentiate(previousLinearVelocity, twistInA.getLinearPart(), deltaT);
            Vector3D angularAccelerationNewFrameNumeric = numericallyDifferentiate(previousAngularVelocity, twistInA.getAngularPart(), deltaT);

            Vector3DBasics linearAccelerationNewFrameAnalytic = acceleration.getLinearPart();
            Vector3DBasics angularAccelerationNewFrameAnalytic = acceleration.getAngularPart();

            EuclidCoreTestTools.assertTuple3DEquals("t = " + t, linearAccelerationNewFrameNumeric, linearAccelerationNewFrameAnalytic, epsilon);
            EuclidCoreTestTools.assertTuple3DEquals("t = " + t, angularAccelerationNewFrameNumeric, angularAccelerationNewFrameAnalytic, epsilon);
         }

         previousLinearVelocity.set(twistInA.getLinearPart());
         previousAngularVelocity.set(twistInA.getAngularPart());
      }
   }

   /**
    * Tests centripetal acceleration
    */

   @Test
   public void testGetLinearAccelerationOfPointFixedInBodyFrame()
   {
      Random random = new Random(1456L);

      SpatialAcceleration accel = new SpatialAcceleration(frameB, frameA, frameA, new Vector3D(), new Vector3D()); // zero relative acceleration
      Twist twist = new Twist(frameB, frameA, frameA, getRandomVector(random), new Vector3D()); // pure rotational velocity
      FramePoint3D pointFixedInFrameB = new FramePoint3D(frameA, getRandomVector(random));
      FrameVector3D accelerationOfPointFixedInFrameB = new FrameVector3D(ReferenceFrame.getWorldFrame());
      accel.getLinearAccelerationAt(twist, pointFixedInFrameB, accelerationOfPointFixedInFrameB);

      Vector3D expected = new Vector3D(pointFixedInFrameB);
      expected.cross(twist.getAngularPart(), expected);
      expected.cross(twist.getAngularPart(), expected);

      EuclidCoreTestTools.assertTuple3DEquals(expected, accelerationOfPointFixedInFrameB, 1e-7);
   }

   /**
    * This test is used to prove that the reference frame in which the linear acceleration of a body
    * fixed point in computed in does not matter.
    */
   @Test
   public void testGetAccelerationOfPointFixedInBodyFrameComputedInDifferentFrames() throws Exception
   {
      Random random = new Random(345345L);

      for (int i = 0; i < 1000; i++)
      {
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         ReferenceFrame baseFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("baseFrame", worldFrame,
                                                                                                      EuclidCoreRandomTools.nextRigidBodyTransform(random));
         ReferenceFrame bodyFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("bodyFrame", worldFrame,
                                                                                                      EuclidCoreRandomTools.nextRigidBodyTransform(random));

         Vector3D linearAcceleration = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3D angularAcceleration = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         SpatialAcceleration spatialAccelerationVector = new SpatialAcceleration(bodyFrame, baseFrame, bodyFrame, linearAcceleration, angularAcceleration);

         Vector3D linearVelocity = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Twist twist = new Twist(bodyFrame, baseFrame, bodyFrame, angularVelocity, linearVelocity);

         FramePoint3D pointFixedInBodyFrame = new FramePoint3D(bodyFrame, EuclidCoreRandomTools.nextPoint3D(random, 1.0));
         FrameVector3D bodyFixedPointLinearAccelerationInBody = new FrameVector3D();
         FrameVector3D bodyFixedPointLinearAccelerationInBase = new FrameVector3D();

         // Compute the linear acceleration while in bodyFrame
         pointFixedInBodyFrame.changeFrame(bodyFrame);
         spatialAccelerationVector.getLinearAccelerationAt(twist, pointFixedInBodyFrame, bodyFixedPointLinearAccelerationInBody);

         // Compute the linear acceleration while in bodyFrame
         pointFixedInBodyFrame.changeFrame(baseFrame);
         spatialAccelerationVector.changeFrame(baseFrame, twist, twist);
         twist.changeFrame(baseFrame);
         spatialAccelerationVector.getLinearAccelerationAt(twist, pointFixedInBodyFrame, bodyFixedPointLinearAccelerationInBase);

         // Verify that they are the same
         bodyFixedPointLinearAccelerationInBody.changeFrame(baseFrame);
         EuclidCoreTestTools.assertTuple3DEquals(bodyFixedPointLinearAccelerationInBase, bodyFixedPointLinearAccelerationInBody, 1.0e-12);
      }
   }

   /**
    * You shouldn't be able to add two spatial acceleration vectors expressed in different frames
    */

   @Test
   public void testAddExpressedInDifferentFrames()
   {
      Assertions.assertThrows(ReferenceFrameMismatchException.class, () -> {
         SpatialAcceleration acceleration1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());
         SpatialAcceleration acceleration2 = createSpatialMotionVector(frameB, frameA, frameA, new Vector3D(), new Vector3D());

         acceleration1.add(acceleration2);
      });
   }

   /**
    * You shouldn't be able to add two spatial acceleration vectors if the second is not relative to
    * the first
    */

   @Test
   public void testAddNotRelative()
   {
      Assertions.assertThrows(ReferenceFrameMismatchException.class, () -> {
         SpatialAcceleration acceleration1 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());
         SpatialAcceleration acceleration2 = createSpatialMotionVector(frameB, frameA, frameC, new Vector3D(), new Vector3D());

         acceleration1.add(acceleration2);
      });
   }

   /**
    * Test adding two spatial motion vectors, both expressed in the same reference frame, and the
    * second relative to the first (which is allowed)
    */

   @Test
   public void testAdd()
   {
      Vector3D angularVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity1 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      SpatialAcceleration spatialMotionVector1 = createSpatialMotionVector(frameB, frameA, frameD, angularVelocity1, linearVelocity1);

      Vector3D angularVelocity2 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      Vector3D linearVelocity2 = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      SpatialAcceleration spatialMotionVector2 = createSpatialMotionVector(frameC, frameB, frameD, angularVelocity2, linearVelocity2);

      spatialMotionVector1.add(spatialMotionVector2);

      assertEquals(frameD, spatialMotionVector1.getReferenceFrame());
      assertEquals(frameA, spatialMotionVector1.getBaseFrame());
      assertEquals(frameC, spatialMotionVector1.getBodyFrame());

      angularVelocity1.add(angularVelocity2);
      linearVelocity1.add(linearVelocity2);

      double epsilon = 1e-14;
      EuclidCoreTestTools.assertTuple3DEquals(angularVelocity1, spatialMotionVector1.getAngularPart(), epsilon);

      EuclidCoreTestTools.assertTuple3DEquals(linearVelocity1, spatialMotionVector1.getLinearPart(), epsilon);

      // Should throw exception if try it the other way:

      spatialMotionVector1 = createSpatialMotionVector(frameB, frameA, frameD, angularVelocity1, linearVelocity1);
      spatialMotionVector2 = createSpatialMotionVector(frameC, frameB, frameD, angularVelocity2, linearVelocity2);

      try
      {
         spatialMotionVector2.add(spatialMotionVector1);

         throw new RuntimeException("Should not be able to add in this direction");
      }
      catch (Exception e)
      {
      }

      spatialMotionVector1 = createSpatialMotionVector(frameB, frameA, frameD, angularVelocity1, linearVelocity1);

      try
      {
         spatialMotionVector1.add(spatialMotionVector1);

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
      SpatialAcceleration vector1 = new SpatialAcceleration(frameB, frameA, frameD, EuclidCoreRandomTools.nextVector3D(random),
                                                            EuclidCoreRandomTools.nextVector3D(random));
      SpatialAcceleration vector2 = new SpatialAcceleration(frameC, frameB, frameD, EuclidCoreRandomTools.nextVector3D(random),
                                                            EuclidCoreRandomTools.nextVector3D(random));
      SpatialAcceleration vector3 = new SpatialAcceleration(vector1);
      vector3.add(vector2);

      double epsilon = 1e-15;

      SpatialAcceleration vector2Back = new SpatialAcceleration(vector3);
      vector2Back.sub(vector1);
      MecanoTestTools.assertSpatialAccelerationEquals(vector2, vector2Back, epsilon);

      SpatialAcceleration vector1Back = new SpatialAcceleration(vector3);
      vector1Back.sub(vector2);
      MecanoTestTools.assertSpatialAccelerationEquals(vector1, vector1Back, epsilon);
   }

   @Test
   public void testSubWrongExpressedInFrame()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
         SpatialAcceleration vector1 = new SpatialAcceleration(frameB, frameA, frameD);
         SpatialAcceleration vector2 = new SpatialAcceleration(frameB, frameC, frameC);
         vector1.sub(vector2);
      });
   }

   @Test
   public void testSubFramesDontMatchUp()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
         SpatialAcceleration vector1 = new SpatialAcceleration(frameD, frameA, frameC);
         SpatialAcceleration vector2 = new SpatialAcceleration(frameB, frameC, frameC);
         vector1.sub(vector2);
      });
   }

   @Test
   public void testSetBasedOnOriginAcceleration()
   {
      SpatialAcceleration acceleration = new SpatialAcceleration(frameA, frameB, frameA);
      Twist twistOfBodyWithRespectToBase = new Twist(frameA, frameB, frameA, EuclidCoreRandomTools.nextVector3D(random),
                                                     EuclidCoreRandomTools.nextVector3D(random));
      FrameVector3D angularAcceleration = new FrameVector3D(twistOfBodyWithRespectToBase.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random));
      FrameVector3D originAcceleration = new FrameVector3D(twistOfBodyWithRespectToBase.getReferenceFrame(), EuclidCoreRandomTools.nextVector3D(random));
      acceleration.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, twistOfBodyWithRespectToBase);

      { // Trivial assertion: Verify that the setter and getter are consistent
         FrameVector3D linearAccelerationCheck = new FrameVector3D();
         acceleration.getLinearAccelerationAtBodyOrigin(twistOfBodyWithRespectToBase, linearAccelerationCheck);
         linearAccelerationCheck.changeFrame(originAcceleration.getReferenceFrame());

         EuclidCoreTestTools.assertTuple3DEquals(linearAccelerationCheck, originAcceleration, 1e-12);
      }

      FrameVector3D originAccelerationBack = new FrameVector3D(twistOfBodyWithRespectToBase.getReferenceFrame());
      FramePoint3D origin = new FramePoint3D(acceleration.getBodyFrame());
      ReferenceFrame baseFrame = acceleration.getBaseFrame();
      origin.changeFrame(baseFrame);
      acceleration.changeFrame(baseFrame, twistOfBodyWithRespectToBase, twistOfBodyWithRespectToBase);
      twistOfBodyWithRespectToBase.changeFrame(baseFrame);
      acceleration.getLinearAccelerationAt(twistOfBodyWithRespectToBase, origin, originAccelerationBack);

      originAccelerationBack.changeFrame(originAcceleration.getReferenceFrame());
      EuclidCoreTestTools.assertTuple3DEquals(originAccelerationBack, originAcceleration, 1e-12);
   }

   @Test
   public void testChangeFrameNoRelativeMotion()
   {
      ReferenceFrame bodyFrame = frameA;
      ReferenceFrame baseFrame = frameB;
      ReferenceFrame expressedInFrame = frameC;
      Twist twist = new Twist(bodyFrame, baseFrame, expressedInFrame, EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
      SpatialAcceleration acceleration = new SpatialAcceleration(bodyFrame, baseFrame, expressedInFrame, twist.getAngularPart(), twist.getLinearPart());

      twist.changeFrame(frameA);
      acceleration.changeFrame(twist.getReferenceFrame());

      double epsilon = 1e-12;
      assertEquals(twist.getReferenceFrame(), acceleration.getReferenceFrame());
      EuclidCoreTestTools.assertTuple3DEquals(twist.getAngularPart(), acceleration.getAngularPart(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(twist.getLinearPart(), acceleration.getLinearPart(), epsilon);
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(462);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame baseFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame desiredFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialAcceleration accelerationMethod1 = MecanoRandomTools.nextSpatialAcceleration(random, bodyFrame, baseFrame, initialFrame);
         SpatialAcceleration accelerationMethod2 = new SpatialAcceleration(accelerationMethod1);
         SpatialAcceleration accelerationMethod3 = new SpatialAcceleration(accelerationMethod1);
         SpatialAcceleration accelerationMethod4 = new SpatialAcceleration(accelerationMethod1);

         Twist bodyTwist = MecanoRandomTools.nextTwist(random, bodyFrame, baseFrame, initialFrame);
         Twist deltaTwist = MecanoRandomTools.nextTwist(random, initialFrame, desiredFrame, initialFrame);
         accelerationMethod1.changeFrame(desiredFrame, deltaTwist, bodyTwist);

         bodyTwist.changeFrame(desiredFrame);
         deltaTwist.changeFrame(desiredFrame);
         accelerationMethod2.changeFrame(desiredFrame, deltaTwist, bodyTwist);

         deltaTwist.invert();
         bodyTwist.changeFrame(initialFrame);
         deltaTwist.changeFrame(initialFrame);
         accelerationMethod3.changeFrame(desiredFrame, deltaTwist, bodyTwist);

         bodyTwist.changeFrame(desiredFrame);
         deltaTwist.changeFrame(desiredFrame);
         accelerationMethod4.changeFrame(desiredFrame, deltaTwist, bodyTwist);

         MecanoTestTools.assertSpatialAccelerationEquals(accelerationMethod1, accelerationMethod2, EPSILON);
         MecanoTestTools.assertSpatialAccelerationEquals(accelerationMethod1, accelerationMethod3, EPSILON);
         MecanoTestTools.assertSpatialAccelerationEquals(accelerationMethod1, accelerationMethod4, EPSILON);

         deltaTwist.changeFrame(EuclidFrameRandomTools.nextReferenceFrame(random));
         accelerationMethod1.setReferenceFrame(initialFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> accelerationMethod1.changeFrame(desiredFrame, deltaTwist, bodyTwist),
                                                 ReferenceFrameMismatchException.class);

         bodyTwist.setReferenceFrame(initialFrame);
         deltaTwist.setReferenceFrame(desiredFrame);
         accelerationMethod1.setReferenceFrame(initialFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> accelerationMethod1.changeFrame(desiredFrame, deltaTwist, bodyTwist),
                                                 ReferenceFrameMismatchException.class);

         bodyTwist.setReferenceFrame(desiredFrame);
         deltaTwist.setReferenceFrame(initialFrame);
         accelerationMethod1.setReferenceFrame(initialFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> accelerationMethod1.changeFrame(desiredFrame, deltaTwist, bodyTwist),
                                                 ReferenceFrameMismatchException.class);

         bodyTwist.setReferenceFrame(initialFrame);
         bodyTwist.setBodyFrame(initialFrame);
         accelerationMethod1.setReferenceFrame(initialFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> accelerationMethod1.changeFrame(desiredFrame, deltaTwist, bodyTwist),
                                                 ReferenceFrameMismatchException.class);

         bodyTwist.setBodyFrame(bodyFrame);
         bodyTwist.setBaseFrame(desiredFrame);
         accelerationMethod1.setReferenceFrame(initialFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> accelerationMethod1.changeFrame(desiredFrame, deltaTwist, bodyTwist),
                                                 ReferenceFrameMismatchException.class);

         bodyTwist.setReferenceFrame(EuclidFrameRandomTools.nextReferenceFrame(random));
         bodyTwist.setBaseFrame(baseFrame);
         accelerationMethod1.setReferenceFrame(initialFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> accelerationMethod1.changeFrame(desiredFrame, deltaTwist, bodyTwist),
                                                 ReferenceFrameMismatchException.class);
      }
   }

   private static Vector3D numericallyDifferentiate(Vector3DReadOnly previousLinearVelocity, Vector3DReadOnly linearVelocity, double deltaT)
   {
      Vector3D ret = new Vector3D(linearVelocity);
      ret.sub(previousLinearVelocity);
      ret.scale(1.0 / deltaT);

      return ret;
   }

   private static Vector3D getSinusoidalVelocity(double[] amplitudes, double[] frequencies, double t)
   {
      double[] velocities = new double[3];

      for (int i = 0; i < 3; i++)
      {
         double linearAmplitude = amplitudes[i];
         double linearFrequency = frequencies[i];

         velocities[i] = linearAmplitude * Math.sin(linearFrequency * t);
      }

      return new Vector3D(velocities);
   }

   private static Vector3D getSinusoidalAcceleration(double[] amplitudes, double[] frequencies, double t)
   {
      double[] accelerations = new double[3];

      for (int i = 0; i < 3; i++)
      {
         double linearAmplitude = amplitudes[i];
         double linearFrequency = frequencies[i];

         accelerations[i] = linearFrequency * linearAmplitude * Math.cos(linearFrequency * t);
      }

      return new Vector3D(accelerations);
   }

   private Vector3D getRandomVector(Random random)
   {
      return new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
   }
}
