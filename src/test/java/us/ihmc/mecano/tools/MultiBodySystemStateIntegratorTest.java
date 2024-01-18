package us.ihmc.mecano.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.PlanarJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.SphericalJoint;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class MultiBodySystemStateIntegratorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;
   private static final double LARGE_EPSILON = 1.0e-10;

   @Test
   public void testSixDoFJointAgainstFiniteDifference()
   {
      Random random = new Random(5464576);
      Pose3D initialPose = new Pose3D();
      Twist initialTwist = new Twist();
      SpatialAcceleration initialAcceleration = new SpatialAcceleration();

      Quaternion difference = new Quaternion();
      FrameVector3D angularVelocityFD = new FrameVector3D();
      FrameVector3D linearVelocityFD = new FrameVector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         String messagePrefix = "Iteration " + i;
         RigidBody root = new RigidBody("root", worldFrame);
         SixDoFJoint joint = new SixDoFJoint("joint", root);
         RigidBody object = new RigidBody("object", joint, new Matrix3D(1, 0, 0, 0, 1, 0, 0, 0, 1), 1.0, new Vector3D());
         Pose3DBasics jointPose = joint.getJointPose();
         FixedFrameTwistBasics jointTwist = joint.getJointTwist();
         FixedFrameSpatialAccelerationBasics jointAcceleration = joint.getJointAcceleration();
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-5, 1.0e-3);
         MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

         { // Test without velocity
            MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joint);
            double expectedKineticCoEnergy = 0.0;

            initialPose.set(jointPose);
            initialTwist.setIncludingFrame(jointTwist);
            initialAcceleration.setIncludingFrame(jointAcceleration);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            EuclidCoreTestTools.assertEquals(messagePrefix, initialPose, jointPose, EPSILON);
            MecanoTestTools.assertTwistEquals(messagePrefix, initialTwist, jointTwist, EPSILON);
            MecanoTestTools.assertSpatialAccelerationEquals(messagePrefix, initialAcceleration, jointAcceleration, EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }

         { // Test linear velocity without angular velocity
            jointTwist.getAngularPart().setToZero();
            jointTwist.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
            jointAcceleration.setToZero();
            root.updateFramesRecursively();
            double expectedKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());

            initialPose.set(jointPose);
            initialTwist.setIncludingFrame(jointTwist);
            initialAcceleration.setIncludingFrame(jointAcceleration);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            angularVelocityFD.setReferenceFrame(joint.getFrameAfterJoint());
            difference.difference(initialPose.getOrientation(), jointPose.getOrientation());
            difference.getRotationVector(angularVelocityFD);
            angularVelocityFD.scale(1.0 / dt);
            EuclidCoreTestTools.assertEquals(new Vector3D(), angularVelocityFD, LARGE_EPSILON);

            linearVelocityFD.setReferenceFrame(worldFrame);
            linearVelocityFD.sub(jointPose.getPosition(), initialPose.getPosition());
            linearVelocityFD.scale(1.0 / dt);
            linearVelocityFD.changeFrame(joint.getFrameAfterJoint());

            EuclidFrameTestTools.assertEquals(messagePrefix, angularVelocityFD, jointTwist.getAngularPart(), LARGE_EPSILON);
            EuclidFrameTestTools.assertEquals(messagePrefix, linearVelocityFD, jointTwist.getLinearPart(), LARGE_EPSILON);

            MecanoTestTools.assertTwistEquals(messagePrefix, initialTwist, jointTwist, EPSILON);
            MecanoTestTools.assertSpatialAccelerationEquals(messagePrefix, initialAcceleration, jointAcceleration, EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }

         { // Test angular velocity without linear velocity
            jointTwist.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
            jointTwist.getLinearPart().setToZero();
            jointAcceleration.setToZero();
            root.updateFramesRecursively();
            double expectedKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());

            initialPose.set(jointPose);
            initialTwist.setIncludingFrame(jointTwist);
            initialAcceleration.setIncludingFrame(jointAcceleration);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            angularVelocityFD.setReferenceFrame(joint.getFrameAfterJoint());
            difference.difference(initialPose.getOrientation(), jointPose.getOrientation());
            difference.getRotationVector(angularVelocityFD);
            angularVelocityFD.scale(1.0 / dt);

            linearVelocityFD.setReferenceFrame(worldFrame);
            linearVelocityFD.sub(jointPose.getPosition(), initialPose.getPosition());
            linearVelocityFD.scale(1.0 / dt);
            linearVelocityFD.changeFrame(joint.getFrameAfterJoint());

            EuclidFrameTestTools.assertEquals(messagePrefix, angularVelocityFD, jointTwist.getAngularPart(), LARGE_EPSILON);
            EuclidFrameTestTools.assertEquals(messagePrefix, linearVelocityFD, jointTwist.getLinearPart(), EPSILON);

            MecanoTestTools.assertSpatialAccelerationEquals(messagePrefix, initialAcceleration, jointAcceleration, EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }

         {// Test without external efforts
            MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joint);
            root.updateFramesRecursively();
            // We use this guy to evaluate Coriolis and centrifugal accelerations.
            ForwardDynamicsCalculator calculator = new ForwardDynamicsCalculator(root);
            calculator.compute();
            calculator.writeComputedJointAcceleration(joint);
            double expectedKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());

            initialPose.set(jointPose);
            initialTwist.setIncludingFrame(jointTwist);
            initialAcceleration.setIncludingFrame(jointAcceleration);
            FrameVector3D expectedLinearAccelerationInWorld = new FrameVector3D(jointAcceleration.getLinearPart());
            expectedLinearAccelerationInWorld.changeFrame(worldFrame);
            FrameVector3D actualLinearAccelerationInWorld = new FrameVector3D(worldFrame);

            SpatialVector initialVelocityInWorld = new SpatialVector(jointTwist);
            initialVelocityInWorld.changeFrame(worldFrame);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            angularVelocityFD.setReferenceFrame(joint.getFrameAfterJoint());
            difference.difference(initialPose.getOrientation(), jointPose.getOrientation());
            difference.getRotationVector(angularVelocityFD);
            angularVelocityFD.scale(1.0 / dt);

            linearVelocityFD.setReferenceFrame(worldFrame);
            linearVelocityFD.sub(jointPose.getPosition(), initialPose.getPosition());
            linearVelocityFD.scale(1.0 / dt);
            linearVelocityFD.changeFrame(joint.getFrameAfterJoint());

            EuclidFrameTestTools.assertEquals(messagePrefix, angularVelocityFD, jointTwist.getAngularPart(), LARGE_EPSILON);
            EuclidFrameTestTools.assertEquals(messagePrefix, linearVelocityFD, jointTwist.getLinearPart(), LARGE_EPSILON);

            EuclidFrameTestTools.assertEquals(messagePrefix, initialAcceleration.getAngularPart(), jointAcceleration.getAngularPart(), LARGE_EPSILON);
            actualLinearAccelerationInWorld.setMatchingFrame(jointAcceleration.getLinearPart());
            EuclidFrameTestTools.assertEquals(messagePrefix, expectedLinearAccelerationInWorld, actualLinearAccelerationInWorld, EPSILON);

            SpatialVector finalVelocityInWorld = new SpatialVector(jointTwist);
            finalVelocityInWorld.changeFrame(worldFrame);
            EuclidFrameTestTools.assertEquals(initialVelocityInWorld.getLinearPart(), finalVelocityInWorld.getLinearPart(), EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }
      }
   }

   @Test
   public void testSixDoFJointBallistic()
   { // Simulating a ball being thrown and pulled by gravity
      Random random = new Random(4366346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double gravity = EuclidCoreRandomTools.nextDouble(random, -100.0, -10.0);

         RigidBody root = new RigidBody("root", worldFrame);
         SixDoFJoint joint = new SixDoFJoint("joint", root);
         new RigidBody("object", joint, new Matrix3D(1, 0, 0, 0, 1, 0, 0, 0, 1), 1.0, new Vector3D());
         Pose3DBasics jointPose = joint.getJointPose();
         FixedFrameTwistBasics jointTwist = joint.getJointTwist();
         FixedFrameSpatialAccelerationBasics jointAcceleration = joint.getJointAcceleration();
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-5, 1.0e-3);
         MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joint);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joint);
         root.updateFramesRecursively();

         Pose3D initialPose = new Pose3D(jointPose);
         Twist initialTwist = new Twist(jointTwist);

         Point3D expectedPosition = new Point3D(jointPose.getPosition());
         FrameVector3D expectedLinearVelocity = new FrameVector3D(jointTwist.getLinearPart());
         expectedLinearVelocity.changeFrame(worldFrame);
         FrameVector3D expectedLinearAcceleration = new FrameVector3D(worldFrame, 0.0, 0.0, gravity);

         double x0 = initialPose.getX();
         double y0 = initialPose.getY();
         double z0 = initialPose.getZ();
         double zd0 = expectedLinearVelocity.getZ();

         ForwardDynamicsCalculator calculator = new ForwardDynamicsCalculator(root);
         calculator.setGravitationalAcceleration(gravity);

         FrameVector3D actualLinearVelocity = new FrameVector3D();
         FrameVector3D actualLinearAcceleration = new FrameVector3D();
         FrameVector3D expectedAngularAcceleration = new FrameVector3D(joint.getFrameAfterJoint());

         for (int j = 0; j < 1000; j++)
         {
            double t = (j + 1.0) * dt;
            String messagePrefix = "Iteration " + i + ", time " + t;

            expectedLinearVelocity.setZ(zd0 + gravity * t);
            expectedPosition.setX(x0 + expectedLinearVelocity.getX() * t);
            expectedPosition.setY(y0 + expectedLinearVelocity.getY() * t);
            expectedPosition.setZ(z0 + zd0 * t + 0.5 * gravity * t * t);

            calculator.compute();
            calculator.writeComputedJointAcceleration(joint);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            EuclidCoreTestTools.assertEquals(messagePrefix, expectedPosition, jointPose.getPosition(), EPSILON);

            EuclidFrameTestTools.assertEquals(messagePrefix, initialTwist.getAngularPart(), jointTwist.getAngularPart(), EPSILON);
            actualLinearVelocity.setMatchingFrame(jointTwist.getLinearPart());
            EuclidFrameTestTools.assertEquals(messagePrefix, expectedLinearVelocity, actualLinearVelocity, EPSILON);

            EuclidFrameTestTools.assertEquals(messagePrefix, expectedAngularAcceleration, jointAcceleration.getAngularPart(), EPSILON);
            jointAcceleration.getLinearAccelerationAtBodyOrigin(jointTwist, actualLinearAcceleration);
            actualLinearAcceleration.changeFrame(worldFrame);
            EuclidFrameTestTools.assertEquals(messagePrefix, expectedLinearAcceleration, actualLinearAcceleration, EPSILON);
         }
      }
   }

   @Test
   public void testPlanarJointAgainstFiniteDifference()
   {
      Random random = new Random(5464576);
      Pose3D initialPose = new Pose3D();
      Twist initialTwist = new Twist();
      SpatialAcceleration initialAcceleration = new SpatialAcceleration();

      Quaternion difference = new Quaternion();
      FrameVector3D angularVelocityFD = new FrameVector3D();
      FrameVector3D linearVelocityFD = new FrameVector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         String messagePrefix = "Iteration " + i;
         RigidBody root = new RigidBody("root", worldFrame);
         PlanarJoint joint = new PlanarJoint("joint", root);
         RigidBody object = new RigidBody("object", joint, new Matrix3D(1, 0, 0, 0, 1, 0, 0, 0, 1), 1.0, new Vector3D());
         Pose3DBasics jointPose = joint.getJointPose();
         FixedFrameTwistBasics jointTwist = joint.getJointTwist();
         FixedFrameSpatialAccelerationBasics jointAcceleration = joint.getJointAcceleration();
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-5, 1.0e-3);
         MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

         { // Test without velocity
            MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joint);
            double expectedKineticCoEnergy = 0.0;

            initialPose.set(jointPose);
            initialTwist.setIncludingFrame(jointTwist);
            initialAcceleration.setIncludingFrame(jointAcceleration);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            EuclidCoreTestTools.assertEquals(messagePrefix, initialPose, jointPose, EPSILON);
            MecanoTestTools.assertTwistEquals(messagePrefix, initialTwist, jointTwist, EPSILON);
            MecanoTestTools.assertSpatialAccelerationEquals(messagePrefix, initialAcceleration, jointAcceleration, EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }

         { // Test linear velocity without angular velocity
            jointTwist.getAngularPart().setToZero();
            jointTwist.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
            jointAcceleration.setToZero();
            root.updateFramesRecursively();
            double expectedKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());

            initialPose.set(jointPose);
            initialTwist.setIncludingFrame(jointTwist);
            initialAcceleration.setIncludingFrame(jointAcceleration);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            angularVelocityFD.setReferenceFrame(joint.getFrameAfterJoint());
            difference.difference(initialPose.getOrientation(), jointPose.getOrientation());
            difference.getRotationVector(angularVelocityFD);
            angularVelocityFD.scale(1.0 / dt);
            EuclidCoreTestTools.assertEquals(new Vector3D(), angularVelocityFD, LARGE_EPSILON);

            linearVelocityFD.setReferenceFrame(worldFrame);
            linearVelocityFD.sub(jointPose.getPosition(), initialPose.getPosition());
            linearVelocityFD.scale(1.0 / dt);
            linearVelocityFD.changeFrame(joint.getFrameAfterJoint());

            EuclidFrameTestTools.assertEquals(messagePrefix, angularVelocityFD, jointTwist.getAngularPart(), LARGE_EPSILON);
            EuclidFrameTestTools.assertEquals(messagePrefix, linearVelocityFD, jointTwist.getLinearPart(), LARGE_EPSILON);

            MecanoTestTools.assertTwistEquals(messagePrefix, initialTwist, jointTwist, EPSILON);
            MecanoTestTools.assertSpatialAccelerationEquals(messagePrefix, initialAcceleration, jointAcceleration, EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }

         { // Test angular velocity without linear velocity
            jointTwist.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
            jointTwist.getLinearPart().setToZero();
            jointAcceleration.setToZero();
            root.updateFramesRecursively();
            double expectedKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());

            initialPose.set(jointPose);
            initialTwist.setIncludingFrame(jointTwist);
            initialAcceleration.setIncludingFrame(jointAcceleration);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            angularVelocityFD.setReferenceFrame(joint.getFrameAfterJoint());
            difference.difference(initialPose.getOrientation(), jointPose.getOrientation());
            difference.getRotationVector(angularVelocityFD);
            angularVelocityFD.scale(1.0 / dt);

            linearVelocityFD.setReferenceFrame(worldFrame);
            linearVelocityFD.sub(jointPose.getPosition(), initialPose.getPosition());
            linearVelocityFD.scale(1.0 / dt);
            linearVelocityFD.changeFrame(joint.getFrameAfterJoint());

            EuclidFrameTestTools.assertEquals(messagePrefix, angularVelocityFD, jointTwist.getAngularPart(), LARGE_EPSILON);
            EuclidFrameTestTools.assertEquals(messagePrefix, linearVelocityFD, jointTwist.getLinearPart(), EPSILON);

            MecanoTestTools.assertSpatialAccelerationEquals(messagePrefix, initialAcceleration, jointAcceleration, EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }

         {// Test without external efforts
            MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joint);
            root.updateFramesRecursively();
            // We use this guy to evaluate Coriolis and centrifugal accelerations.
            ForwardDynamicsCalculator calculator = new ForwardDynamicsCalculator(root);
            calculator.compute();
            calculator.writeComputedJointAcceleration(joint);
            double expectedKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());

            initialPose.set(jointPose);
            initialTwist.setIncludingFrame(jointTwist);
            initialAcceleration.setIncludingFrame(jointAcceleration);
            FrameVector3D expectedLinearAccelerationInWorld = new FrameVector3D(jointAcceleration.getLinearPart());
            expectedLinearAccelerationInWorld.changeFrame(worldFrame);
            FrameVector3D actualLinearAccelerationInWorld = new FrameVector3D(worldFrame);

            SpatialVector initialVelocityInWorld = new SpatialVector(jointTwist);
            initialVelocityInWorld.changeFrame(worldFrame);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            angularVelocityFD.setReferenceFrame(joint.getFrameAfterJoint());
            difference.difference(initialPose.getOrientation(), jointPose.getOrientation());
            difference.getRotationVector(angularVelocityFD);
            angularVelocityFD.scale(1.0 / dt);

            linearVelocityFD.setReferenceFrame(worldFrame);
            linearVelocityFD.sub(jointPose.getPosition(), initialPose.getPosition());
            linearVelocityFD.scale(1.0 / dt);
            linearVelocityFD.changeFrame(joint.getFrameAfterJoint());

            EuclidFrameTestTools.assertEquals(messagePrefix, angularVelocityFD, jointTwist.getAngularPart(), LARGE_EPSILON);
            EuclidFrameTestTools.assertEquals(messagePrefix, linearVelocityFD, jointTwist.getLinearPart(), LARGE_EPSILON);

            EuclidFrameTestTools.assertEquals(messagePrefix, initialAcceleration.getAngularPart(), jointAcceleration.getAngularPart(), LARGE_EPSILON);
            actualLinearAccelerationInWorld.setMatchingFrame(jointAcceleration.getLinearPart());
            EuclidFrameTestTools.assertEquals(messagePrefix, expectedLinearAccelerationInWorld, actualLinearAccelerationInWorld, EPSILON);

            SpatialVector finalVelocityInWorld = new SpatialVector(jointTwist);
            finalVelocityInWorld.changeFrame(worldFrame);
            EuclidFrameTestTools.assertEquals(initialVelocityInWorld.getLinearPart(), finalVelocityInWorld.getLinearPart(), EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }
      }
   }

   @Test
   public void testPlanarJointBallistic()
   { // Simulating a ball being thrown and pulled by gravity
      Random random = new Random(4366346);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double gravity = EuclidCoreRandomTools.nextDouble(random, -100.0, -10.0);

         RigidBody root = new RigidBody("root", worldFrame);
         PlanarJoint joint = new PlanarJoint("joint", root);
         new RigidBody("object", joint, new Matrix3D(1, 0, 0, 0, 1, 0, 0, 0, 1), 1.0, new Vector3D());
         Pose3DBasics jointPose = joint.getJointPose();
         FixedFrameTwistBasics jointTwist = joint.getJointTwist();
         FixedFrameSpatialAccelerationBasics jointAcceleration = joint.getJointAcceleration();
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-5, 1.0e-3);
         MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joint);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joint);
         root.updateFramesRecursively();

         Pose3D initialPose = new Pose3D(jointPose);
         Twist initialTwist = new Twist(jointTwist);

         Point3D expectedPosition = new Point3D(jointPose.getPosition());
         FrameVector3D expectedLinearVelocity = new FrameVector3D(jointTwist.getLinearPart());
         expectedLinearVelocity.changeFrame(worldFrame);
         FrameVector3D expectedLinearAcceleration = new FrameVector3D(worldFrame, 0.0, 0.0, gravity);

         double x0 = initialPose.getX();
         double y0 = initialPose.getY();
         double z0 = initialPose.getZ();
         double zd0 = expectedLinearVelocity.getZ();

         ForwardDynamicsCalculator calculator = new ForwardDynamicsCalculator(root);
         calculator.setGravitationalAcceleration(gravity);

         FrameVector3D actualLinearVelocity = new FrameVector3D();
         FrameVector3D actualLinearAcceleration = new FrameVector3D();
         FrameVector3D expectedAngularAcceleration = new FrameVector3D(joint.getFrameAfterJoint());

         for (int j = 0; j < 1000; j++)
         {
            double t = (j + 1.0) * dt;
            String messagePrefix = "Iteration " + i + ", time " + t;

            expectedLinearVelocity.setZ(zd0 + gravity * t);
            expectedPosition.setX(x0 + expectedLinearVelocity.getX() * t);
            expectedPosition.setY(y0 + expectedLinearVelocity.getY() * t);
            expectedPosition.setZ(z0 + zd0 * t + 0.5 * gravity * t * t);

            calculator.compute();
            calculator.writeComputedJointAcceleration(joint);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            EuclidCoreTestTools.assertEquals(messagePrefix, expectedPosition, jointPose.getPosition(), EPSILON);

            EuclidFrameTestTools.assertEquals(messagePrefix, initialTwist.getAngularPart(), jointTwist.getAngularPart(), EPSILON);
            actualLinearVelocity.setMatchingFrame(jointTwist.getLinearPart());
            EuclidFrameTestTools.assertEquals(messagePrefix, expectedLinearVelocity, actualLinearVelocity, EPSILON);

            EuclidFrameTestTools.assertEquals(messagePrefix, expectedAngularAcceleration, jointAcceleration.getAngularPart(), EPSILON);
            jointAcceleration.getLinearAccelerationAtBodyOrigin(jointTwist, actualLinearAcceleration);
            actualLinearAcceleration.changeFrame(worldFrame);
            EuclidFrameTestTools.assertEquals(messagePrefix, expectedLinearAcceleration, actualLinearAcceleration, EPSILON);
         }
      }
   }

   @Test
   public void testSphericalJointAgainstFiniteDifference()
   {
      Random random = new Random(5464576);
      Quaternion initialOrientation = new Quaternion();
      FrameVector3D initialAngularVelocity = new FrameVector3D();
      FrameVector3D initialAngularAcceleration = new FrameVector3D();

      Quaternion difference = new Quaternion();
      FrameVector3D angularVelocityFD = new FrameVector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         String messagePrefix = "Iteration " + i;
         RigidBody root = new RigidBody("root", worldFrame);
         SphericalJoint joint = new SphericalJoint("joint", root, new RigidBodyTransform());
         RigidBody object = new RigidBody("object", joint, new Matrix3D(1, 0, 0, 0, 1, 0, 0, 0, 1), 1.0, new Vector3D());
         QuaternionBasics jointOrientation = joint.getJointOrientation();
         FixedFrameVector3DBasics jointAngularVelocity = joint.getJointAngularVelocity();
         FixedFrameVector3DBasics jointAngularAcceleration = joint.getJointAngularAcceleration();
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-5, 1.0e-3);
         MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

         { // Test without velocity
            MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joint);
            double expectedKineticCoEnergy = 0.0;

            initialOrientation.set(jointOrientation);
            initialAngularVelocity.setIncludingFrame(jointAngularVelocity);
            initialAngularAcceleration.setIncludingFrame(jointAngularAcceleration);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(messagePrefix, initialOrientation, jointOrientation, EPSILON);
            EuclidFrameTestTools.assertEquals(messagePrefix, initialAngularVelocity, jointAngularVelocity, EPSILON);
            EuclidFrameTestTools.assertEquals(messagePrefix, initialAngularAcceleration, jointAngularAcceleration, EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }

         {// Test without external efforts
            MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joint);
            root.updateFramesRecursively();
            // We use this guy to evaluate Coriolis and centrifugal accelerations.
            ForwardDynamicsCalculator calculator = new ForwardDynamicsCalculator(root);
            calculator.compute();
            calculator.writeComputedJointAcceleration(joint);
            double expectedKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());

            initialOrientation.set(jointOrientation);
            initialAngularVelocity.setIncludingFrame(jointAngularVelocity);
            initialAngularAcceleration.setIncludingFrame(jointAngularAcceleration);

            integrator.doubleIntegrateFromAcceleration(joint);
            root.updateFramesRecursively();

            angularVelocityFD.setReferenceFrame(joint.getFrameAfterJoint());
            difference.difference(initialOrientation, jointOrientation);
            difference.getRotationVector(angularVelocityFD);
            angularVelocityFD.scale(1.0 / dt);

            EuclidFrameTestTools.assertEquals(messagePrefix, angularVelocityFD, jointAngularVelocity, LARGE_EPSILON);

            EuclidFrameTestTools.assertEquals(messagePrefix, initialAngularAcceleration, jointAngularAcceleration, LARGE_EPSILON);

            double actualKineticCoEnergy = object.getInertia().computeKineticCoEnergy(object.getBodyFixedFrame().getTwistOfFrame());
            assertEquals(expectedKineticCoEnergy, actualKineticCoEnergy, EPSILON);
         }
      }
   }
}
