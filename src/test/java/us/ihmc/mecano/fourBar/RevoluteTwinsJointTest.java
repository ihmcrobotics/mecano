package us.ihmc.mecano.fourBar;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoTestTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class RevoluteTwinsJointTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;

   @Test
   public void testMotionSubspaceDot()
   {
      Random random = new Random(346346L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RevoluteTwinsJoint joint = nextRevoluteTwinsJoint(random, "joint" + i, EuclidCoreRandomTools.nextUnitVector3D(random));
         double dt = 0.5e-6;
         DMatrixRMaj Sprev = new DMatrixRMaj(6, 1);
         DMatrixRMaj Scurr = new DMatrixRMaj(6, 1);
         DMatrixRMaj actualSPrime = new DMatrixRMaj(6, 1);
         DMatrixRMaj expectedSPrime = new DMatrixRMaj(6, 1);
         DMatrixRMaj errorSPrime = new DMatrixRMaj(6, 1);

         double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
         double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);
         double q = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);
         double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);

         joint.setQ(q);
         joint.setQd(qd);
         joint.updateFramesRecursively();

         joint.getMotionSubspace(Sprev);
         joint.getMotionSubspaceDot(actualSPrime);

         q += qd * dt;
         joint.setQ(q);
         joint.updateFramesRecursively();
         joint.getMotionSubspace(Scurr);

         CrossFourBarJointTest.numericallyDifferentiate(expectedSPrime, Sprev, Scurr, dt);

         CommonOps_DDRM.subtract(expectedSPrime, actualSPrime, errorSPrime);
         assertTrue(MatrixFeatures_DDRM.isEquals(expectedSPrime, actualSPrime, 1.0e-4),
                    String.format("Iteration: %d\nExpected:\n%s\nwas:\n%s\nDifference:\n%s", i, expectedSPrime, actualSPrime, errorSPrime));
      }
   }

   @Test
   public void testJointPose()
   {
      Random random = new Random(346346L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RevoluteTwinsJoint joint = nextRevoluteTwinsJoint(random, "joint" + i, EuclidCoreRandomTools.nextUnitVector3D(random));
         double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
         double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);

         double q = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);

         joint.setQ(q);
         joint.updateFramesRecursively();

         FramePose3D actualPose = new FramePose3D(joint.getFrameAfterJoint());
         actualPose.changeFrame(ReferenceFrame.getWorldFrame());

         // Now we make a clone of the joint exposing its internal kinematics, and we'll use that to check the pose.
         RigidBody rootClone = new RigidBody("rootClone", worldFrame);
         RevoluteJoint jointAClone = new RevoluteJoint("jointAClone",
                                                       rootClone,
                                                       joint.getJointA().getFrameBeforeJoint().getTransformToParent(),
                                                       joint.getJointA().getJointAxis());
         RigidBody bodyABClone = new RigidBody("bodyABClone",
                                               jointAClone,
                                               new Matrix3D(),
                                               0.0,
                                               joint.getJointA().getSuccessor().getBodyFixedFrame().getTransformToParent());
         RevoluteJoint jointBClone = new RevoluteJoint("jointBClone",
                                                       bodyABClone,
                                                       joint.getJointB().getFrameBeforeJoint().getTransformToParent(),
                                                       joint.getJointB().getJointAxis());
         jointAClone.setQ(joint.getJointA().getQ());
         jointBClone.setQ(joint.getJointB().getQ());
         rootClone.updateFramesRecursively();

         FramePose3D expectedPose = new FramePose3D(jointBClone.getFrameAfterJoint());
         expectedPose.changeFrame(ReferenceFrame.getWorldFrame());

         EuclidFrameTestTools.assertEquals(expectedPose, actualPose, 1.0e-12);
      }
   }

   @Test
   public void testJointTwist()
   {
      Random random = new Random(346346L);
      double dt = 1.0e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         RevoluteTwinsJoint joint = nextRevoluteTwinsJoint(random, "joint" + i, EuclidCoreRandomTools.nextUnitVector3D(random));
         double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
         double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);

         double q = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);
         double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);

         joint.setQ(q);
         joint.setQd(qd);
         joint.updateFramesRecursively();

         Twist actualTwist = new Twist(joint.getJointTwist());
         FramePose3D posePrev = new FramePose3D(joint.getFrameAfterJoint());
         posePrev.changeFrame(joint.getFrameBeforeJoint());

         q += qd * dt;
         joint.setQ(q);
         joint.updateFramesRecursively();

         FramePose3D poseCurr = new FramePose3D(joint.getFrameAfterJoint());
         poseCurr.changeFrame(joint.getFrameBeforeJoint());

         Twist expectedTwist = new Twist(joint.getFrameAfterJoint(), joint.getFrameBeforeJoint(), joint.getFrameAfterJoint());
         expectedTwist.getLinearPart().setMatchingFrame(finiteDifference(poseCurr.getPosition(), posePrev.getPosition(), dt));
         expectedTwist.getAngularPart().setMatchingFrame(finiteDifference(poseCurr.getOrientation(), posePrev.getOrientation(), dt));

         MecanoTestTools.assertTwistEquals("Iteration: " + i, expectedTwist, actualTwist, 1.0e-6);
      }
   }

   @Test
   public void testJointSpatialAcceleration()
   {
      Random random = new Random(346346L);
      double dt = 0.5e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         RevoluteTwinsJoint joint = nextRevoluteTwinsJoint(random, "joint" + i, EuclidCoreRandomTools.nextUnitVector3D(random));
         double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
         double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);

         double q = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);
         double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double qdd = EuclidCoreRandomTools.nextDouble(random, 10.0);

         joint.setQ(q);
         joint.setQd(qd);
         joint.setQdd(qdd);
         joint.updateFramesRecursively();

         SpatialAcceleration actualAcceleration = new SpatialAcceleration(joint.getJointAcceleration());
         Twist twistPrev = new Twist(joint.getJointTwist());
         twistPrev.changeFrame(joint.getFrameBeforeJoint());

         q += qd * dt + 0.5 * qdd * dt * dt;
         qd += qdd * dt;
         joint.setQ(q);
         joint.setQd(qd);
         joint.updateFramesRecursively();

         Twist twistCurr = new Twist(joint.getJointTwist());
         twistCurr.changeFrame(joint.getFrameBeforeJoint());

         SpatialAcceleration expectedAcceleration = new SpatialAcceleration(joint.getFrameAfterJoint(),
                                                                            joint.getFrameBeforeJoint(),
                                                                            joint.getFrameBeforeJoint());
         expectedAcceleration.set(finiteDifference(twistCurr, twistPrev, dt));
         expectedAcceleration.changeFrame(joint.getFrameAfterJoint(), twistCurr, twistCurr);

         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + i, expectedAcceleration, actualAcceleration, 5.0e-6);
      }
   }

   @Test
   public void testConstraint()
   {
      Random random = new Random(346346L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RevoluteTwinsJoint joint = nextRevoluteTwinsJoint(random, "joint" + i, EuclidCoreRandomTools.nextUnitVector3D(random));

         double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
         double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);
         double q = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);
         double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double qdd = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double tau = EuclidCoreRandomTools.nextDouble(random, 10.0);

         joint.setQ(q);
         joint.setQd(qd);
         joint.setQdd(qdd);
         joint.setTau(tau);
         joint.updateFramesRecursively();

         // Let's make sure the getters are returning the correct values.
         double epsilon = 1.0e-12;
         assertEquals(q, joint.getQ(), epsilon);
         assertEquals(qd, joint.getQd(), epsilon);
         assertEquals(qdd, joint.getQdd(), epsilon);
         assertEquals(tau, joint.getTau(), epsilon);

         // Let's make sure the joint is properly constrained.
         assertEquals(joint.getJointA().getQ(), joint.getJointB().getQ() * joint.getConstraintRatio() + joint.getConstraintOffset(), epsilon);
         assertEquals(joint.getJointA().getQd(), joint.getJointB().getQd() * joint.getConstraintRatio(), epsilon);
         assertEquals(joint.getJointA().getQdd(), joint.getJointB().getQdd() * joint.getConstraintRatio(), epsilon);
         // Only jointA can be used as a torque source
         assertEquals(joint.getJointA().getTau(), joint.computeActuatedJointTau(joint.getTau()), epsilon);
         assertEquals(0.0, joint.getJointB().getTau(), epsilon);

         // Let's check with the matrices
         DMatrixRMaj qdMatrix = new DMatrixRMaj(2, 1);
         DMatrixRMaj qddMatrix = new DMatrixRMaj(2, 1);

         DMatrixRMaj ydMatrix = new DMatrixRMaj(1, 1);
         DMatrixRMaj yddMatrix = new DMatrixRMaj(1, 1);

         ydMatrix.set(0, 0, joint.getActuatedJoint().getQd());
         yddMatrix.set(0, 0, joint.getActuatedJoint().getQdd());
         CommonOps_DDRM.mult(joint.getConstraintJacobian(), ydMatrix, qdMatrix);
         CommonOps_DDRM.mult(joint.getConstraintJacobian(), yddMatrix, qddMatrix);
         CommonOps_DDRM.addEquals(qddMatrix, joint.getConstraintConvectiveTerm());

         assertEquals(joint.getJointA().getQd(), qdMatrix.get(0, 0), epsilon);
         assertEquals(joint.getJointB().getQd(), qdMatrix.get(1, 0), epsilon);
         assertEquals(joint.getJointA().getQdd(), qddMatrix.get(0, 0), epsilon);
         assertEquals(joint.getJointB().getQdd(), qddMatrix.get(1, 0), epsilon);
      }
   }

   public static SpatialVector finiteDifference(SpatialVectorReadOnly current, SpatialVectorReadOnly previous, double dt)
   {
      SpatialVector diff = new SpatialVector(current.getReferenceFrame());
      diff.sub(current, previous);
      diff.scale(1.0 / dt);
      return diff;
   }

   public static FrameVector3D finiteDifference(FrameTuple3DReadOnly current, FrameTuple3DReadOnly previous, double dt)
   {
      FrameVector3D diff = new FrameVector3D(current.getReferenceFrame());
      diff.sub(current, previous);
      diff.scale(1.0 / dt);
      return diff;
   }

   public static FrameVector3D finiteDifference(FrameQuaternionReadOnly current, FrameQuaternionReadOnly previous, double dt)
   {
      FrameQuaternion diff = new FrameQuaternion(current.getReferenceFrame());
      diff.difference(previous, current);
      FrameVector3D angularVelocity = new FrameVector3D(current.getReferenceFrame());
      diff.getRotationVector(angularVelocity);
      angularVelocity.scale(1.0 / dt);
      // The angular velocity was computed in the body frame described by the current orientation.
      // Need to change it to expressed in the base frame.
      current.transform(angularVelocity);
      return angularVelocity;
   }

   public static RevoluteTwinsJoint nextRevoluteTwinsJoint(Random random, String name, Vector3DReadOnly jointAxis)
   {
      RigidBody rootBody = new RigidBody(name + "RootBody", worldFrame);
      return nextRevoluteTwinsJoint(random, name, jointAxis, rootBody);
   }

   public static RevoluteTwinsJoint nextRevoluteTwinsJoint(Random random, String name, Vector3DReadOnly jointAxis, RigidBodyBasics predecessor)
   {
      RigidBodyTransform transformAToPredecessor = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      transformAToPredecessor.setToZero();
      RigidBodyTransform transformBToA = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      //      transformBToA.getRotation().setToZero();
      int actuatedJointIndex = 0;//random.nextInt(2);
      double constraintRatio = 1.0;//EuclidCoreRandomTools.nextDouble(random, 0.25, 1.5);
      double constraintOffset = 0.0;//EuclidCoreRandomTools.nextDouble(random, -1.0, 1.0);
      return new RevoluteTwinsJoint(name,
                                    predecessor,
                                    null,
                                    null,
                                    null,
                                    transformAToPredecessor,
                                    transformBToA,
                                    null,
                                    0.0,
                                    null,
                                    actuatedJointIndex,
                                    constraintRatio,
                                    constraintOffset,
                                    jointAxis);
   }
}
