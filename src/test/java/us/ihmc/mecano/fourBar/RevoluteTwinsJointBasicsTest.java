package us.ihmc.mecano.fourBar;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixD1;
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
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteTwinsJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteTwinsJointReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoTestTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public abstract class RevoluteTwinsJointBasicsTest<J extends RevoluteTwinsJointBasics>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int ITERATIONS = 1000;

   public abstract J nextRevoluteTwinsJoint(Random random, String name);

   @Test
   public void testMotionSubspaceDot()
   {
      Random random = new Random(346346L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         J joint = nextRevoluteTwinsJoint(random, "joint" + i);
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
         J joint = nextRevoluteTwinsJoint(random, "joint" + i);
         double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
         double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);

         double q = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);

         joint.setQ(q);
         joint.updateFramesRecursively();

         FramePose3D actualPose = new FramePose3D(joint.getFrameAfterJoint());
         actualPose.changeFrame(worldFrame);

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
         expectedPose.changeFrame(worldFrame);

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
         J joint = nextRevoluteTwinsJoint(random, "joint" + i);
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
         expectedTwist.getLinearPart().setMatchingFrame(RevoluteTwinsJointBasicsTest.finiteDifference(poseCurr.getPosition(), posePrev.getPosition(), dt));
         expectedTwist.getAngularPart()
                      .setMatchingFrame(RevoluteTwinsJointBasicsTest.finiteDifference(poseCurr.getOrientation(), posePrev.getOrientation(), dt));

         MecanoTestTools.assertTwistEquals("Iteration: " + i, expectedTwist, actualTwist, 1.0e-6);

         // Test the unit-twist against the joint twist
         actualTwist.setIncludingFrame(joint.getUnitJointTwist());
         actualTwist.scale(qd);

         MecanoTestTools.assertTwistEquals("Iteration: " + i, joint.getJointTwist(), actualTwist, 1.0e-12);

         // Test the unit-successor-twist against the unit-twist
         expectedTwist.setIncludingFrame(joint.getUnitJointTwist());
         expectedTwist.changeFrame(joint.getSuccessor().getBodyFixedFrame());
         expectedTwist.setBodyFrame(joint.getSuccessor().getBodyFixedFrame());
         expectedTwist.setBaseFrame(joint.getPredecessor().getBodyFixedFrame());

         MecanoTestTools.assertTwistEquals("Iteration: " + i, expectedTwist, joint.getUnitSuccessorTwist(), 1.0e-12);

         // Test the unit-predecessor-twist
         expectedTwist.invert();
         expectedTwist.changeFrame(joint.getPredecessor().getBodyFixedFrame());

         MecanoTestTools.assertTwistEquals("Iteration: " + i, expectedTwist, joint.getUnitPredecessorTwist(), 1.0e-12);
      }
   }

   @Test
   public void testJointSpatialAcceleration()
   {
      Random random = new Random(346346L);
      double dt = 0.5e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         J joint = nextRevoluteTwinsJoint(random, "joint" + i);
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
         expectedAcceleration.set(RevoluteTwinsJointBasicsTest.finiteDifference(twistCurr, twistPrev, dt));
         expectedAcceleration.changeFrame(joint.getFrameAfterJoint(), twistCurr, twistCurr);

         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + i, expectedAcceleration, actualAcceleration, 5.0e-6);

         // Test the unit-acceleration against the joint acceleration
         actualAcceleration.setIncludingFrame(joint.getUnitJointAcceleration());
         actualAcceleration.scale(qdd);
         actualAcceleration.add((SpatialVectorReadOnly) joint.getJointBiasAcceleration());

         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + i, joint.getJointAcceleration(), actualAcceleration, 1.0e-12);

         // Test the unit-successor-acceleration against the unit-acceleration
         expectedAcceleration.setIncludingFrame(joint.getUnitJointAcceleration());
         expectedAcceleration.changeFrame(joint.getSuccessor().getBodyFixedFrame());
         expectedAcceleration.setBodyFrame(joint.getSuccessor().getBodyFixedFrame());
         expectedAcceleration.setBaseFrame(joint.getPredecessor().getBodyFixedFrame());

         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + i, expectedAcceleration, joint.getUnitSuccessorAcceleration(), 1.0e-12);

         // Test the unit-predecessor-acceleration
         expectedAcceleration.invert();
         expectedAcceleration.changeFrame(joint.getPredecessor().getBodyFixedFrame());

         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + i, expectedAcceleration, joint.getUnitPredecessorAcceleration(), 1.0e-12);
      }
   }

   @Test
   public void testJointBiasAcceleration()
   {
      Random random = new Random(346346L);
      double dt = 0.5e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         J joint = nextRevoluteTwinsJoint(random, "joint" + i);
         double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
         double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);

         double q = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);
         double qd = EuclidCoreRandomTools.nextDouble(random, 10.0);

         joint.setQ(q);
         joint.setQd(qd);
         joint.setQdd(0);
         joint.updateFramesRecursively();

         SpatialAcceleration actualAcceleration = new SpatialAcceleration(joint.getJointBiasAcceleration());
         Twist twistPrev = new Twist(joint.getJointTwist());
         twistPrev.changeFrame(joint.getFrameBeforeJoint());

         q += qd * dt;
         joint.setQ(q);
         joint.setQd(qd);
         joint.updateFramesRecursively();

         Twist twistCurr = new Twist(joint.getJointTwist());
         twistCurr.changeFrame(joint.getFrameBeforeJoint());

         SpatialAcceleration expectedAcceleration = new SpatialAcceleration(joint.getFrameAfterJoint(),
                                                                            joint.getFrameBeforeJoint(),
                                                                            joint.getFrameBeforeJoint());
         expectedAcceleration.set(RevoluteTwinsJointBasicsTest.finiteDifference(twistCurr, twistPrev, dt));
         expectedAcceleration.changeFrame(joint.getFrameAfterJoint(), joint.getJointTwist(), joint.getJointTwist());

         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + i, expectedAcceleration, actualAcceleration, 5.0e-6);

         // Test the successor bias acceleration
         expectedAcceleration.setIncludingFrame(joint.getJointBiasAcceleration());
         expectedAcceleration.changeFrame(joint.getSuccessor().getBodyFixedFrame());
         expectedAcceleration.setBodyFrame(joint.getSuccessor().getBodyFixedFrame());
         expectedAcceleration.setBaseFrame(joint.getPredecessor().getBodyFixedFrame());

         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + i, expectedAcceleration, joint.getSuccessorBiasAcceleration(), 1.0e-12);
      }
   }

   @Test
   public void testConstraint()
   {
      Random random = new Random(346346L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         J joint = nextRevoluteTwinsJoint(random, "joint" + i);

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
         RevoluteJointBasics jointActuated = joint.getActuatedJoint();
         RevoluteJointBasics jointConstrained = joint.getConstrainedJoint();
         assertEquals(jointActuated.getQ() * joint.getConstraintRatio() + joint.getConstraintOffset(), jointConstrained.getQ(), epsilon, "Iteration: " + i);
         assertEquals(jointActuated.getQd() * joint.getConstraintRatio(), jointConstrained.getQd(), epsilon, "Iteration: " + i);
         assertEquals(jointActuated.getQdd() * joint.getConstraintRatio(), jointConstrained.getQdd(), epsilon, "Iteration: " + i);
         // Only jointA can be used as a torque source
         assertEquals(jointActuated.getTau(), joint.computeActuatedJointTau(joint.getTau()), epsilon, "Iteration: " + i);
         assertEquals(0.0, jointConstrained.getTau(), epsilon);

         // Let's check with the matrices
         DMatrixRMaj qdMatrix = new DMatrixRMaj(2, 1);
         DMatrixRMaj qddMatrix = new DMatrixRMaj(2, 1);

         DMatrixRMaj ydMatrix = new DMatrixRMaj(1, 1);
         DMatrixRMaj yddMatrix = new DMatrixRMaj(1, 1);

         ydMatrix.set(0, 0, jointActuated.getQd());
         yddMatrix.set(0, 0, jointActuated.getQdd());
         CommonOps_DDRM.mult((DMatrix1Row) joint.getConstraintJacobian(), ydMatrix, qdMatrix);
         CommonOps_DDRM.mult((DMatrix1Row) joint.getConstraintJacobian(), yddMatrix, qddMatrix);
         CommonOps_DDRM.addEquals(qddMatrix, (DMatrixD1) joint.getConstraintConvectiveTerm());

         int actuatedJointIndex = joint.getActuatedJointIndex();
         assertEquals(jointActuated.getQd(), qdMatrix.get(actuatedJointIndex, 0), epsilon);
         assertEquals(jointConstrained.getQd(), qdMatrix.get(1 - actuatedJointIndex, 0), epsilon);
         assertEquals(jointActuated.getQdd(), qddMatrix.get(actuatedJointIndex, 0), epsilon);
         assertEquals(jointConstrained.getQdd(), qddMatrix.get(1 - actuatedJointIndex, 0), epsilon);
      }
   }

   @Test
   public void testJointLimits()
   {
      Random random = new Random(346346L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         J joint = nextRevoluteTwinsJoint(random, "joint" + i);
         RevoluteJointBasics jointA = joint.getJointA();
         RevoluteJointBasics jointB = joint.getJointB();
         double qMinA = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         double qRangeA = EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0 * Math.PI);
         double qMaxA = qMinA + qRangeA;
         jointA.setJointLimitLower(qMinA);
         jointA.setJointLimitUpper(qMaxA);
         double qMinB = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         double qRangeB = EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0 * Math.PI);
         double qMaxB = qMinB + qRangeB;
         jointB.setJointLimitLower(qMinB);
         jointB.setJointLimitUpper(qMaxB);

         if (RevoluteTwinsJointReadOnly.computeJointLimitLower(joint) > RevoluteTwinsJointReadOnly.computeJointLimitUpper(joint))
         {
            assertThrows(IllegalStateException.class, joint::getJointLimitLower);
            assertThrows(IllegalStateException.class, joint::getJointLimitUpper);
            continue;
         }

         double jointLimitLower = joint.getJointLimitLower();
         joint.setQ(jointLimitLower);
         joint.updateFramesRecursively();
         double qA = jointA.getQ();
         double qB = jointB.getQ();

         assertTrue(qA >= qMinA - 1.0e-12, "Iteration: " + i + ", jointA is violating its lower limit. q=" + qA + ", qMinA=" + qMinA);
         assertTrue(qA <= qMaxA + 1.0e-12, "Iteration: " + i + ", jointA is violating its upper limit. q=" + qA + ", qMaxA=" + qMaxA);
         assertTrue(qB >= qMinB - 1.0e-12, "Iteration: " + i + ", jointB is violating its lower limit. q=" + qB + ", qMinB=" + qMinB);
         assertTrue(qB <= qMaxB + 1.0e-12, "Iteration: " + i + ", jointB is violating its upper limit. q=" + qB + ", qMaxB=" + qMaxB);

         joint.setQ(joint.getJointLimitUpper());
         joint.updateFramesRecursively();
         qA = jointA.getQ();
         qB = jointB.getQ();

         assertTrue(qA >= qMinA - 1.0e-12, "Iteration: " + i + ", jointA is violating its lower limit. q=" + qA + ", qMinA=" + qMinA);
         assertTrue(qA <= qMaxA + 1.0e-12, "Iteration: " + i + ", jointA is violating its upper limit. q=" + qA + ", qMaxA=" + qMaxA);
         assertTrue(qB >= qMinB - 1.0e-12, "Iteration: " + i + ", jointB is violating its lower limit. q=" + qB + ", qMinB=" + qMinB);
         assertTrue(qB <= qMaxB + 1.0e-12, "Iteration: " + i + ", jointB is violating its upper limit. q=" + qB + ", qMaxB=" + qMaxB);
      }
   }

   @Test
   public void testJointVelocityLimits()
   {
      Random random = new Random(346346L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         J joint = nextRevoluteTwinsJoint(random, "joint" + i);
         double qDotMinA = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double qDotRangeA = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double qDotMaxA = qDotMinA + qDotRangeA;
         joint.getJointA().setVelocityLimitLower(qDotMinA);
         joint.getJointA().setVelocityLimitUpper(qDotMaxA);
         double qDotMinB = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double qDotRangeB = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double qDotMaxB = qDotMinB + qDotRangeB;
         joint.getJointB().setVelocityLimitLower(qDotMinB);
         joint.getJointB().setVelocityLimitUpper(qDotMaxB);

         if (RevoluteTwinsJointReadOnly.computeVelocityLimitLower(joint) > RevoluteTwinsJointReadOnly.computeVelocityLimitUpper(joint))
         {
            assertThrows(IllegalStateException.class, joint::getVelocityLimitLower);
            assertThrows(IllegalStateException.class, joint::getVelocityLimitUpper);
            continue;
         }

         double velocityLimitLower = joint.getVelocityLimitLower();
         joint.setQd(velocityLimitLower);
         joint.updateFramesRecursively();
         double qdA = joint.getJointA().getQd();
         double qdB = joint.getJointB().getQd();

         assertTrue(qdA >= qDotMinA - 1.0e-12, "Iteration: " + i + ", jointA is violating its lower limit. qd=" + qdA + ", qDotMinA=" + qDotMinA);
         assertTrue(qdA <= qDotMaxA + 1.0e-12, "Iteration: " + i + ", jointA is violating its upper limit. qd=" + qdA + ", qDotMaxA=" + qDotMaxA);
         assertTrue(qdB >= qDotMinB - 1.0e-12, "Iteration: " + i + ", jointB is violating its lower limit. qd=" + qdB + ", qDotMinB=" + qDotMinB);
         assertTrue(qdB <= qDotMaxB + 1.0e-12, "Iteration: " + i + ", jointB is violating its upper limit. qd=" + qdB + ", qDotMaxB=" + qDotMaxB);

         joint.setQd(joint.getVelocityLimitUpper());
         joint.updateFramesRecursively();
         qdA = joint.getJointA().getQd();
         qdB = joint.getJointB().getQd();

         assertTrue(qdA >= qDotMinA - 1.0e-12, "Iteration: " + i + ", jointA is violating its lower limit. qd=" + qdA + ", qDotMinA=" + qDotMinA);
         assertTrue(qdA <= qDotMaxA + 1.0e-12, "Iteration: " + i + ", jointA is violating its upper limit. qd=" + qdA + ", qDotMaxA=" + qDotMaxA);
         assertTrue(qdB >= qDotMinB - 1.0e-12, "Iteration: " + i + ", jointB is violating its lower limit. qd=" + qdB + ", qDotMinB=" + qDotMinB);
         assertTrue(qdB <= qDotMaxB + 1.0e-12, "Iteration: " + i + ", jointB is violating its upper limit. qd=" + qdB + ", qDotMaxB=" + qDotMaxB);
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
}
