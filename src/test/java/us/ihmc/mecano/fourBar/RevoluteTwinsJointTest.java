package us.ihmc.mecano.fourBar;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MecanoTestTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class RevoluteTwinsJointTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;
   private static final double SMALL_EPSILON = 1.0e-10;
   private static final double MID_EPSILON = 1.0e-7;
   private static final double LARGE_EPSILON = 1.0e-5;

   public static Vector3D finiteDifference(Point3DReadOnly current, Point3DReadOnly previous, double dt)
   {
      Vector3D diff = new Vector3D();
      diff.sub(current, previous);
      diff.scale(1.0 / dt);
      return diff;
   }

   public static Vector3D finiteDifference(QuaternionReadOnly current, QuaternionReadOnly previous, double dt)
   {
      Quaternion diff = new Quaternion();
      diff.difference(previous, current);
      Vector3D angularVelocity = new Vector3D();
      diff.getRotationVector(angularVelocity);
      angularVelocity.scale(1.0 / dt);
      return angularVelocity;
   }

   public static RevoluteTwinsJoint nextRevoluteTwinsJoint(Random random, String name, Vector3DReadOnly jointAxis)
   {
      RigidBody rootBody = new RigidBody(name + "RootBody", worldFrame);
      return nextRevoluteTwinsJoint(random, name, jointAxis, rootBody);
   }

   public static RevoluteTwinsJoint nextRevoluteTwinsJoint(Random random, String name, Vector3DReadOnly jointAxis, RigidBodyBasics predecessor)
   {
      jointAxis = Axis3D.Y;
      RigidBodyTransform transformAToPredecessor = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      transformAToPredecessor.setToZero();
      RigidBodyTransform transformBToA = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      transformBToA.getRotation().setToZero();
      int actuatedJointIndex = 0;random.nextInt(2);
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

         for (int j = 0; j < ITERATIONS; j++)
         {
            double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
            double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);
            double qRange = qMax - qMin;
            qMin += 0.05 * qRange;
            qMax -= 0.05 * qRange;
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
                       String.format("Iteration: %d\nExpected:\n%s\nwas:\n%s\nDifference:\n%s",
                                     i,
                                     expectedSPrime.toString(),
                                     actualSPrime.toString(),
                                     errorSPrime.toString()));
         }
      }
   }

   @Test
   public void testJointTwist()
   {
      Random random = new Random(346346L);
      double dt = 0.5e-6;

      for (int i = 0; i < ITERATIONS; i++)
      {
         RevoluteTwinsJoint joint = nextRevoluteTwinsJoint(random, "joint" + i, EuclidCoreRandomTools.nextUnitVector3D(random));
         double qMin = Math.max(joint.getJointLimitLower(), -Math.PI);
         double qMax = Math.min(joint.getJointLimitUpper(), Math.PI);
         double qRange = qMax - qMin;
         qMin += 0.05 * qRange;
         qMax -= 0.05 * qRange;

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
         expectedTwist.getLinearPart().set(finiteDifference(poseCurr.getPosition(), posePrev.getPosition(), dt));
         expectedTwist.getAngularPart().set(finiteDifference(poseCurr.getOrientation(), posePrev.getOrientation(), dt));

         MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-8);
      }
   }
}
