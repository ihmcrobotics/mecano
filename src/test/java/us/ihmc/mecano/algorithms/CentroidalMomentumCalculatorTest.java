package us.ihmc.mecano.algorithms;

import static us.ihmc.robotics.Assert.*;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

public class CentroidalMomentumCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 500;
   private static final double EPSILON = 2.0e-10;

   @Test
   public void testMomentumWithOneDoFJointChain()
   {
      Random random = new Random(360675);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CentroidalMomentumCalculator centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootBody, worldFrame);

         MomentumReadOnly actualMomentum = centroidalMomentumCalculator.getMomentum();
         Momentum expectedMomentum = computeMomentum(rootBody, centroidalMomentumCalculator.getReferenceFrame());

         MecanoTestTools.assertMomentumEquals(expectedMomentum, actualMomentum, EPSILON);

         FrameVector3DReadOnly actualCenterOfMassVelocity = centroidalMomentumCalculator.getCenterOfMassVelocity();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, worldFrame);
         assertEquals(centerOfMassJacobian.getTotalMass(), centroidalMomentumCalculator.getTotalMass(), EPSILON);
         FrameVector3DReadOnly expectedCenterOfMassVelocity = centerOfMassJacobian.getCenterOfMassVelocity();
         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedCenterOfMassVelocity, actualCenterOfMassVelocity, EPSILON);
      }
   }

   @Test
   public void testMomentumWithOneDoFJointTree()
   {
      Random random = new Random(360675);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CentroidalMomentumCalculator centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootBody, worldFrame);

         MomentumReadOnly actualMomentum = centroidalMomentumCalculator.getMomentum();
         Momentum expectedMomentum = computeMomentum(rootBody, centroidalMomentumCalculator.getReferenceFrame());

         MecanoTestTools.assertMomentumEquals(expectedMomentum, actualMomentum, EPSILON);

         FrameVector3DReadOnly actualCenterOfMassVelocity = centroidalMomentumCalculator.getCenterOfMassVelocity();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, worldFrame);
         assertEquals(centerOfMassJacobian.getTotalMass(), centroidalMomentumCalculator.getTotalMass(), EPSILON);
         FrameVector3DReadOnly expectedCenterOfMassVelocity = centerOfMassJacobian.getCenterOfMassVelocity();
         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedCenterOfMassVelocity, actualCenterOfMassVelocity, EPSILON);
      }
   }

   @Test
   public void testMomentumWithJointChain()
   {
      Random random = new Random(360675);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CentroidalMomentumCalculator centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootBody, worldFrame);

         MomentumReadOnly actualMomentum = centroidalMomentumCalculator.getMomentum();
         Momentum expectedMomentum = computeMomentum(rootBody, centroidalMomentumCalculator.getReferenceFrame());

         MecanoTestTools.assertMomentumEquals(expectedMomentum, actualMomentum, EPSILON);

         FrameVector3DReadOnly actualCenterOfMassVelocity = centroidalMomentumCalculator.getCenterOfMassVelocity();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, worldFrame);
         assertEquals(centerOfMassJacobian.getTotalMass(), centroidalMomentumCalculator.getTotalMass(), EPSILON);
         FrameVector3DReadOnly expectedCenterOfMassVelocity = centerOfMassJacobian.getCenterOfMassVelocity();
         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedCenterOfMassVelocity, actualCenterOfMassVelocity, EPSILON);
      }
   }

   public static Momentum computeMomentum(RigidBodyReadOnly rootBody, ReferenceFrame referenceFrame)
   {
      Momentum momentum = new Momentum(referenceFrame);

      for (RigidBodyReadOnly rigidBody : rootBody.subtreeIterable())
      {
         if (rigidBody.getInertia() == null)
            continue;
         Momentum bodyMomentum = new Momentum(rigidBody.getBodyFixedFrame());
         bodyMomentum.compute(rigidBody.getInertia(), rigidBody.getBodyFixedFrame().getTwistOfFrame());
         bodyMomentum.changeFrame(momentum.getReferenceFrame());
         momentum.add(bodyMomentum);
      }

      return momentum;
   }
}
