package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.*;

public class CenterOfMassJacobianTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 500;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testOneDoFJointChain()
   {
      Random random = new Random(34532);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, worldFrame);
         CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, worldFrame);

         assertEquals(centerOfMassCalculator.getTotalMass(), centerOfMassJacobian.getTotalMass(), EPSILON);
         EuclidFrameTestTools.assertEquals(centerOfMassCalculator.getCenterOfMass(), centerOfMassJacobian.getCenterOfMass(), EPSILON);
         EuclidFrameTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()),
                                                       centerOfMassJacobian.getCenterOfMassVelocity(),
                                                       EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, "centerOfMassFrame");
         CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, worldFrame);

         assertEquals(centerOfMassCalculator.getTotalMass(), centerOfMassJacobian.getTotalMass(), EPSILON);
         FramePoint3D centerOfMass = new FramePoint3D(centerOfMassJacobian.getCenterOfMass());
         centerOfMass.changeFrame(worldFrame);
         EuclidFrameTestTools.assertEquals(centerOfMassCalculator.getCenterOfMass(), centerOfMass, EPSILON);
         EuclidFrameTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()),
                                                       centerOfMassJacobian.getCenterOfMassVelocity(),
                                                       EPSILON);
      }
   }

   @Test
   public void testJointChain()
   {
      Random random = new Random(342532);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, worldFrame);
         CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, worldFrame);

         EuclidFrameTestTools.assertEquals(centerOfMassCalculator.getCenterOfMass(), centerOfMassJacobian.getCenterOfMass(), EPSILON);
         EuclidFrameTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()),
                                                       centerOfMassJacobian.getCenterOfMassVelocity(),
                                                       EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, "centerOfMassFrame");
         CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, worldFrame);

         assertEquals(centerOfMassCalculator.getTotalMass(), centerOfMassJacobian.getTotalMass(), EPSILON);
         FramePoint3D centerOfMass = new FramePoint3D(centerOfMassJacobian.getCenterOfMass());
         centerOfMass.changeFrame(worldFrame);
         EuclidFrameTestTools.assertEquals(centerOfMassCalculator.getCenterOfMass(), centerOfMass, EPSILON);
         EuclidFrameTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()),
                                                       centerOfMassJacobian.getCenterOfMassVelocity(),
                                                       EPSILON);
      }
   }

   @Test
   public void testOnDoFJointTree()
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, worldFrame);
         CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, worldFrame);

         EuclidFrameTestTools.assertEquals(centerOfMassCalculator.getCenterOfMass(), centerOfMassJacobian.getCenterOfMass(), EPSILON);
         EuclidFrameTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()),
                                                       centerOfMassJacobian.getCenterOfMassVelocity(),
                                                       EPSILON);

         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         rootBody.updateFramesRecursively();

         DMatrixRMaj jointVelocities = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);
         DMatrixRMaj centerOfMassVelocityMatrix = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(centerOfMassJacobian.getJacobianMatrix(), jointVelocities, centerOfMassVelocityMatrix);
         Vector3D actualVelocity = new Vector3D();
         actualVelocity.set(centerOfMassVelocityMatrix);

         EuclidCoreTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()), actualVelocity, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, "centerOfMassFrame");
         CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, worldFrame);

         FramePoint3D centerOfMass = new FramePoint3D(centerOfMassJacobian.getCenterOfMass());
         EuclidFrameTestTools.assertEquals(centerOfMassCalculator.getCenterOfMass(), centerOfMass, EPSILON);
         EuclidFrameTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()),
                                                       centerOfMassJacobian.getCenterOfMassVelocity(),
                                                       EPSILON);

         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         rootBody.updateFramesRecursively();

         DMatrixRMaj jointVelocities = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);
         DMatrixRMaj centerOfMassVelocityMatrix = new DMatrixRMaj(3, 1);
         CommonOps_DDRM.mult(centerOfMassJacobian.getJacobianMatrix(), jointVelocities, centerOfMassVelocityMatrix);
         Vector3D actualVelocity = new Vector3D();
         actualVelocity.set(centerOfMassVelocityMatrix);

         EuclidCoreTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()), actualVelocity, EPSILON);
      }
   }

   @Test
   public void testModifiedRigidBodyParameters()
   {
      Random random = new Random(45);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;

         // Create a list of joints whose rigid body parameters will be updated, and an identical list that will not be modified
         List<JointBasics> jointsToUpdate = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
//         List<JointBasics> jointsToLeave = List.of(MultiBodySystemFactories.cloneKinematicChain(jointsToUpdate.toArray(JointBasics[]::new)));
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, jointsToUpdate);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, jointsToUpdate);
//         MultiBodySystemTools.copyJointsState(jointsToUpdate, jointsToLeave, JointStateType.CONFIGURATION);
//         MultiBodySystemTools.copyJointsState(jointsToUpdate, jointsToLeave, JointStateType.VELOCITY);

         // Create a center of mass jacobian from the joint chain whose rigid body parameters will be updated
         RigidBodyBasics rootBodyToUpdate = jointsToUpdate.get(0).getPredecessor();
         rootBodyToUpdate.updateFramesRecursively();
         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBodyToUpdate, worldFrame);

         // Create a center of mass calculator that will be used to check the jacobian before the update, here we use the joint chain that we are purposefully
         // not updating
//         RigidBodyBasics rootBodyToLeave = jointsToLeave.get(0).getPredecessor();
//         rootBodyToLeave.updateFramesRecursively();
//         CenterOfMassCalculator centerOfMassCalculatorBeforeUpdate = new CenterOfMassCalculator(rootBodyToLeave, worldFrame);
//         FramePoint3DReadOnly expectedCenterOfMassBeforeUpdate = centerOfMassCalculatorBeforeUpdate.getCenterOfMass();
//         FrameVector3DReadOnly expectedCenterOfMassVelocityBeforeUpdate = computeCenterOfMassVelocity(rootBodyToLeave, centerOfMassCalculatorBeforeUpdate.getReferenceFrame());
//
//         EuclidFrameTestTools.assertEquals(expectedCenterOfMassBeforeUpdate, centerOfMassJacobian.getCenterOfMass(), EPSILON);
//         EuclidFrameTestTools.assertEquals(expectedCenterOfMassVelocityBeforeUpdate,
//                                           centerOfMassJacobian.getCenterOfMassVelocity(),
//                                           EPSILON);

         // Vary inertial parameters of random body and reset jacobian
         RigidBodyBasics body = jointsToUpdate.get(random.nextInt(0, numberOfJoints)).getPredecessor();
         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));
//         centerOfMassJacobian.reset();

         // Create another center of mass calculator that will be used to check the jacobian after the update, here we use the joint chain that we have modified
         // so that the expected values will take into account the modified rigid body
//         CenterOfMassCalculator centerOfMassCalculatorAfterUpdate = new CenterOfMassCalculator(rootBodyToUpdate, worldFrame);
//         FramePoint3DReadOnly expectedCenterOfMassAfterUpdate = centerOfMassCalculatorAfterUpdate.getCenterOfMass();
         FrameVector3DReadOnly expectedCenterOfMassVelocityAfterUpdate = computeCenterOfMassVelocity(rootBodyToUpdate, worldFrame); // centerOfMassCalculatorAfterUpdate.getReferenceFrame());

//         EuclidFrameTestTools.assertEquals(expectedCenterOfMassAfterUpdate, centerOfMassJacobian.getCenterOfMass(), EPSILON);
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassVelocityAfterUpdate,
                                           centerOfMassJacobian.getCenterOfMassVelocity(),
                                           EPSILON);
      }

//      for (int i = 0; i < ITERATIONS; i++)
//      {
//         int numberOfJoints = random.nextInt(50) + 1;
//         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
//         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
//         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
//
//         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
//         rootBody.updateFramesRecursively();
//
//         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, "centerOfMassFrame");
//         CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, worldFrame);
//
//         assertEquals(centerOfMassCalculator.getTotalMass(), centerOfMassJacobian.getTotalMass(), EPSILON);
//         FramePoint3D centerOfMass = new FramePoint3D(centerOfMassJacobian.getCenterOfMass());
//         centerOfMass.changeFrame(worldFrame);
//         EuclidFrameTestTools.assertEquals(centerOfMassCalculator.getCenterOfMass(), centerOfMass, EPSILON);
//         EuclidFrameTestTools.assertEquals(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()),
//                                           centerOfMassJacobian.getCenterOfMassVelocity(),
//                                           EPSILON);
//      }

   }

   private FrameVector3D computeCenterOfMassVelocity(RigidBodyReadOnly rootBody, ReferenceFrame referenceFrame)
   {
      FrameVector3D centerOfMassVelocity = new FrameVector3D(referenceFrame);

      for (RigidBodyReadOnly rigidBody : rootBody.subtreeIterable())
      {
         if (rigidBody.getInertia() == null)
            continue;
         FrameVector3D bodyLinearMomentum = new FrameVector3D(rigidBody.getBodyFixedFrame().getTwistOfFrame().getLinearPart());
         bodyLinearMomentum.scale(rigidBody.getInertia().getMass());
         bodyLinearMomentum.changeFrame(referenceFrame);
         centerOfMassVelocity.add(bodyLinearMomentum);
      }

      centerOfMassVelocity.scale(1.0
            / rootBody.subtreeStream().filter(body -> body.getInertia() != null).mapToDouble(body -> body.getInertia().getMass()).sum());
      return centerOfMassVelocity;
   }
}
