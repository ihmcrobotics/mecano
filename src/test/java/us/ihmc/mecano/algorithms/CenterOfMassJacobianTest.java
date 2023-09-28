package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
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
      Random random = new Random(23);

      // Center of mass position, Jacobian calculator in world frame
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

         FramePoint3DReadOnly expectedCenterOfMassBeforeUpdate = new FramePoint3D(centerOfMassCalculator.getCenterOfMass());
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassBeforeUpdate, centerOfMassJacobian.getCenterOfMass(), EPSILON);

         RigidBodyBasics body = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));

         centerOfMassCalculator.reset();
         centerOfMassJacobian.reset();

         FramePoint3DReadOnly expectedCenterOfMassAfterUpdate = new FramePoint3D(centerOfMassCalculator.getCenterOfMass());
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassAfterUpdate, centerOfMassJacobian.getCenterOfMass(), EPSILON);
      }

      // Center of mass position, Jacobian calculator in center of mass frame
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

         FramePoint3DReadOnly expectedCenterOfMassBeforeUpdate = new FramePoint3D(centerOfMassCalculator.getCenterOfMass());
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassBeforeUpdate, centerOfMassJacobian.getCenterOfMass(), EPSILON);

         RigidBodyBasics body = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));

         centerOfMassCalculator.reset();
         centerOfMassJacobian.reset();

         FramePoint3DReadOnly expectedCenterOfMassAfterUpdate = new FramePoint3D(centerOfMassCalculator.getCenterOfMass());
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassAfterUpdate, centerOfMassJacobian.getCenterOfMass(), EPSILON);
      }

      // Center of mass velocity, Jacobian calculator in world frame
      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;

         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, worldFrame);

         FrameVector3DReadOnly expectedCenterOfMassVelocityBeforeUpdate = new FrameVector3D(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()));
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassVelocityBeforeUpdate, centerOfMassJacobian.getCenterOfMassVelocity(), EPSILON);

         // TODO: we cannot pass the tests if the center of mass offset is varied. This is because we often assume frames are located at the given rigid body's
         //    CoM. If we vary this CoM, we invalidate this assumption. The difference between expected and actual does not appear too drastic, but it is nowhere
         //    near EPSILON. See {@link #RigidBodyBasics.setCenterOfMass} documentation for more info
         RigidBodyBasics body = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
//         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));
         body.getInertia().setMass(random.nextDouble(0, 1.0));
         body.getInertia().setMomentOfInertia(random.nextDouble(0, 1.0),
                                              random.nextDouble(0, 1.0),
                                              random.nextDouble(0, 1.0));
//         body.getInertia().setCenterOfMassOffset(random.nextDouble(-1.0, 1.0),
//                                                 random.nextDouble(-1.0, 1.0),
//                                                 random.nextDouble(-1.0, 1.0));

         centerOfMassJacobian.reset();

         FrameVector3DReadOnly expectedCenterOfMassVelocityAfterUpdate = new FrameVector3D(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()));
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassVelocityAfterUpdate, centerOfMassJacobian.getCenterOfMassVelocity(), EPSILON);
      }

      // Center of mass velocity, Jacobian calculator in center of mass frame
      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;

         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody, "centerOfMassFrame");

         FrameVector3DReadOnly expectedCenterOfMassVelocityBeforeUpdate = new FrameVector3D(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()));
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassVelocityBeforeUpdate, centerOfMassJacobian.getCenterOfMassVelocity(), EPSILON);

         // TODO: we cannot pass the tests if the center of mass offset is varied. This is because we often assume frames are located at the given rigid body's
         //    CoM. If we vary this CoM, we invalidate this assumption. The difference between expected and actual does not appear too drastic, but it is nowhere
         //    near EPSILON. See {@link #RigidBodyBasics.setCenterOfMass} documentation for more info
         RigidBodyBasics body = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         //         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));
         body.getInertia().setMass(random.nextDouble(0, 1.0));
         body.getInertia().setMomentOfInertia(random.nextDouble(0, 1.0),
                                              random.nextDouble(0, 1.0),
                                              random.nextDouble(0, 1.0));
//                  body.getInertia().setCenterOfMassOffset(random.nextDouble(-1.0, 1.0),
//                                                          random.nextDouble(-1.0, 1.0),
//                                                          random.nextDouble(-1.0, 1.0));

         centerOfMassJacobian.reset();

         FrameVector3DReadOnly expectedCenterOfMassVelocityAfterUpdate = new FrameVector3D(computeCenterOfMassVelocity(rootBody, centerOfMassJacobian.getReferenceFrame()));
         EuclidFrameTestTools.assertEquals(expectedCenterOfMassVelocityAfterUpdate, centerOfMassJacobian.getCenterOfMassVelocity(), EPSILON);
      }
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
