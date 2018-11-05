package us.ihmc.mecano.algorithms;

import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class CentroidalMomentumRateCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 500;
   private static final double EPSILON = 2.0e-10;
   private static final double FD_EPSILON = 5.0e-4;

   @After
   public void tearDown() throws Exception
   {
      worldFrame.clearChildren();
   }

   @Test
   public void testMomentumRateWithOneDoFJointChain()
   {
      Random random = new Random(360675);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, rootBody);
         centerOfMassFrame.update();

         CentroidalMomentumCalculator centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootBody, centerOfMassFrame);
         centroidalMomentumCalculator.reset();
         CentroidalMomentumRateCalculator centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(rootBody, centerOfMassFrame);
         centroidalMomentumRateCalculator.reset();

         MomentumReadOnly actualMomentum = centroidalMomentumRateCalculator.getMomentum();
         MomentumReadOnly expectedMomentum = centroidalMomentumCalculator.getMomentum();
         MecanoTestTools.assertMomentumEquals(expectedMomentum, actualMomentum, EPSILON);

         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, worldFrame);
         spatialAccelerationCalculator.reset();

         SpatialForceReadOnly actualMomentumRate = centroidalMomentumRateCalculator.getMomentumRate();
         SpatialForce expectedMomentumRate = computeMomentumRate(rootBody, spatialAccelerationCalculator, centroidalMomentumRateCalculator.getReferenceFrame());
         MecanoTestTools.assertSpatialVectorEquals(expectedMomentumRate, actualMomentumRate, EPSILON);
      }
   }

   @Test
   public void testMomentumRateWithOneDoFJointTree()
   {
      Random random = new Random(360675);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, rootBody);
         centerOfMassFrame.update();

         CentroidalMomentumCalculator centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootBody, centerOfMassFrame);
         centroidalMomentumCalculator.reset();
         CentroidalMomentumRateCalculator centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(rootBody, centerOfMassFrame);
         centroidalMomentumRateCalculator.reset();

         MomentumReadOnly actualMomentum = centroidalMomentumRateCalculator.getMomentum();
         MomentumReadOnly expectedMomentum = centroidalMomentumCalculator.getMomentum();
         MecanoTestTools.assertMomentumEquals(expectedMomentum, actualMomentum, EPSILON);

         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, worldFrame);
         spatialAccelerationCalculator.reset();

         SpatialForceReadOnly actualMomentumRate = centroidalMomentumRateCalculator.getMomentumRate();
         SpatialForce expectedMomentumRate = computeMomentumRate(rootBody, spatialAccelerationCalculator, centroidalMomentumRateCalculator.getReferenceFrame());
         MecanoTestTools.assertSpatialVectorEquals(expectedMomentumRate, actualMomentumRate, EPSILON);
      }
   }

   @Test
   public void testMomentumRateWithJointChain()
   {
      Random random = new Random(360675);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         rootBody.updateFramesRecursively();

         CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, rootBody);
         centerOfMassFrame.update();

         CentroidalMomentumCalculator centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootBody, centerOfMassFrame);
         centroidalMomentumCalculator.reset();
         CentroidalMomentumRateCalculator centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(rootBody, centerOfMassFrame);
         centroidalMomentumRateCalculator.reset();

         MomentumReadOnly actualMomentum = centroidalMomentumRateCalculator.getMomentum();
         MomentumReadOnly expectedMomentum = centroidalMomentumCalculator.getMomentum();
         MecanoTestTools.assertMomentumEquals(expectedMomentum, actualMomentum, EPSILON);

         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, worldFrame);
         spatialAccelerationCalculator.reset();

         SpatialForceReadOnly actualMomentumRate = centroidalMomentumRateCalculator.getMomentumRate();
         SpatialForce expectedMomentumRate = computeMomentumRate(rootBody, spatialAccelerationCalculator, centroidalMomentumRateCalculator.getReferenceFrame());
         MecanoTestTools.assertSpatialVectorEquals(expectedMomentumRate, actualMomentumRate, EPSILON);
      }
   }

   @Test
   public void testAgainsFiniteDifference()
   {
      Random random = new Random(360675);
      double dt = 1.0e-7;

      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();
      integrator.setIntegrationDT(dt);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<? extends OneDoFJointBasics> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -0.5, 0.5, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, -0.5, 0.5, joints);

         RigidBodyBasics rootBody = joints.get(0).getPredecessor();
         CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, rootBody);

         CentroidalMomentumCalculator centroidalMomentumCalculator = new CentroidalMomentumCalculator(rootBody, centerOfMassFrame);
         CentroidalMomentumRateCalculator centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(rootBody, centerOfMassFrame);

         Momentum previousMomentum = null;
         Momentum currentMomentum = null;
         SpatialForce expectedMomentumRate = new SpatialForce();

         for (int j = 0; j < 10; j++)
         {
            rootBody.updateFramesRecursively();
            centerOfMassFrame.update();

            centroidalMomentumCalculator.reset();
            centroidalMomentumRateCalculator.reset();

            currentMomentum = new Momentum(centroidalMomentumCalculator.getMomentum());

            if (previousMomentum != null)
            {
               expectedMomentumRate.setIncludingFrame(currentMomentum);
               expectedMomentumRate.sub(previousMomentum);
               expectedMomentumRate.scale(1.0 / dt);

               SpatialForceReadOnly actualMomentumRate = centroidalMomentumRateCalculator.getMomentumRate();
               MecanoTestTools.assertSpatialVectorEquals(expectedMomentumRate, actualMomentumRate, FD_EPSILON);
            }

            previousMomentum = currentMomentum;
            integrator.doubleIntegrateFromAccelerationSubtree(rootBody);
         }
      }
   }

   public static Momentum extractMomentum(List<? extends JointReadOnly> joints, CentroidalMomentumRateCalculator centroidalMomentumRateCalculator)
   {
      DenseMatrix64F jointVelocities = new DenseMatrix64F(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);

      DenseMatrix64F momentumMatrix = new DenseMatrix64F(6, 1);
      CommonOps.mult(centroidalMomentumRateCalculator.getCentroidalMomentumMatrix(), jointVelocities, momentumMatrix);

      return new Momentum(centroidalMomentumRateCalculator.getReferenceFrame(), momentumMatrix);
   }

   public static SpatialForce computeMomentumRate(RigidBodyReadOnly rootBody, SpatialAccelerationCalculator spatialAccelerationCalculator,
                                                  ReferenceFrame referenceFrame)
   {
      SpatialForce momentumRate = new SpatialForce(referenceFrame);

      for (RigidBodyReadOnly rigidBody : rootBody.subtreeIterable())
      {
         SpatialInertiaReadOnly inertia = rigidBody.getInertia();
         if (inertia == null)
            continue;

         Wrench bodyDynamicWrench = new Wrench();
         SpatialAcceleration bodyAcceleration = new SpatialAcceleration();
         bodyAcceleration.setIncludingFrame(spatialAccelerationCalculator.getAccelerationOfBody(rigidBody));
         TwistReadOnly bodyTwist = rigidBody.getBodyFixedFrame().getTwistOfFrame();

         inertia.computeDynamicWrenchFast(bodyAcceleration, bodyTwist, bodyDynamicWrench);

         bodyDynamicWrench.changeFrame(referenceFrame);
         momentumRate.add(bodyDynamicWrench);
      }

      return momentumRate;
   }
}
