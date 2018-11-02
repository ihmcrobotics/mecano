package us.ihmc.mecano.algorithms;

import static org.junit.Assert.*;

import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class GeometricJacobianCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @After
   public void clearFrames()
   {
      worldFrame.clearChildren();
   }

   @Test
   public void testBasicFeatures() throws Exception
   {
      Random random = new Random(435435L);
      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

      verifyThatHasBeenCleared(jacobianCalculator);

      // Setting random information to make sure it is being saved in the calculator.
      int numberOfJoints = 10;
      List<? extends Joint> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

      jacobianCalculator.setKinematicChain(joints.get(0).getPredecessor(), joints.get(numberOfJoints - 1).getSuccessor());

      { // Just checking the matrix sizing is correct
         DenseMatrix64F jacobianMatrix = new DenseMatrix64F(jacobianCalculator.getJacobianMatrix());
         assertEquals(6, jacobianMatrix.getNumRows());
         assertEquals(numberOfJoints, jacobianMatrix.getNumCols());

         DenseMatrix64F convectiveTerm = new DenseMatrix64F(jacobianCalculator.getConvectiveTermMatrix());
         assertEquals(6, convectiveTerm.getNumRows());
         assertEquals(1, convectiveTerm.getNumCols());
      }

      jacobianCalculator.clear();

      verifyThatHasBeenCleared(jacobianCalculator);

      // Do the same thing but now using setKinematicChain(OneDoFJoint[])
      jacobianCalculator.setKinematicChain(joints.toArray(new Joint[0]));

      { // Just checking the matrix sizing is correct
         DenseMatrix64F jacobianMatrix = new DenseMatrix64F(jacobianCalculator.getJacobianMatrix());
         assertEquals(6, jacobianMatrix.getNumRows());
         assertEquals(numberOfJoints, jacobianMatrix.getNumCols());

         DenseMatrix64F convectiveTerm = new DenseMatrix64F(jacobianCalculator.getConvectiveTermMatrix());
         assertEquals(6, convectiveTerm.getNumRows());
         assertEquals(1, convectiveTerm.getNumCols());
      }

      jacobianCalculator.clear();

      verifyThatHasBeenCleared(jacobianCalculator);
   }

   public void verifyThatHasBeenCleared(GeometricJacobianCalculator jacobianCalculator)
   {
      {// Simply checking that it is empty
         assertNull(jacobianCalculator.getBase());
         assertNull(jacobianCalculator.getEndEffector());
         assertNull(jacobianCalculator.getJacobianFrame());
         assertTrue(jacobianCalculator.getJointsFromBaseToEndEffector().isEmpty());
         assertEquals(-1, jacobianCalculator.getNumberOfDegreesOfFreedom());
      }

      // Try some exceptions
      try
      {
         jacobianCalculator.getJacobianMatrix();
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
      try
      {
         jacobianCalculator.getConvectiveTerm();
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testAgainstTwistCalculatorChainRobot() throws Exception
   {
      Random random = new Random(4324342L);

      int numberOfJoints = random.nextInt(100);

      List<? extends Joint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getSuccessor());

      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

      for (int i = 0; i < 1000; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         rootBody.updateFramesRecursively();

         int randomEndEffectorIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(rootBody, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianTwistAgainstTwistCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-12);

         RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(randomBase, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianTwistAgainstTwistCalculator(randomBase, randomEndEffector, jacobianCalculator, 1.0e-12);

         // Test with a random Jacobian frame attached to end effector
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, 10.0));
         ReferenceFrame fixedInEndEffector = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("fixedFrame" + i,
               randomEndEffector.getBodyFixedFrame(), transformToParent);

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(rootBody, randomEndEffector);
         jacobianCalculator.setJacobianFrame(fixedInEndEffector);

         compareJacobianTwistAgainstTwistCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-12);

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(randomBase, randomEndEffector);
         jacobianCalculator.setJacobianFrame(fixedInEndEffector);

         compareJacobianTwistAgainstTwistCalculator(randomBase, randomEndEffector, jacobianCalculator, 1.0e-12);

         // Do the same thing but now using setKinematicChain(OneDoFJoint[])
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(rootBody, randomEndEffector));
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianTwistAgainstTwistCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-12);

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(randomBase, randomEndEffector));
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianTwistAgainstTwistCalculator(randomBase, randomEndEffector, jacobianCalculator, 1.0e-12);
      }
   }

   @Test
   public void testConvectiveTerm() throws Exception
   {
      Random random = new Random(345345L);

      int numberOfJoints = 100;
      List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

      for (int i = 0; i < 1000; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         rootBody.updateFramesRecursively();

         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, worldFrame, true, false);

         spatialAccelerationCalculator.reset();

         int randomEndEffectorIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();
         RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(randomBase, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());
         SpatialAcceleration actualConvectiveTerm = new SpatialAcceleration(jacobianCalculator.getConvectiveTerm());

         SpatialAccelerationReadOnly expectedConvectiveTerm = spatialAccelerationCalculator.getRelativeAcceleration(randomBase, randomEndEffector);

         MecanoTestTools.assertSpatialAccelerationEquals(expectedConvectiveTerm, actualConvectiveTerm, 1.0e-11);
      }

      for (int i = 0; i < 1000; i++)
      { // Test computing the convective term at fixed frame in the end-effector.
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         RigidBodyBasics body = joints.get(0).getPredecessor();
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(body);
         rootBody.updateFramesRecursively();

         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, worldFrame, true, false);

         spatialAccelerationCalculator.reset();

         int randomEndEffectorIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();
         RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();

         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.setTranslation(EuclidCoreRandomTools.nextPoint3D(random, 10.0));
         ReferenceFrame fixedInEndEffector = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("fixedFrame" + i,
               randomEndEffector.getBodyFixedFrame(), transformToParent);

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(randomBase, randomEndEffector);
         jacobianCalculator.setJacobianFrame(fixedInEndEffector);
         SpatialAcceleration actualConvectiveTerm = new SpatialAcceleration(jacobianCalculator.getConvectiveTerm());

         SpatialAcceleration expectedConvectiveTerm = new SpatialAcceleration(
               spatialAccelerationCalculator.getRelativeAcceleration(randomBase, randomEndEffector));
         expectedConvectiveTerm.changeFrame(fixedInEndEffector);

         MecanoTestTools.assertSpatialAccelerationEquals(expectedConvectiveTerm, actualConvectiveTerm, 1.0e-11);
      }
   }

   @Test
   public void testAgainstSpatialAccelerationCalculatorChainRobot() throws Exception
   {
      Random random = new Random(4324342L);

      int numberOfJoints = random.nextInt(100);

      List<? extends Joint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getSuccessor());

      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

      for (int i = 0; i < 1000; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);
         rootBody.updateFramesRecursively();

         int randomEndEffectorIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(rootBody, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianAccelerationAgainstSpatialAccelerationCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-10);

         RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(randomBase, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianAccelerationAgainstSpatialAccelerationCalculator(randomBase, randomEndEffector, jacobianCalculator, 1.0e-10);

         // Do the same thing but now using setKinematicChain(OneDoFJoint[])
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(rootBody, randomEndEffector));
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianAccelerationAgainstSpatialAccelerationCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-10);

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(randomBase, randomEndEffector));
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianAccelerationAgainstSpatialAccelerationCalculator(randomBase, randomEndEffector, jacobianCalculator, 1.0e-10);
      }
   }

   @Test
   public void testAgainstTwistCalculatorFloatingJointRobot() throws Exception
   {
      Random random = new Random(4324342L);

      int numberOfJoints = random.nextInt(100);
      List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getSuccessor());

      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

      for (int i = 0; i < 1000; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);
         rootBody.updateFramesRecursively();

         int randomEndEffectorIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(rootBody, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianTwistAgainstTwistCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-12);

         RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(randomBase, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianTwistAgainstTwistCalculator(randomBase, randomEndEffector, jacobianCalculator, 1.0e-12);
      }
   }

   @Test
   public void testAgainstSpatialAccelerationCalculatorFloatingJointRobot() throws Exception
   {
      Random random = new Random(4324342L);

      int numberOfJoints = random.nextInt(100);

      List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getSuccessor());

      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

      for (int i = 0; i < 1000; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);
         rootBody.updateFramesRecursively();

         int randomEndEffectorIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(rootBody, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianAccelerationAgainstSpatialAccelerationCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-10);

         RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(randomBase, randomEndEffector);
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianAccelerationAgainstSpatialAccelerationCalculator(randomBase, randomEndEffector, jacobianCalculator, 1.0e-10);
      }
   }

   public static void compareJacobianTwistAgainstTwistCalculator(RigidBodyReadOnly base, RigidBodyReadOnly endEffector,
         GeometricJacobianCalculator jacobianCalculator, double epsilon) throws AssertionError
   {
      Twist expectedTwist = new Twist();
      Twist actualTwist = new Twist();

      DenseMatrix64F jointVelocitiesMatrix = new DenseMatrix64F(jacobianCalculator.getNumberOfDegreesOfFreedom(), 1);
      MultiBodySystemTools.extractJointsState(jacobianCalculator.getJointsFromBaseToEndEffector(), JointStateType.VELOCITY, jointVelocitiesMatrix);

      endEffector.getBodyFixedFrame().getTwistRelativeToOther(base.getBodyFixedFrame(), expectedTwist);
      expectedTwist.changeFrame(jacobianCalculator.getJacobianFrame());

      jacobianCalculator.getEndEffectorTwist(jointVelocitiesMatrix, actualTwist);

      MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, epsilon);
   }

   public static void compareJacobianAccelerationAgainstSpatialAccelerationCalculator(RigidBodyReadOnly base, RigidBodyReadOnly endEffector,
         GeometricJacobianCalculator jacobianCalculator, double epsilon) throws AssertionError
   {
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(base, worldFrame);
      spatialAccelerationCalculator.reset();

      SpatialAcceleration actualAcceleration = new SpatialAcceleration();

      DenseMatrix64F jointDesiredAccelerationsMatrix = new DenseMatrix64F(jacobianCalculator.getNumberOfDegreesOfFreedom(), 1);
      MultiBodySystemTools.extractJointsState(jacobianCalculator.getJointsFromBaseToEndEffector(), JointStateType.ACCELERATION,
            jointDesiredAccelerationsMatrix);

      SpatialAccelerationReadOnly expectedAcceleration = spatialAccelerationCalculator.getRelativeAcceleration(base, endEffector);
      jacobianCalculator.getEndEffectorAcceleration(jointDesiredAccelerationsMatrix, actualAcceleration);

      MecanoTestTools.assertSpatialAccelerationEquals(expectedAcceleration, actualAcceleration, epsilon);
   }
}
