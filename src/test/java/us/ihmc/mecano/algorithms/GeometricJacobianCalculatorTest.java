package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
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
   private static final int ITERATIONS = 1000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testNoPathFromBaseToEndEffector()
   {
      Random random = new Random(34635);
      List<JointBasics> chain1 = MultiBodySystemRandomTools.nextJointChain(random, 20);
      List<JointBasics> chain2 = MultiBodySystemRandomTools.nextJointChain(random, 20);

      RigidBodyBasics bodyOnChain1 = chain1.get(random.nextInt(chain1.size())).getSuccessor();
      RigidBodyBasics bodyOnChain2 = chain2.get(random.nextInt(chain2.size())).getSuccessor();
      GeometricJacobianCalculator calculator = new GeometricJacobianCalculator();
      assertThrows(IllegalArgumentException.class, () -> calculator.setKinematicChain(bodyOnChain1, bodyOnChain2));
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

      RigidBodyBasics base = joints.get(0).getPredecessor();
      RigidBodyBasics endEffector = joints.get(numberOfJoints - 1).getSuccessor();
      jacobianCalculator.setKinematicChain(base, endEffector);

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
      assertTrue(base == jacobianCalculator.getBase());
      assertTrue(endEffector == jacobianCalculator.getEndEffector());

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

      for (int i = 0; i < ITERATIONS; i++)
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
         transformToParent.getTranslation().set(EuclidCoreRandomTools.nextPoint3D(random, 10.0));
         ReferenceFrame fixedInEndEffector = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("fixedFrame" + i,
                                                                                                               randomEndEffector.getBodyFixedFrame(),
                                                                                                               transformToParent);

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
         assertTrue(rootBody == jacobianCalculator.getBase());
         assertTrue(randomEndEffector == jacobianCalculator.getEndEffector());
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianTwistAgainstTwistCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-12);

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(randomBase, randomEndEffector));
         assertTrue(randomBase == jacobianCalculator.getBase());
         assertTrue(randomEndEffector == jacobianCalculator.getEndEffector());
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

      for (int i = 0; i < ITERATIONS; i++)
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

      for (int i = 0; i < ITERATIONS; i++)
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
         transformToParent.getTranslation().set(EuclidCoreRandomTools.nextPoint3D(random, 10.0));
         ReferenceFrame fixedInEndEffector = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("fixedFrame" + i,
                                                                                                               randomEndEffector.getBodyFixedFrame(),
                                                                                                               transformToParent);

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(randomBase, randomEndEffector);
         jacobianCalculator.setJacobianFrame(fixedInEndEffector);
         SpatialAcceleration actualConvectiveTerm = new SpatialAcceleration(jacobianCalculator.getConvectiveTerm());

         SpatialAcceleration expectedConvectiveTerm = new SpatialAcceleration(spatialAccelerationCalculator.getRelativeAcceleration(randomBase,
                                                                                                                                    randomEndEffector));
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

      for (int i = 0; i < ITERATIONS; i++)
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
         assertTrue(rootBody == jacobianCalculator.getBase());
         assertTrue(randomEndEffector == jacobianCalculator.getEndEffector());
         jacobianCalculator.setJacobianFrame(randomEndEffector.getBodyFixedFrame());

         compareJacobianAccelerationAgainstSpatialAccelerationCalculator(rootBody, randomEndEffector, jacobianCalculator, 1.0e-10);

         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(randomBase, randomEndEffector));
         assertTrue(randomBase == jacobianCalculator.getBase());
         assertTrue(randomEndEffector == jacobianCalculator.getEndEffector());
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

      for (int i = 0; i < ITERATIONS; i++)
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

      for (int i = 0; i < ITERATIONS; i++)
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

   /**
    * This test verifies that a Jacobian between two rigid-bodies that do not share a
    * ancestor-descendant relationship can be created.
    */
   @Test
   public void testJacobianBetweenTwoEndEffectors()
   {
      Random random = new Random(1266545L);

      { // Chain with random prismatic joints, end-effector is ancestor of base.
         int numberOfJoints = 50;

         List<? extends Joint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getSuccessor());

         GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

         for (int i = 0; i < ITERATIONS; i++)
         {
            MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
            MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
            rootBody.updateFramesRecursively();

            int randomEndEffectorIndex = random.nextInt(numberOfJoints);
            RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();

            jacobianCalculator.clear();
            jacobianCalculator.setKinematicChain(randomEndEffector, rootBody);
            jacobianCalculator.setJacobianFrame(rootBody.getBodyFixedFrame());

            compareJacobianTwistAgainstTwistCalculator(randomEndEffector, rootBody, jacobianCalculator, 1.0e-12);
            compareJacobianAccelerationAgainstSpatialAccelerationCalculator(randomEndEffector, rootBody, jacobianCalculator, 1.0e-10);

            RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
            jacobianCalculator.clear();
            jacobianCalculator.setKinematicChain(randomEndEffector, randomBase);
            jacobianCalculator.setJacobianFrame(randomBase.getBodyFixedFrame());

            compareJacobianTwistAgainstTwistCalculator(randomEndEffector, randomBase, jacobianCalculator, 1.0e-12);
            compareJacobianAccelerationAgainstSpatialAccelerationCalculator(randomEndEffector, randomBase, jacobianCalculator, 1.0e-10);

            if (MultiBodySystemTools.computeDistance(randomEndEffector, rootBody) > 1)
            {
               jacobianCalculator.clear();
               jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(randomEndEffector, rootBody));
               assertTrue(jacobianCalculator.getBase() == randomEndEffector);
               assertTrue(jacobianCalculator.getEndEffector() == rootBody);
               jacobianCalculator.setJacobianFrame(rootBody.getBodyFixedFrame());

               compareJacobianTwistAgainstTwistCalculator(randomEndEffector, rootBody, jacobianCalculator, 1.0e-12);
               compareJacobianAccelerationAgainstSpatialAccelerationCalculator(randomEndEffector, rootBody, jacobianCalculator, 1.0e-10);
            }

            if (MultiBodySystemTools.computeDistance(randomEndEffector, randomBase) > 1)
            {
               jacobianCalculator.clear();
               jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(randomEndEffector, randomBase));
               assertTrue(jacobianCalculator.getBase() == randomEndEffector);
               assertTrue(jacobianCalculator.getEndEffector() == randomBase);
               jacobianCalculator.setJacobianFrame(randomBase.getBodyFixedFrame());

               compareJacobianTwistAgainstTwistCalculator(randomEndEffector, randomBase, jacobianCalculator, 1.0e-12);
               compareJacobianAccelerationAgainstSpatialAccelerationCalculator(randomEndEffector, randomBase, jacobianCalculator, 1.0e-10);
            }
         }
      }

      { // Tree with random prismatic joints
         int numberOfJoints = 50;

         List<PrismaticJoint> joints = MultiBodySystemRandomTools.nextPrismaticJointTree(random, numberOfJoints);
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         List<RigidBodyBasics> endEffectors = Arrays.asList(MultiBodySystemTools.collectSubtreeEndEffectors(rootBody));

         GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

         for (int i = 0; i < ITERATIONS; i++)
         {
            for (JointStateType stateToRandomize : JointStateType.values())
               MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);
            rootBody.updateFramesRecursively();

            int endEffector1Index = random.nextInt(endEffectors.size());
            int endEffector2Index = random.nextInt(endEffectors.size());
            while (endEffector2Index == endEffector1Index)
               endEffector2Index = random.nextInt(endEffectors.size());

            RigidBodyBasics endEffector1 = endEffectors.get(endEffector1Index);
            RigidBodyBasics endEffector2 = endEffectors.get(endEffector2Index);

            jacobianCalculator.clear();
            jacobianCalculator.setKinematicChain(endEffector1, endEffector2);
            jacobianCalculator.setJacobianFrame(endEffector2.getBodyFixedFrame());
            compareJacobianTwistAgainstTwistCalculator(endEffector1, endEffector2, jacobianCalculator, 1.0e-12);
            compareJacobianAccelerationAgainstSpatialAccelerationCalculator(endEffector1, endEffector2, jacobianCalculator, 1.0e-10);

            if (MultiBodySystemTools.computeDistance(endEffector1, endEffector2) > 1)
            {
               jacobianCalculator.clear();
               jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(endEffector1, endEffector2));
               assertTrue(endEffector1 == jacobianCalculator.getBase());
               assertTrue(endEffector2 == jacobianCalculator.getEndEffector());
               jacobianCalculator.setJacobianFrame(endEffector2.getBodyFixedFrame());
               compareJacobianTwistAgainstTwistCalculator(endEffector1, endEffector2, jacobianCalculator, 1.0e-12);
               compareJacobianAccelerationAgainstSpatialAccelerationCalculator(endEffector1, endEffector2, jacobianCalculator, 1.0e-10);
            }
         }
      }

      { // Tree with random joints
         int numberOfJoints = 50;

         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         List<RigidBodyBasics> endEffectors = Arrays.asList(MultiBodySystemTools.collectSubtreeEndEffectors(rootBody));

         GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

         for (int i = 0; i < ITERATIONS; i++)
         {
            for (JointStateType stateToRandomize : JointStateType.values())
               MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);
            rootBody.updateFramesRecursively();

            int endEffector1Index = random.nextInt(endEffectors.size());
            int endEffector2Index = random.nextInt(endEffectors.size());
            while (endEffector2Index == endEffector1Index)
               endEffector2Index = random.nextInt(endEffectors.size());

            RigidBodyBasics endEffector1 = endEffectors.get(endEffector1Index);
            RigidBodyBasics endEffector2 = endEffectors.get(endEffector2Index);

            jacobianCalculator.clear();
            jacobianCalculator.setKinematicChain(endEffector1, endEffector2);
            jacobianCalculator.setJacobianFrame(endEffector2.getBodyFixedFrame());

            compareJacobianTwistAgainstTwistCalculator(endEffector1, endEffector2, jacobianCalculator, 1.0e-12);
            compareJacobianAccelerationAgainstSpatialAccelerationCalculator(endEffector1, endEffector2, jacobianCalculator, 1.0e-10);

            if (MultiBodySystemTools.computeDistance(endEffector1, endEffector2) > 1)
            {
               jacobianCalculator.clear();
               jacobianCalculator.setKinematicChain(MultiBodySystemTools.createJointPath(endEffector1, endEffector2));
               assertTrue(endEffector1 == jacobianCalculator.getBase());
               assertTrue(endEffector2 == jacobianCalculator.getEndEffector());
               jacobianCalculator.setJacobianFrame(endEffector2.getBodyFixedFrame());
               compareJacobianTwistAgainstTwistCalculator(endEffector1, endEffector2, jacobianCalculator, 1.0e-12);
               compareJacobianAccelerationAgainstSpatialAccelerationCalculator(endEffector1, endEffector2, jacobianCalculator, 1.0e-10);
            }
         }
      }
   }

   public static void compareJacobianTwistAgainstTwistCalculator(RigidBodyReadOnly base, RigidBodyReadOnly endEffector,
                                                                 GeometricJacobianCalculator jacobianCalculator, double epsilon)
         throws AssertionError
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
                                                                                      GeometricJacobianCalculator jacobianCalculator, double epsilon)
         throws AssertionError
   {
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(base, worldFrame);
      spatialAccelerationCalculator.reset();

      SpatialAcceleration actualAcceleration = new SpatialAcceleration();

      DenseMatrix64F jointDesiredAccelerationsMatrix = new DenseMatrix64F(jacobianCalculator.getNumberOfDegreesOfFreedom(), 1);
      MultiBodySystemTools.extractJointsState(jacobianCalculator.getJointsFromBaseToEndEffector(),
                                              JointStateType.ACCELERATION,
                                              jointDesiredAccelerationsMatrix);

      SpatialAccelerationReadOnly expectedAcceleration = spatialAccelerationCalculator.getRelativeAcceleration(base, endEffector);
      jacobianCalculator.getEndEffectorAcceleration(jointDesiredAccelerationsMatrix, actualAcceleration);

      MecanoTestTools.assertSpatialAccelerationEquals(expectedAcceleration, actualAcceleration, epsilon);
   }
}
