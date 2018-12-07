package us.ihmc.mecano.algorithms;

import static us.ihmc.robotics.Assert.*;

import java.util.List;
import java.util.Random;

import org.ejml.ops.MatrixFeatures;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

public class CompositeRigidBodyMassMatrixCalculatorTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testCentroidalMomentumPart()
   {
      Random random = new Random(2342);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         ReferenceFrame matrixFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         MultiBodySystemReadOnly input = MultiBodySystemReadOnly.toMultiBodySystemInput(joints);
         CentroidalMomentumRateCalculator centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(input, matrixFrame);
         CompositeRigidBodyMassMatrixCalculator compositeRigidBodyMassMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(input, matrixFrame);

         assertTrue(MatrixFeatures.isEquals(centroidalMomentumRateCalculator.getCentroidalMomentumMatrix(),
               compositeRigidBodyMassMatrixCalculator.getCentroidalMomentumMatrix(), EPSILON));
         MecanoTestTools.assertSpatialForceEquals(centroidalMomentumRateCalculator.getBiasSpatialForce(),
               compositeRigidBodyMassMatrixCalculator.getCentroidalConvectiveTerm(), EPSILON);

         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         centroidalMomentumRateCalculator.reset();
         compositeRigidBodyMassMatrixCalculator.reset();
         assertTrue(MatrixFeatures.isEquals(centroidalMomentumRateCalculator.getCentroidalMomentumMatrix(),
               compositeRigidBodyMassMatrixCalculator.getCentroidalMomentumMatrix(), EPSILON));
         MecanoTestTools.assertSpatialForceEquals(centroidalMomentumRateCalculator.getBiasSpatialForce(),
               compositeRigidBodyMassMatrixCalculator.getCentroidalConvectiveTerm(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(50) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         ReferenceFrame matrixFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         MultiBodySystemReadOnly input = MultiBodySystemReadOnly.toMultiBodySystemInput(joints);
         CentroidalMomentumRateCalculator centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(input, matrixFrame);
         CompositeRigidBodyMassMatrixCalculator compositeRigidBodyMassMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(input);
         compositeRigidBodyMassMatrixCalculator.setCentroidalMomentumFrame(matrixFrame);

         assertTrue(MatrixFeatures.isEquals(centroidalMomentumRateCalculator.getCentroidalMomentumMatrix(),
               compositeRigidBodyMassMatrixCalculator.getCentroidalMomentumMatrix(), EPSILON));
         MecanoTestTools.assertSpatialForceEquals(centroidalMomentumRateCalculator.getBiasSpatialForce(),
               compositeRigidBodyMassMatrixCalculator.getCentroidalConvectiveTerm(), EPSILON);
      }
   }
}
