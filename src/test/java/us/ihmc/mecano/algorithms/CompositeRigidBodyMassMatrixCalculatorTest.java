package us.ihmc.mecano.algorithms;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

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

         assertTrue(MatrixFeatures_DDRM.isEquals(centroidalMomentumRateCalculator.getCentroidalMomentumMatrix(),
                                                 compositeRigidBodyMassMatrixCalculator.getCentroidalMomentumMatrix(),
                                                 EPSILON));
         MecanoTestTools.assertSpatialForceEquals(centroidalMomentumRateCalculator.getBiasSpatialForce(),
                                                  compositeRigidBodyMassMatrixCalculator.getCentroidalConvectiveTerm(),
                                                  EPSILON);

         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         centroidalMomentumRateCalculator.reset();
         compositeRigidBodyMassMatrixCalculator.reset();
         assertTrue(MatrixFeatures_DDRM.isEquals(centroidalMomentumRateCalculator.getCentroidalMomentumMatrix(),
                                                 compositeRigidBodyMassMatrixCalculator.getCentroidalMomentumMatrix(),
                                                 EPSILON));
         MecanoTestTools.assertSpatialForceEquals(centroidalMomentumRateCalculator.getBiasSpatialForce(),
                                                  compositeRigidBodyMassMatrixCalculator.getCentroidalConvectiveTerm(),
                                                  EPSILON);
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

         assertTrue(MatrixFeatures_DDRM.isEquals(centroidalMomentumRateCalculator.getCentroidalMomentumMatrix(),
                                                 compositeRigidBodyMassMatrixCalculator.getCentroidalMomentumMatrix(),
                                                 EPSILON));
         MecanoTestTools.assertSpatialForceEquals(centroidalMomentumRateCalculator.getBiasSpatialForce(),
                                                  compositeRigidBodyMassMatrixCalculator.getCentroidalConvectiveTerm(),
                                                  EPSILON);
      }
   }

   @Test
   public void testCoriolisMatrix()
   {
      Random random = new Random(547467);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with a joint chain
         int numberOfJoints = random.nextInt(20) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);

         MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(joints);
         input.getRootBody().updateFramesRecursively();
         CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(input);
         massMatrixCalculator.setEnableCoriolisMatrixCalculation(true);
         DMatrixRMaj jointVelocities = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);

         DMatrixRMaj actualJointTaus = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
         CommonOps_DDRM.mult(massMatrixCalculator.getCoriolisMatrix(), jointVelocities, actualJointTaus);

         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(input);
         inverseDynamicsCalculator.setConsiderJointAccelerations(false);
         inverseDynamicsCalculator.compute();
         DMatrixRMaj expectedJointTaus = inverseDynamicsCalculator.getJointTauMatrix();

         MecanoTestTools.assertDMatrixEquals("Iteration " + i, expectedJointTaus, actualJointTaus, 1.0e-11);
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      { // Test with a joint tree
         int numberOfJoints = random.nextInt(20) + 1;
         List<JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         
         MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(joints);
         input.getRootBody().updateFramesRecursively();
         CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(input);
         massMatrixCalculator.setEnableCoriolisMatrixCalculation(true);
         DMatrixRMaj jointVelocities = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocities);
         
         DMatrixRMaj actualJointTaus = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
         CommonOps_DDRM.mult(massMatrixCalculator.getCoriolisMatrix(), jointVelocities, actualJointTaus);
         
         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(input);
         inverseDynamicsCalculator.setConsiderJointAccelerations(false);
         inverseDynamicsCalculator.compute();
         DMatrixRMaj expectedJointTaus = inverseDynamicsCalculator.getJointTauMatrix();
         
         MecanoTestTools.assertDMatrixEquals("Iteration " + i, expectedJointTaus, actualJointTaus, 1.0e-11);
      }
   }
}
