package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.*;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

public class JointTorqueRegressorCalculatorTest
{
   private static final int STATE_ITERATIONS = 100;
   private static final int SYSTEM_ITERATIONS = 5;

   private static final double EPSILON = 1.0e-12;

   private static final int WARMUP_ITERATIONS = 100;
   private static final int ITERATIONS = 1000;

   // Simple two joint system that is useful for debugging -- keeps the matrices small and readable.
   @Test
   public void testRegressorAndParametersMatchInverseDynamicsSimple()
   {
      Random random = new Random(25);

      int numberOfJoints = 2;
      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

      for (JointStateType stateToRandomize : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

      // Create an inverse dynamics calculator to compare torque results to
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
      inverseDynamicsCalculator.setGravitationalAcceleration(-9.81);
      inverseDynamicsCalculator.compute();
      DMatrixRMaj expectedjointTau = inverseDynamicsCalculator.getJointTauMatrix();

      // We want the product of the regressor matrix and the parameter vector to equal the expected joint torques
      JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
      DMatrixRMaj parameterVector = regressorCalculator.getParameterVector();
      regressorCalculator.setGravitationalAcceleration(-9.81);
      regressorCalculator.compute();
      DMatrixRMaj regressorMatrix = regressorCalculator.getJointTorqueRegressorMatrix();

      // Compare results
      DMatrixRMaj actualJointTau = new DMatrixRMaj(numberOfJoints, 1);
      CommonOps_DDRM.mult(regressorMatrix, parameterVector, actualJointTau);
      assertEquals(expectedjointTau.getData().length, actualJointTau.getData().length);
      assertArrayEquals(expectedjointTau.getData(), actualJointTau.getData(), EPSILON);
   }

   @Test
   public void testThrowsUnsupportedOperationExceptionWithKinematicLoop()
   {
      Random random = new Random(25);

      int numberOfJoints = 10;
      List<RevoluteJoint> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);

      int loopStartIndex = 2;
      int loopEndIndex = 7;
      int kinematicLoopSize = 5;
      RigidBodyBasics loopStart = joints.get(loopStartIndex).getSuccessor();
      RigidBodyBasics loopEnd = joints.get(loopEndIndex).getSuccessor();
      List<RevoluteJoint> loop = MultiBodySystemRandomTools.nextKinematicLoopRevoluteJoints(random, "loop", loopStart, loopEnd, kinematicLoopSize);

      joints.addAll(loop);

      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);
      try
      {
         JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
         fail("Should have thrown an exception.");
      }
      catch (UnsupportedOperationException e)
      {
         // Good
      }
   }

   @Test
   public void testRegressorAndParametersMatchInverseDynamicsOneDoFJointChain()
   {
      Random random = new Random(25);

      for (int i = 0; i < SYSTEM_ITERATIONS; i++)
      {
         // Randomise the number of joints in the system
         int numberOfJoints = random.nextInt(30) + 1;  // to avoid zero
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         for (int j = 0; j < STATE_ITERATIONS; j++)
         {
            // Randomise the state of the system
            for (JointStateType stateToRandomize : JointStateType.values())
               MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

            // Create an inverse dynamics calculator to compare torque results to
            InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
            inverseDynamicsCalculator.setGravitationalAcceleration(-9.81);
            inverseDynamicsCalculator.compute();
            DMatrixRMaj expectedjointTau = inverseDynamicsCalculator.getJointTauMatrix();

            // We want the product of the regressor matrix and the parameter vector to equal the expected joint torques
            JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
            DMatrixRMaj parameterVector = regressorCalculator.getParameterVector();
            regressorCalculator.setGravitationalAcceleration(-9.81);
            regressorCalculator.compute();
            DMatrixRMaj regressorMatrix = regressorCalculator.getJointTorqueRegressorMatrix();

            // Compare results
            DMatrixRMaj actualJointTau = new DMatrixRMaj(numberOfJoints, 1);
            CommonOps_DDRM.mult(regressorMatrix, parameterVector, actualJointTau);
            assertEquals(expectedjointTau.getData().length, actualJointTau.getData().length);
            assertArrayEquals(expectedjointTau.getData(), actualJointTau.getData(), EPSILON);
         }
      }
   }

   @Test
   public void benchmarkOneDoFJointChain()
   {
      Random random = new Random(25);

      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, 30);
      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

      JointTorqueRegressorCalculator calculator = new JointTorqueRegressorCalculator(system);
      calculator.setGravitationalAcceleration(-9.81);

      long totalTime = 0L;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         calculator.compute();
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         long startTime = System.nanoTime();
         calculator.compute();
         totalTime += System.nanoTime() - startTime;
      }

      LogTools.info("1-DoF chain: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }

   @Test
   public void testRegressorAndParametersMatchInverseDynamicsFloatingOneDoFJointChain()
   {
      Random random = new Random(25);

      for (int i = 0; i < SYSTEM_ITERATIONS; i++)
      {
         // Randomise the number of joints in the system
         int numberOfJoints = random.nextInt(30) + 1;  // to avoid zero

         List<Joint> joints = new ArrayList<>();
         RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
         joints.add(new SixDoFJoint("floating", elevator));
         RigidBody floatingBody = MultiBodySystemRandomTools.nextRigidBody(random, "floatingBody", joints.get(0));
         joints.addAll(MultiBodySystemRandomTools.nextOneDoFJointChain(random, floatingBody, numberOfJoints));
         MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         for (int j = 0; j < STATE_ITERATIONS; j++)
         {
            // Randomise the state of the system
            for (JointStateType stateToRandomize : JointStateType.values())
               MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

            // Create an inverse dynamics calculator to compare torque results to
            InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
            inverseDynamicsCalculator.setGravitationalAcceleration(-9.81);
            inverseDynamicsCalculator.compute();
            DMatrixRMaj expectedjointTau = inverseDynamicsCalculator.getJointTauMatrix();

            // We want the product of the regressor matrix and the parameter vector to equal the expected joint torques
            JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
            DMatrixRMaj parameterVector = regressorCalculator.getParameterVector();
            regressorCalculator.setGravitationalAcceleration(-9.81);
            regressorCalculator.compute();
            DMatrixRMaj regressorMatrix = regressorCalculator.getJointTorqueRegressorMatrix();

            // Compare results
            DMatrixRMaj actualJointTau = new DMatrixRMaj(numberOfJoints, 1);
            CommonOps_DDRM.mult(regressorMatrix, parameterVector, actualJointTau);
            assertEquals(expectedjointTau.getData().length, actualJointTau.getData().length);
            assertArrayEquals(expectedjointTau.getData(), actualJointTau.getData(), EPSILON);
         }
      }
   }

   @Test
   public void benchmarkForFloatingOneDoFJointChain()
   {
      Random random = new Random(25);

      List<Joint> joints = new ArrayList<>();
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      joints.add(new SixDoFJoint("floating", elevator));
      RigidBody floatingBody = MultiBodySystemRandomTools.nextRigidBody(random, "floatingBody", joints.get(0));
      joints.addAll(MultiBodySystemRandomTools.nextOneDoFJointChain(random, floatingBody, 30));
      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

      JointTorqueRegressorCalculator calculator = new JointTorqueRegressorCalculator(system);
      calculator.setGravitationalAcceleration(-9.81);

      long totalTime = 0L;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         calculator.compute();
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         long startTime = System.nanoTime();
         calculator.compute();
         totalTime += System.nanoTime() - startTime;
      }

      LogTools.info("Floating 1-DoF chain: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }

   @Test
   public void testRegressorAndParametersMatchInverseDynamicsOneDoFJointTree()
   {
      Random random = new Random(25);

      for (int i = 0; i < SYSTEM_ITERATIONS; i++)
      {
         // Randomise the number of joints in the system
         int numberOfJoints = random.nextInt(30) + 1;  // to avoid zero
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         for (int j = 0; j < STATE_ITERATIONS; j++)
         {
            // Randomise the state of the system
            for (JointStateType stateToRandomize : JointStateType.values())
               MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

            // Create an inverse dynamics calculator to compare torque results to
            InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
            inverseDynamicsCalculator.setGravitationalAcceleration(-9.81);
            inverseDynamicsCalculator.compute();
            DMatrixRMaj expectedjointTau = inverseDynamicsCalculator.getJointTauMatrix();

            // We want the product of the regressor matrix and the parameter vector to equal the expected joint torques
            JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
            DMatrixRMaj parameterVector = regressorCalculator.getParameterVector();
            regressorCalculator.setGravitationalAcceleration(-9.81);
            regressorCalculator.compute();
            DMatrixRMaj regressorMatrix = regressorCalculator.getJointTorqueRegressorMatrix();

            // Compare results
            DMatrixRMaj actualJointTau = new DMatrixRMaj(numberOfJoints, 1);
            CommonOps_DDRM.mult(regressorMatrix, parameterVector, actualJointTau);
            assertEquals(expectedjointTau.getData().length, actualJointTau.getData().length);
            assertArrayEquals(expectedjointTau.getData(), actualJointTau.getData(), EPSILON);
         }
      }
   }

   @Test
   public void benchmarkForOneDoFJointTree()
   {
      Random random = new Random(25);

      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, 30);
      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

      JointTorqueRegressorCalculator calculator = new JointTorqueRegressorCalculator(system);
      calculator.setGravitationalAcceleration(-9.81);

      long totalTime = 0L;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         calculator.compute();
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         long startTime = System.nanoTime();
         calculator.compute();
         totalTime += System.nanoTime() - startTime;
      }

      LogTools.info("1-DoF tree: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }

   @Test
   public void testRegressorAndParametersMatchInverseDynamicsFloatingOneDoFJointTree()
   {
      Random random = new Random(25);

      for (int i = 0; i < SYSTEM_ITERATIONS; i++)
      {
         // Randomise the number of joints in the system
         int numberOfJoints = random.nextInt(30) + 1;  // to avoid zero

         List<Joint> joints = new ArrayList<>();
         RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
         joints.add(new SixDoFJoint("floating", elevator));
         RigidBody floatingBody = MultiBodySystemRandomTools.nextRigidBody(random, "floatingBody", joints.get(0));
         joints.addAll(MultiBodySystemRandomTools.nextOneDoFJointTree(random, floatingBody, numberOfJoints));
         MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         for (int j = 0; j < STATE_ITERATIONS; j++)
         {
            // Randomise the state of the system
            for (JointStateType stateToRandomize : JointStateType.values())
               MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

            // Create an inverse dynamics calculator to compare torque results to
            InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
            inverseDynamicsCalculator.setGravitationalAcceleration(-9.81);
            inverseDynamicsCalculator.compute();
            DMatrixRMaj expectedjointTau = inverseDynamicsCalculator.getJointTauMatrix();

            // We want the product of the regressor matrix and the parameter vector to equal the expected joint torques
            JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
            DMatrixRMaj parameterVector = regressorCalculator.getParameterVector();
            regressorCalculator.setGravitationalAcceleration(-9.81);
            regressorCalculator.compute();
            DMatrixRMaj regressorMatrix = regressorCalculator.getJointTorqueRegressorMatrix();

            // Compare results
            DMatrixRMaj actualJointTau = new DMatrixRMaj(numberOfJoints, 1);
            CommonOps_DDRM.mult(regressorMatrix, parameterVector, actualJointTau);
            assertEquals(expectedjointTau.getData().length, actualJointTau.getData().length);
            assertArrayEquals(expectedjointTau.getData(), actualJointTau.getData(), EPSILON);
         }
      }
   }

   @Test
   public void benchmarkForFloatingOneDoFJointTree()
   {
      Random random = new Random(25);

      List<Joint> joints = new ArrayList<>();
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      joints.add(new SixDoFJoint("floating", elevator));
      RigidBody floatingBody = MultiBodySystemRandomTools.nextRigidBody(random, "floatingBody", joints.get(0));
      joints.addAll(MultiBodySystemRandomTools.nextOneDoFJointTree(random, floatingBody, 30));
      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

      JointTorqueRegressorCalculator calculator = new JointTorqueRegressorCalculator(system);
      calculator.setGravitationalAcceleration(-9.81);

      long totalTime = 0L;

      for (int i = 0; i < WARMUP_ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         calculator.compute();
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

         long startTime = System.nanoTime();
         calculator.compute();
         totalTime += System.nanoTime() - startTime;
      }

      LogTools.info("Floating 1-DoF tree: Took on average per iteration: " + totalTime / 1e9 / ITERATIONS + " seconds");
   }

   @Test
   public void testRegressorAndParametersMatchInverseDynamicsOneDoFJointChainWithExternalWrenches()
   {
      Random random = new Random(25);

      for (int i = 0; i < SYSTEM_ITERATIONS; i++)
      {
         // Randomise the number of joints in the system
         int numberOfJoints = random.nextInt(30) + 1;  // to avoid zero
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         for (int j = 0; j < STATE_ITERATIONS; j++)
         {
            // Randomise the state of the system
            for (JointStateType stateToRandomize : JointStateType.values())
               MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

            // Create an inverse dynamics calculator to compare torque results to
            InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
            inverseDynamicsCalculator.setGravitationalAcceleration(-9.81);
            // Randomly pick rigid bodies to apply external wrenches to
            Map<RigidBodyReadOnly, Wrench> rigidBodiesToExternalWrenchMap = new LinkedHashMap<>();
            for (RigidBodyBasics body : system.getRootBody().subtreeArray())
            {
               if (body.getInertia() != null && random.nextBoolean())  // null check to exclude dummy base link
               {
                  Wrench wrenchToApply = MecanoRandomTools.nextWrench(random, body.getBodyFixedFrame(), body.getBodyFixedFrame());
                  inverseDynamicsCalculator.setExternalWrench(body, wrenchToApply);
                  rigidBodiesToExternalWrenchMap.put(body, wrenchToApply);
               }
            }
            inverseDynamicsCalculator.compute();
            DMatrixRMaj expectedjointTau = inverseDynamicsCalculator.getJointTauMatrix();

            // We want the product of the regressor matrix and the parameter vector to equal the expected joint torques
            JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
            DMatrixRMaj parameterVector = regressorCalculator.getParameterVector();
            regressorCalculator.setGravitationalAcceleration(-9.81);
            // Need to add in the contribution of the external wrenches we applied to the inverse dynamics calculator
            Map<RigidBodyReadOnly, DMatrixRMaj> rigidBodyToContactJacobianMap = new LinkedHashMap<>();
            GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
            DMatrixRMaj torques = new DMatrixRMaj(system.getNumberOfDoFs());
            for (RigidBodyReadOnly body : rigidBodiesToExternalWrenchMap.keySet())
            {
               jacobianCalculator.setKinematicChain(system.getRootBody(), body);
               jacobianCalculator.getJointTorques(rigidBodiesToExternalWrenchMap.get(body), torques);
               rigidBodyToContactJacobianMap.put(body, new DMatrixRMaj(torques));
               rigidBodyToContactJacobianMap.get(body).reshape(system.getNumberOfDoFs(), 1, true);
            }
            regressorCalculator.compute();
            DMatrixRMaj regressorMatrix = regressorCalculator.getJointTorqueRegressorMatrix();
            DMatrixRMaj actualJointTau = new DMatrixRMaj(numberOfJoints, 1);

            // The term corresponding to the regressor and the inertial parameters
            CommonOps_DDRM.mult(regressorMatrix, parameterVector, actualJointTau);
            // Torque contribution from the external wrenches
            for (DMatrixRMaj torque : rigidBodyToContactJacobianMap.values())
               CommonOps_DDRM.add(actualJointTau, -1.0, torque, actualJointTau);

            // Compare results
            assertEquals(expectedjointTau.getData().length, actualJointTau.getData().length);
            assertArrayEquals(expectedjointTau.getData(), actualJointTau.getData(), EPSILON);
         }
      }
   }

   @Test
   public void testSpatialInertiaParameterBasis()
   {
      DMatrixRMaj expectedBasis = new DMatrixRMaj(6, 6);
      DMatrixRMaj actualBasis = new DMatrixRMaj(6, 6);
      double[][] expectedBasisToPack;

      int numberOfJoints = 1;
      Random random = new Random(25);
      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
      MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);
      // We'll test the parameter bases on the first (and this case, only) body after the root, because the root body
      // usually is set to have null parameters and reference frames
      RigidBodyBasics body = system.findRigidBody("Body0");

      JointTorqueRegressorCalculator.SpatialInertiaParameterBasis basis = new JointTorqueRegressorCalculator.SpatialInertiaParameterBasis(body);
      for (JointTorqueRegressorCalculator.SpatialInertiaBasisOption basisOption : JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values)
         switch (basisOption)
         {
            case M ->
            {
               expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 1., 0., 0.},
                                                     {0., 0., 0., 0., 1., 0.},
                                                     {0., 0., 0., 0., 0., 1.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case MCOM_X ->
            {
               expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., -1.},
                                                     {0., 0., 0., 0., 1., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 1., 0., 0., 0.},
                                                     {0., -1., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case MCOM_Y ->
            {
               expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 1.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., -1., 0., 0.},
                                                     {0., 0., -1., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {1., 0., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case MCOM_Z ->
            {
               expectedBasisToPack = new double[][] {{0., 0., 0., 0., -1., 0.},
                                                     {0., 0., 0., 1., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 1., 0., 0., 0., 0.},
                                                     {-1., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case I_XX ->
            {
               expectedBasisToPack = new double[][] {{1., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case I_YY ->
            {
               expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                                                     {0., 1., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case I_ZZ ->
            {
               expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 1., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case I_XY ->
            {
               expectedBasisToPack = new double[][] {{0., 1., 0., 0., 0., 0.},
                                                     {1., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case I_XZ ->
            {
               expectedBasisToPack = new double[][] {{0., 0., 1., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {1., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
            case I_YZ ->
            {
               expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 1., 0., 0., 0.},
                                                     {0., 1., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.},
                                                     {0., 0., 0., 0., 0., 0.}};
               expectedBasis.set(expectedBasisToPack);
               basis.setBasis(basisOption);
               basis.get(actualBasis);
               assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
            }
         }
   }
}
