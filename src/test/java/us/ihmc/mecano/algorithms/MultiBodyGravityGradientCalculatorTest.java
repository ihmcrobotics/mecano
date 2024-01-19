package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.TablePrinter.Alignment;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Function;

public class MultiBodyGravityGradientCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;
   private static final double GRAVITY = 10.0;
   private static final boolean ADD_EXT_WRENCHES = true;

   @Test
   public void testGravityMatrixAgainstInverseDynamics()
   {
      Random random = new Random(345345780);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 20;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
         MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

         InverseDynamicsCalculator inverseDynamics = new InverseDynamicsCalculator(input);
         inverseDynamics.setGravitationalAcceleration(-GRAVITY);
         inverseDynamics.setConsiderCoriolisAndCentrifugalForces(false);
         inverseDynamics.setConsiderJointAccelerations(false);

         MultiBodyGravityGradientCalculator calculator = new MultiBodyGravityGradientCalculator(input);
         calculator.setGravitionalAcceleration(-GRAVITY);

         if (ADD_EXT_WRENCHES)
         {
            if (random.nextBoolean())
            { // Add external wrench
               int numberOfWrenches = random.nextInt(3) + 1;
               for (int j = 0; j < numberOfWrenches; j++)
               {
                  int bodyIndex = random.nextInt(joints.size());
                  RigidBodyBasics body = joints.get(bodyIndex).getSuccessor();

                  Wrench wrench = MecanoRandomTools.nextWrench(random, body.getBodyFixedFrame(), body.getBodyFixedFrame(), 10.0, 10.0);
                  inverseDynamics.getExternalWrench(body).set(wrench);
                  calculator.getExternalWrench(body).set(wrench);
               }
            }
         }

         inverseDynamics.compute();
         calculator.reset();

         try
         {
            MecanoTestTools.assertDMatrixEquals("Iteration: " + i, inverseDynamics.getJointTauMatrix(), calculator.getTauMatrix(), 1.0e-12);
         }
         catch (Throwable e)
         {
            TablePrinter tablePrinter = new TablePrinter();
            int col = 0;
            int row = 1;
            for (JointBasics joint : input.getJointsToConsider())
            {
               for (int j = 0; j < joint.getDegreesOfFreedom(); j++)
               {
                  String dofName = joint.getName() + " [" + toShortTypeString(joint) + "]";
                  tablePrinter.setCell(row++, col, dofName, Alignment.LEFT);
               }
            }
            col++;
            DMatrixRMaj diff = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
            CommonOps_DDRM.subtract(inverseDynamics.getJointTauMatrix(), calculator.getTauMatrix(), diff);
            CommonOps_DDRM.abs(diff);
            tablePrinter.setCell(0, col, "Exp.", Alignment.LEFT);
            tablePrinter.setSubTable(1, col++, inverseDynamics.getJointTauMatrix());
            tablePrinter.setCell(0, col, "Act.", Alignment.LEFT);
            tablePrinter.setSubTable(1, col++, calculator.getTauMatrix());
            tablePrinter.setCell(0, col, "Err.", Alignment.LEFT);
            tablePrinter.setSubTable(1, col++, diff);
            System.out.println(tablePrinter);
            throw e;
         }
      }
   }

   @Test
   public void testCalculatorPrismaticJointChain()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-6;
      double dq = 1.0e-7;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         compareAgainstFiniteDifference(random, joints, externalWrenches, dq, 2.0e-5, i);
         testMultiBodyGravityGradientCalculator(random, joints, externalWrenches, dt, 1.0e-9, i);
      }
   }

   @Test
   public void testCalculatorRevoluteJointChain()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-6;
      double dq = 1.0e-7;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         compareAgainstFiniteDifference(random, joints, externalWrenches, dq, 2.0e-5, i);
         testMultiBodyGravityGradientCalculator(random, joints, externalWrenches, dt, 1.0e-9, i);
      }
   }

   @Test
   public void testCalculatorOneDoFJointChain()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-6;
      double dq = 1.0e-7;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         compareAgainstFiniteDifference(random, joints, externalWrenches, dq, 2.0e-5, i);
         testMultiBodyGravityGradientCalculator(random, joints, externalWrenches, dt, 1.0e-9, i);
      }
   }

   @Test
   public void testCalculatorJointChain()
   {
      Random random = new Random(2342350);

      double dt = 1.0e-6;
      double dq = 1.0e-7;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         compareAgainstFiniteDifference(random, joints, externalWrenches, dq, 2.0e-5, i);
         testMultiBodyGravityGradientCalculator(random, joints, externalWrenches, dt, 1.0e-9, i);
      }
   }

   @Test
   public void testCalculatorRevoluteJointTree()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-6;
      double dq = 1.0e-7;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         compareAgainstFiniteDifference(random, joints, externalWrenches, dq, 2.0e-5, i);
         testMultiBodyGravityGradientCalculator(random, joints, externalWrenches, dt, 1.0e-9, i);
      }
   }

   @Test
   public void testCalculatorOneDoFJointTree()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-6;
      double dq = 1.0e-7;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         compareAgainstFiniteDifference(random, joints, externalWrenches, dq, 2.0e-5, i);
         testMultiBodyGravityGradientCalculator(random, joints, externalWrenches, dt, 1.0e-9, i);
      }
   }

   @Test
   public void testCalculatorJointTree()
   {
      Random random = new Random(2342350);

      double dt = 1.0e-6;
      double dq = 1.0e-7;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         compareAgainstFiniteDifference(random, joints, externalWrenches, dq, 2.0e-5, i);
         testMultiBodyGravityGradientCalculator(random, joints, externalWrenches, dt, 1.0e-9, i);
      }
   }

   @Test
   public void testModifiedRigidBodyParameters()
   {
      Random random = new Random(70689L);

      double dt = 1.0e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);

         MultiBodyGravityGradientCalculator calculator = new MultiBodyGravityGradientCalculator(MultiBodySystemBasics.toMultiBodySystemBasics(joints));
         calculator.setGravitionalAcceleration(-GRAVITY);
         calculator.reset();

         testCalculatorAgainstFiniteDifference(random, joints, externalWrenches, dt, input->calculator.getTauGradientMatrix(), 5.0e-7, i);

         RigidBodyBasics body = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         body.getInertia().set(MecanoRandomTools.nextSpatialInertia(random, body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));

         calculator.reset();

         testCalculatorAgainstFiniteDifference(random, joints, externalWrenches, dt, input->calculator.getTauGradientMatrix(), 5.0e-7, i);
      }
   }

   private static Map<RigidBodyBasics, Wrench> nextExternalWrenches(Random random, List<? extends JointBasics> joints)
   {
      Map<RigidBodyBasics, Wrench> wrenches = new HashMap<>();

      if (ADD_EXT_WRENCHES)
      {
         int numberOfWrenches;
         if (joints.size() >= 4)
            numberOfWrenches = random.nextInt(joints.size() / 4) + 1;
         else
            numberOfWrenches = 1;

         while (wrenches.size() < numberOfWrenches)
         {
            int jointIndex = random.nextInt(joints.size());
            RigidBodyBasics body = joints.get(jointIndex).getSuccessor();
            Wrench wrench = MecanoRandomTools.nextWrench(random, body.getBodyFixedFrame(), body.getBodyFixedFrame(), 10.0, 10.0);
            wrench.getLinearPart().setToZero();
            //            wrench.getAngularPart().setToZero();
            wrenches.put(body, wrench);
         }
      }

      return wrenches;
   }

   public void compareAgainstFiniteDifference(Random random,
                                              List<? extends JointBasics> joints,
                                              Map<RigidBodyBasics, Wrench> externalWrenches,
                                              double dq,
                                              double epsilon,
                                              int iteration)
   {
      MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      input.getRootBody().updateFramesRecursively();

      DMatrixRMaj expectedGradient = computeGradientByFD(input, externalWrenches, dq);
      MultiBodyGravityGradientCalculator calculator = new MultiBodyGravityGradientCalculator(input);
      calculator.setGravitionalAcceleration(-GRAVITY);
      externalWrenches.forEach((body, wrench) -> calculator.getExternalWrench(body).set(wrench));
      calculator.reset();
      DMatrixRMaj actualGradient = calculator.getTauGradientMatrix();

      try
      {
         MecanoTestTools.assertDMatrixEquals("Iteration: " + iteration, expectedGradient, actualGradient, epsilon);
      }
      catch (Throwable e)
      {
         System.out.println("Expected gradient: ");
         System.out.println(printGradient(input, expectedGradient));
         System.out.println("Actual gradient: ");
         System.out.println(printGradient(input, actualGradient));
         DMatrixRMaj diff = new DMatrixRMaj(input.getNumberOfDoFs(), input.getNumberOfDoFs());
         CommonOps_DDRM.subtract(expectedGradient, actualGradient, diff);
         CommonOps_DDRM.abs(diff);
         System.out.println("Difference: ");
         System.out.println(printGradient(input, diff));
         if (ADD_EXT_WRENCHES)
         {
            System.out.println("External wrenches: ");
            System.out.println(printExternalWrenches(input, externalWrenches));
         }
         throw e;
      }
   }

   private String printGradient(MultiBodySystemBasics input, DMatrixRMaj gradient)
   {
      TablePrinter tablePrinter = new TablePrinter();
      int col = 1;
      int row = 1;
      for (JointBasics joint : input.getJointsToConsider())
      {
         for (int j = 0; j < joint.getDegreesOfFreedom(); j++)
         {
            String dofName = joint.getName() + " [" + toShortTypeString(joint) + "]";
            tablePrinter.setCell(0, col++, dofName, Alignment.LEFT);
            tablePrinter.setCell(row++, 0, dofName, Alignment.LEFT);
         }
      }
      tablePrinter.setSubTable(1, 1, gradient);
      return tablePrinter.toString();
   }

   private String printExternalWrenches(MultiBodySystemBasics input, Map<RigidBodyBasics, Wrench> externalWrenches)
   {
      TablePrinter tablePrinter = new TablePrinter();
      int col = 1;
      DMatrixRMaj vector = new DMatrixRMaj(6, 1);

      for (JointBasics joint : input.getJointsToConsider())
      {
         String dofName = joint.getName() + " [" + toShortTypeString(joint) + "]";
         tablePrinter.setCell(0, col, dofName, Alignment.LEFT);

         Wrench wrench = externalWrenches.get(joint.getSuccessor());

         if (wrench != null)
            wrench.get(vector);
         else
            vector.zero();

         tablePrinter.setSubTable(1, col++, vector);
      }

      int row = 1;
      tablePrinter.setCell(row++, 0, "Tx", Alignment.LEFT);
      tablePrinter.setCell(row++, 0, "Ty", Alignment.LEFT);
      tablePrinter.setCell(row++, 0, "Tz", Alignment.LEFT);
      tablePrinter.setCell(row++, 0, "Fx", Alignment.LEFT);
      tablePrinter.setCell(row++, 0, "Fy", Alignment.LEFT);
      tablePrinter.setCell(row++, 0, "Fz", Alignment.LEFT);

      return tablePrinter.toString();
   }

   private static String toShortTypeString(JointReadOnly joint)
   {
      if (joint instanceof RevoluteJointReadOnly)
         return "R";
      if (joint instanceof PrismaticJointReadOnly)
         return "P";
      if (joint instanceof CrossFourBarJointReadOnly)
         return "C";
      if (joint instanceof RevoluteTwinsJointReadOnly)
         return "R2";
      if (joint instanceof SphericalJointReadOnly)
         return "S";
      if (joint instanceof SixDoFJointReadOnly)
         return "6";
      if (joint instanceof FixedJointReadOnly)
         return "0";
      if (joint instanceof PlanarJointReadOnly)
         return "3";
      return null;
   }

   @Test
   public void testFiniteDifferenceWithRevoluteJointChain()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-5;
      double dq = 1.0e-5;
      double epsilon = 1.0e-6;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         testFiniteDifferenceCalculator(random, joints, externalWrenches, dt, dq, epsilon, i);
      }
   }

   @Test
   public void testFiniteDifferenceWithOneDoFJointChain()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-5;
      double dq = 1.0e-5;
      double epsilon = 1.0e-6;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         testFiniteDifferenceCalculator(random, joints, externalWrenches, dt, dq, epsilon, i);
      }
   }

   @Test
   public void testFiniteDifferenceWithJointChain()
   {
      Random random = new Random(2342355);

      double dt = 1.0e-5;
      double dq = 1.0e-5;
      double epsilon = 1.0e-6;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         testFiniteDifferenceCalculator(random, joints, externalWrenches, dt, dq, epsilon, i);
      }
   }

   @Test
   public void testFiniteDifferenceWithRevoluteJointTree()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-5;
      double dq = 1.0e-5;
      double epsilon = 1.0e-6;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 20;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextRevoluteJointTree(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         testFiniteDifferenceCalculator(random, joints, externalWrenches, dt, dq, epsilon, i);
      }
   }

   @Test
   public void testFiniteDifferenceWithOneDoFJointTree()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-5;
      double dq = 1.0e-5;
      double epsilon = 1.0e-6;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 20;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextOneDoFJointTree(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         testFiniteDifferenceCalculator(random, joints, externalWrenches, dt, dq, epsilon, i);
      }
   }

   @Test
   public void testFiniteDifferenceWithJointTree()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-5;
      double dq = 1.0e-5;
      double epsilon = 1.0e-6;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 20;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, numberOfJoints);
         Map<RigidBodyBasics, Wrench> externalWrenches = nextExternalWrenches(random, joints);
         testFiniteDifferenceCalculator(random, joints, externalWrenches, dt, dq, epsilon, i);
      }
   }

   public void testMultiBodyGravityGradientCalculator(Random random,
                                                      List<? extends JointBasics> joints,
                                                      Map<RigidBodyBasics, Wrench> externalWrenches,
                                                      double dt,
                                                      double epsilon,
                                                      int iteration)
   {
      testCalculatorAgainstFiniteDifference(random, joints, externalWrenches, dt, input ->
      {
         MultiBodyGravityGradientCalculator calculator = new MultiBodyGravityGradientCalculator(input);
         calculator.setGravitionalAcceleration(-GRAVITY);
         externalWrenches.forEach((body, wrench) -> calculator.getExternalWrench(body).set(wrench));
         calculator.reset();
         return calculator.getTauGradientMatrix();
      }, epsilon, iteration);
   }

   public void testFiniteDifferenceCalculator(Random random,
                                              List<? extends JointBasics> joints,
                                              Map<RigidBodyBasics, Wrench> externalWrenches,
                                              double dt,
                                              double dq,
                                              double epsilon,
                                              int iteration)
   {
      testCalculatorAgainstFiniteDifference(random,
                                            joints,
                                            externalWrenches,
                                            dt,
                                            input -> computeGradientByFD(input, externalWrenches, dq),
                                            epsilon,
                                            iteration);
   }

   public void testCalculatorAgainstFiniteDifference(Random random,
                                                     List<? extends JointBasics> joints,
                                                     Map<RigidBodyBasics, Wrench> externalWrenches,
                                                     double dt,
                                                     Function<MultiBodySystemBasics, DMatrixRMaj> calculatorToTest,
                                                     double epsilon,
                                                     int iteration)
   {
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);
      MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(joints);
      DMatrixRMaj qDot = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
      DMatrixRMaj actualDeltaGravity = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
      DMatrixRMaj actualGravity = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
      DMatrixRMaj expectedDeltaGravity = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
      DMatrixRMaj errorDeltaGravity = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
      DMatrixRMaj prevGravity = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
      DMatrixRMaj expectedGravity = new DMatrixRMaj(input.getNumberOfDoFs(), 1);

      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      input.getRootBody().updateFramesRecursively();

      DMatrixRMaj gradient = calculatorToTest.apply(input);

      MultiBodySystemTools.extractJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, qDot);
      CommonOps_DDRM.mult(gradient, qDot, actualDeltaGravity);
      CommonOps_DDRM.scale(dt, actualDeltaGravity);

      // Remember the wrenches in world, so we can re-apply them after integration.
      Map<RigidBodyBasics, Wrench> externalWrenchesWorld = new HashMap<>();
      externalWrenches.forEach((body, wrench) ->
                               {
                                  Wrench wrenchInWorld = new Wrench(wrench);
                                  wrenchInWorld.changeFrame(worldFrame);
                                  externalWrenchesWorld.put(body, wrenchInWorld);
                               });

      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(input);
      calculator.setConsiderCoriolisAndCentrifugalForces(false);
      calculator.setConsiderJointAccelerations(false);
      calculator.setGravitationalAcceleration(-GRAVITY);
      externalWrenches.forEach((body, wrench) -> calculator.getExternalWrench(body).set(wrench));
      calculator.compute();
      prevGravity.set(calculator.getJointTauMatrix());
      CommonOps_DDRM.add(prevGravity, actualDeltaGravity, actualGravity);

      input.getRootBody().updateFramesRecursively();
      integrator.integrateFromVelocitySubtree(input.getRootBody());
      input.getRootBody().updateFramesRecursively();
      calculator.setExternalWrenchesToZero();
      externalWrenchesWorld.forEach((body, wrench) -> calculator.getExternalWrench(body).setMatchingFrame(wrench));
      calculator.compute();
      expectedGravity.set(calculator.getJointTauMatrix());

      CommonOps_DDRM.subtract(expectedGravity, prevGravity, expectedDeltaGravity);
      CommonOps_DDRM.subtract(expectedDeltaGravity, actualDeltaGravity, errorDeltaGravity);

      try
      {
         MecanoTestTools.assertDMatrixEquals("Iteration: " + iteration, expectedGravity, actualGravity, epsilon);
         MecanoTestTools.assertDMatrixEquals("Iteration: " + iteration, expectedDeltaGravity, actualDeltaGravity, epsilon);
      }
      catch (Throwable e)
      {
         TablePrinter tablePrinter = new TablePrinter();
         int col = 0;
         tablePrinter.setCell(0, col, "Joint", Alignment.LEFT);
         int row = 1;
         for (JointBasics joint : input.getJointsToConsider())
         {
            for (int j = 0; j < joint.getDegreesOfFreedom(); j++)
            {
               String dofName = joint.getName() + " [" + toShortTypeString(joint) + "]";
               tablePrinter.setCell(row++, col, dofName, Alignment.LEFT);
            }
         }
         col++;
         tablePrinter.setCell(0, col, "Act. Delta", Alignment.LEFT);
         tablePrinter.setSubTable(1, col++, actualDeltaGravity);
         tablePrinter.setCell(0, col, "Exp. Delta", Alignment.LEFT);
         tablePrinter.setSubTable(1, col++, expectedDeltaGravity);
         tablePrinter.setCell(0, col, "Err. Delta", Alignment.LEFT);
         tablePrinter.setSubTable(1, col++, errorDeltaGravity);
         tablePrinter.setCell(0, col, "Act. Gradient", Alignment.LEFT);
         CommonOps_DDRM.scale(dt, gradient);
         tablePrinter.setSubTable(1, col++, gradient);
         System.out.println(tablePrinter);
         throw e;
      }
   }

   private static DMatrixRMaj computeGradientByFD(MultiBodySystemBasics input, Map<RigidBodyBasics, Wrench> externalWrenches, double dq)
   {
      DMatrixRMaj originalConfiguration = new DMatrixRMaj(input.getJointsToConsider().stream().mapToInt(JointReadOnly::getConfigurationMatrixSize).sum(), 1);
      MultiBodySystemTools.extractJointsState(input.getJointsToConsider(), JointStateType.CONFIGURATION, originalConfiguration);
      DMatrixRMaj originalVelocity = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
      MultiBodySystemTools.extractJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, originalVelocity);

      // Remember the wrenches in world, so we can re-apply them after integration.
      Map<RigidBodyBasics, Wrench> externalWrenchesWorld = new HashMap<>();
      externalWrenches.forEach((body, wrench) ->
                               {
                                  Wrench wrenchInWorld = new Wrench(wrench);
                                  wrenchInWorld.changeFrame(worldFrame);
                                  externalWrenchesWorld.put(body, wrenchInWorld);
                               });

      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(input);
      calculator.setConsiderCoriolisAndCentrifugalForces(false);
      calculator.setConsiderJointAccelerations(false);
      calculator.setGravitationalAcceleration(-GRAVITY);
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dq);

      int nDoFs = input.getNumberOfDoFs();
      DMatrixRMaj qDot = new DMatrixRMaj(nDoFs, 1);
      DMatrixRMaj gradient = new DMatrixRMaj(nDoFs, nDoFs);
      DMatrixRMaj gradientCol = new DMatrixRMaj(nDoFs, 1);

      MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, qDot);
      input.getRootBody().updateFramesRecursively();

      calculator.setExternalWrenchesToZero();
      externalWrenchesWorld.forEach((body, wrench) -> calculator.getExternalWrench(body).setMatchingFrame(wrench));
      calculator.compute();
      DMatrixRMaj referenceGravity = new DMatrixRMaj(calculator.getJointTauMatrix());

      for (int dofIndex = 0; dofIndex < nDoFs; dofIndex++)
      {
         qDot.zero();
         qDot.set(dofIndex, 0, 1);
         MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, qDot);
         input.getRootBody().updateFramesRecursively();
         integrator.integrateFromVelocitySubtree(input.getRootBody());
         input.getRootBody().updateFramesRecursively();

         calculator.setExternalWrenchesToZero();
         externalWrenchesWorld.forEach((body, wrench) -> calculator.getExternalWrench(body).setMatchingFrame(wrench));
         calculator.compute();

         CommonOps_DDRM.subtract(calculator.getJointTauMatrix(), referenceGravity, gradientCol);
         CommonOps_DDRM.scale(1.0 / dq, gradientCol);
         CommonOps_DDRM.insert(gradientCol, gradient, 0, dofIndex);

         MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.CONFIGURATION, originalConfiguration);
      }

      MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, originalVelocity);
      input.getRootBody().updateFramesRecursively();

      return gradient;
   }
}