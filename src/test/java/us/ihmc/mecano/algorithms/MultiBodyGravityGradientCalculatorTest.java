package us.ihmc.mecano.algorithms;

import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.mecano.algorithms.TablePrinter.Alignment;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FixedJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PrismaticJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class MultiBodyGravityGradientCalculatorTest
{
   private static final int ITERATIONS = 1000;
   private static final double GRAVITY = 10.0;

   @Test
   public void testCalculatorRevoluteJointChain()
   {
      Random random = new Random(2342356);

      double dq = 1.0e-7;
      double epsilon = 1.0e-5;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 5;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
         compareAgainstFiniteDifference(random, joints, dq, epsilon, i);
      }
   }
   
   @Test
   public void testCalculatorOneDoFJointChain()
   {
      Random random = new Random(2342356);
      
      double dq = 1.0e-7;
      double epsilon = 1.0e-5;
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 5;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
         compareAgainstFiniteDifference(random, joints, dq, epsilon, i);
      }
   }

   @Test
   public void testCalculatorJointChain()
   {
      Random random = new Random(2342350);

      double dq = 1.0e-7;
      double epsilon = 1.0e-5;

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 1;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointChain(random, numberOfJoints);
         compareAgainstFiniteDifference(random, joints, dq, epsilon, i);
      }
   }
   
   public void compareAgainstFiniteDifference(Random random, List<? extends JointBasics> joints, double dq, double epsilon, int iteration)
   {
      MultiBodySystemBasics input = MultiBodySystemBasics.toMultiBodySystemBasics(joints);
//      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, joints);
      MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
      input.getRootBody().updateFramesRecursively();

      DMatrixRMaj expectedGradient = computeGradientByFD(input, dq);
      MultiBodyGravityGradientCalculator calculator = new MultiBodyGravityGradientCalculator(input);
      calculator.setGravitionalAcceleration(-GRAVITY);
      calculator.compute();
      DMatrixRMaj actualGradient = calculator.getGradientMatrix();

      try
      {
         MecanoTestTools.assertDMatrixEquals("Iteration: " + iteration, expectedGradient, actualGradient, epsilon);
      }
      catch (Throwable e)
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
         DMatrixRMaj diff = new DMatrixRMaj(input.getNumberOfDoFs(), input.getNumberOfDoFs());
         CommonOps_DDRM.subtract(expectedGradient, actualGradient, diff);
         CommonOps_DDRM.abs(diff);
         tablePrinter.setSubTable(1, 1, diff);
         System.out.println(tablePrinter);
         throw e;
      }
   }

   private static String toShortTypeString(JointReadOnly joint)
   {
      if (joint instanceof RevoluteJointReadOnly)
         return "R";
      if (joint instanceof PrismaticJointReadOnly)
         return "P";
      if (joint instanceof CrossFourBarJointReadOnly)
         return "C";
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
   public void testFiniteDifference()
   {
      Random random = new Random(2342356);

      double dt = 1.0e-5;
      double dq = 1.0e-5;
      double epsilon = 1.0e-6;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = 10;
         List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
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

         DMatrixRMaj gradient = computeGradientByFD(input, dq);

         MultiBodySystemTools.extractJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, qDot);
         CommonOps_DDRM.mult(gradient, qDot, actualDeltaGravity);
         CommonOps_DDRM.scale(dt, actualDeltaGravity);

         InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(input);
         calculator.setConsiderCoriolisAndCentrifugalForces(false);
         calculator.setConsiderJointAccelerations(false);
         calculator.setGravitionalAcceleration(-GRAVITY);
         calculator.compute();
         prevGravity.set(calculator.getJointTauMatrix());
         CommonOps_DDRM.add(prevGravity, actualDeltaGravity, actualGravity);

         input.getRootBody().updateFramesRecursively();
         integrator.integrateFromVelocitySubtree(input.getRootBody());
         input.getRootBody().updateFramesRecursively();
         calculator.compute();
         expectedGravity.set(calculator.getJointTauMatrix());

         CommonOps_DDRM.subtract(expectedGravity, prevGravity, expectedDeltaGravity);
         CommonOps_DDRM.subtract(expectedDeltaGravity, actualDeltaGravity, errorDeltaGravity);

         try
         {
            MecanoTestTools.assertDMatrixEquals(expectedGravity, actualGravity, epsilon);
            MecanoTestTools.assertDMatrixEquals(expectedDeltaGravity, actualDeltaGravity, epsilon);
         }
         catch (Throwable e)
         {
            TablePrinter tablePrinter = new TablePrinter();
            int col = 0;
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
   }

   private static DMatrixRMaj computeGradientByFD(MultiBodySystemBasics input, double dq)
   {
      DMatrixRMaj originalConfiguration = new DMatrixRMaj(input.getJointsToConsider().stream().mapToInt(JointReadOnly::getConfigurationMatrixSize).sum(), 1);
      MultiBodySystemTools.extractJointsState(input.getJointsToConsider(), JointStateType.CONFIGURATION, originalConfiguration);
      DMatrixRMaj originalVelocity = new DMatrixRMaj(input.getNumberOfDoFs(), 1);
      MultiBodySystemTools.extractJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, originalVelocity);

      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(input);
      calculator.setConsiderCoriolisAndCentrifugalForces(false);
      calculator.setConsiderJointAccelerations(false);
      calculator.setGravitionalAcceleration(-GRAVITY);
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dq);

      int nDoFs = input.getNumberOfDoFs();
      DMatrixRMaj qDot = new DMatrixRMaj(nDoFs, 1);
      DMatrixRMaj gradient = new DMatrixRMaj(nDoFs, nDoFs);
      DMatrixRMaj gradientCol = new DMatrixRMaj(nDoFs, 1);

      MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, qDot);
      input.getRootBody().updateFramesRecursively();

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

         calculator.compute();

         CommonOps_DDRM.subtract(calculator.getJointTauMatrix(), referenceGravity, gradientCol);
         CommonOps_DDRM.scale(1.0 / dq, gradientCol);
         CommonOps_DDRM.insert(gradientCol, gradient, 0, dofIndex);

         MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.CONFIGURATION, originalConfiguration);
      }

      MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, originalVelocity);
      input.getRootBody().updateFramesRecursively();

      CommonOps_DDRM.transpose(gradient);
      return gradient;
   }
}