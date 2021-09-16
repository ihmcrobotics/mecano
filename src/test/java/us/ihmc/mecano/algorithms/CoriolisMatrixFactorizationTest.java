package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;

import java.util.Random;

/**
 * Tests that the following identity is valid as computed by {@link CompositeRigidBodyMassMatrixCalculator}:
 *
 * <pre>
 * (d/dt) H(q) = C(q, qDot) + C(q, qDot)<sup>T</sup>
 * </pre>
 * <p>
 * For information on the identity, see:
 * "Numerical Methods to Compute the Coriolis Matrix and Christoffel Symbols for Rigid-Body Systems", Echeandia et. al - Condition 2
 */
public class CoriolisMatrixFactorizationTest
{
   private static final int ITERATIONS = 500;
   private static final double EPSILON = 1.0e-4;

   private static final double DT = 1.0e-7;
   private static final int MAX_JOINTS = 20;

   @Test
   public void testCoriolisMatrixFactorization()
   {
      Random random = new Random(329023);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfJoints = random.nextInt(MAX_JOINTS);
         MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain robot = new MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain(random,
                                                                                                                                             numberOfJoints);
         RigidBody elevator = robot.getElevator();
         MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(elevator);
         robot.nextState(random, JointStateType.CONFIGURATION, JointStateType.VELOCITY, JointStateType.ACCELERATION);

         CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(multiBodySystemInput);
         massMatrixCalculator.setEnableCoriolisMatrixCalculation(true);

         DMatrixRMaj massMatrix0 = new DMatrixRMaj(0);
         DMatrixRMaj massMatrix1 = new DMatrixRMaj(0);
         DMatrixRMaj massMatrixDot = new DMatrixRMaj(0);

         massMatrixCalculator.reset();
         massMatrix0.set(massMatrixCalculator.getMassMatrix());

         MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(DT);
         integrator.integrateFromVelocitySubtree(elevator);
         robot.getElevator().updateFramesRecursively();

         massMatrixCalculator.reset();
         massMatrix1.set(massMatrixCalculator.getMassMatrix());

         CommonOps_DDRM.subtract(massMatrix1, massMatrix0, massMatrixDot);
         CommonOps_DDRM.scale(1.0 / DT, massMatrixDot);

         DMatrixRMaj coriolisMatrix = new DMatrixRMaj(0);
         DMatrixRMaj coriolisMatrixTranspose = new DMatrixRMaj(0);
         DMatrixRMaj coriolisSum = new DMatrixRMaj(0);

         coriolisMatrix.set(massMatrixCalculator.getCoriolisMatrix());
         coriolisMatrixTranspose.set(coriolisMatrix);
         CommonOps_DDRM.transpose(coriolisMatrixTranspose);
         CommonOps_DDRM.add(coriolisMatrix, coriolisMatrixTranspose, coriolisSum);

         DMatrixRMaj error = new DMatrixRMaj(0);
         CommonOps_DDRM.subtract(coriolisSum, massMatrixDot, error);

         for (int j = 0; j < error.getNumElements(); j++)
         {
            Assertions.assertTrue(Math.abs(error.get(j)) < EPSILON,
                                  "Invalid factorization found for serial chain with " + numberOfJoints + " one dof joints. Error term of " + error.get(j));
         }
      }
   }
}
