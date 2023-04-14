package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class JointTorqueRegressorCalculatorTest
{
    private static final int WARMUP_ITERATIONS = 5000;
    private static final int STATE_ITERATIONS = 100;
    private static final int SYSTEM_ITERATIONS = 5;

    private static final double EPSILON = 1.0e-12;

    @Test
    public void testRegressorAndParametersMatchInverseDynamicsOneDoFJointChain() {
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
                inverseDynamicsCalculator.compute();
                DMatrixRMaj expectedjointTau = inverseDynamicsCalculator.getJointTauMatrix();

                // We want the product of the regressor matrix and the parameter vector to equal the expected joint torques
                JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
                DMatrixRMaj parameterVector = regressorCalculator.getParameterVector();
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
    public void testSpatialInertiaParameterBasis()
    {
        SpatialInertia spatialInertia = new SpatialInertia();
        DMatrixRMaj expectedBasis = new DMatrixRMaj(6, 6);
        DMatrixRMaj actualBasis = new DMatrixRMaj(6, 6);
        double[][] expectedBasisToPack = new double[6][6];

        for (JointTorqueRegressorCalculator.SpatialInertiaParameterBasisOptions basis : JointTorqueRegressorCalculator.SpatialInertiaParameterBasisOptions.values())
            switch (basis)
            {
                case M:
                    expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 1., 0., 0.},
                            {0., 0., 0., 0., 1., 0.},
                            {0., 0., 0., 0., 0., 1.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case MCOM_X:
                    expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., -1.},
                            {0., 0., 0., 0., 1., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 1., 0., 0., 0.},
                            {0., -1., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case MCOM_Y:
                    expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 1.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., -1., 0., 0.},
                            {0., 0., -1., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {1., 0., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case MCOM_Z:
                    expectedBasisToPack = new double[][] {{0., 0., 0., 0., -1., 0.},
                            {0., 0., 0., 1., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 1., 0., 0., 0., 0.},
                            {-1., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case I_XX:
                    expectedBasisToPack = new double[][] {{1., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case I_YY:
                    expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                            {0., 1., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case I_ZZ:
                    expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 1., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case I_XY:
                    expectedBasisToPack = new double[][] {{0., 1., 0., 0., 0., 0.},
                            {1., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case I_XZ:
                    expectedBasisToPack = new double[][] {{0., 0., 1., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {1., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
                case I_YZ:
                    expectedBasisToPack = new double[][] {{0., 0., 0., 0., 0., 0.},
                            {0., 0., 1., 0., 0., 0.},
                            {0., 1., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.},
                            {0., 0., 0., 0., 0., 0.}};
                    expectedBasis.set(expectedBasisToPack);
                    basis.getBasis(null, null).get(actualBasis);
                    assertArrayEquals(expectedBasis.getData(), actualBasis.getData(), EPSILON);
                    break;
            }
    }
}
