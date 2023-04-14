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

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

public class JointTorqueRegressorCalculatorTest
{
    private static final int WARMUP_ITERATIONS = 5000;
    private static final int ITERATIONS = 50000;

    @Test
    public void test()  // TODO name me
    {
        Random random = new Random(25);

        List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, 2);
        MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);

        for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());

        InverseDynamicsCalculator referenceCalculator = new InverseDynamicsCalculator(system);
        referenceCalculator.compute();
        DMatrixRMaj jointTau = referenceCalculator.getJointTauMatrix();

        DMatrixRMaj parameterVector = new DMatrixRMaj(10 * 2, 1);  // TODO hardcoded
        for (int i = 0; i < 2; i++)  // TODO hardcoded
        {
            CommonOps_DDRM.insert(JointTorqueRegressorCalculator.toParameterVector((SpatialInertia) system.findRigidBody("Body" + String.valueOf(i)).getInertia()),
                    parameterVector, i * 10, 0);
        }

        // Generate regressor matrix by calling RNEA several times, one for each body / parameter combination
        DMatrixRMaj regressorMatrix = new DMatrixRMaj(2, 20); // TODO hardcoded
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < JointTorqueRegressorCalculator.SpatialInertiaParameterBasis.values().length; j++){
                DMatrixRMaj regressorColumn = JointTorqueRegressorCalculator.calculateRegressorColumn(system, i, JointTorqueRegressorCalculator.SpatialInertiaParameterBasis.values()[j]);
                CommonOps_DDRM.insert(regressorColumn, regressorMatrix, 0, (i * 10) + j);
            }
        }

        DMatrixRMaj testResult = new DMatrixRMaj(2, 1);  // TODO hardcoded
        CommonOps_DDRM.mult(regressorMatrix, parameterVector, testResult);
        assertArrayEquals(testResult.getData(), jointTau.getData());
    }

}
