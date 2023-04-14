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

        InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
        inverseDynamicsCalculator.compute();
        DMatrixRMaj jointTau = inverseDynamicsCalculator.getJointTauMatrix();

        DMatrixRMaj parameterVector = new DMatrixRMaj(10 * 2, 1);  // TODO hardcoded
        for (int i = 0; i < 2; i++)  // TODO hardcoded
        {
            CommonOps_DDRM.insert(JointTorqueRegressorCalculator.toParameterVector((SpatialInertia) system.findRigidBody("Body" + String.valueOf(i)).getInertia()),
                    parameterVector, i * 10, 0);
        }

        JointTorqueRegressorCalculator regressorCalculator = new JointTorqueRegressorCalculator(system);
        DMatrixRMaj regressorMatrix = new DMatrixRMaj(2, 20); // TODO hardcoded
        regressorCalculator.compute();
        regressorMatrix.set(regressorCalculator.getJointTorqueRegressorMatrix());

        DMatrixRMaj testResult = new DMatrixRMaj(2, 1);  // TODO hardcoded
        CommonOps_DDRM.mult(regressorMatrix, parameterVector, testResult);
        assertArrayEquals(testResult.getData(), jointTau.getData());
    }
}
