package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.List;
import java.util.Objects;
import java.util.Random;

public class JointTorqueRegressorCalculator {

    private final DMatrixRMaj jointTorqueRegressorMatrix;

    public DMatrixRMaj getJointTorqueRegressorMatrix()
    {
        return  jointTorqueRegressorMatrix;
    }

    enum SpatialInertiaParameterBasis
    {
        M,
        MCOM_X,
        MCOM_Y,
        MCOM_Z,
        I_XX,
        I_YY,
        I_ZZ,
        I_XY,
        I_XZ,
        I_YZ;

        public SpatialInertia getBasis(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
        {
            SpatialInertia spatialInertia = new SpatialInertia(bodyFrame, expressedInFrame);
            spatialInertia.setToZero();
            switch (this) {
                case M -> spatialInertia.setMass(1.0);
                case MCOM_X -> spatialInertia.setCenterOfMassOffset(1.0, 0.0, 0.0);
                case MCOM_Y -> spatialInertia.setCenterOfMassOffset(0.0, 1.0, 0.0);
                case MCOM_Z -> spatialInertia.setCenterOfMassOffset(0.0, 0.0, 1.0);
                case I_XX -> spatialInertia.setMomentOfInertia(1.0, 0.0, 0.0);
                case I_YY -> spatialInertia.setMomentOfInertia(0.0, 1.0, 0.0);
                case I_ZZ -> spatialInertia.setMomentOfInertia(0.0, 0.0, 1.0);
                case I_XY -> spatialInertia.setMomentOfInertia(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
                case I_XZ -> spatialInertia.setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
                case I_YZ -> spatialInertia.setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            }
            return spatialInertia;
        }
    }

    public JointTorqueRegressorCalculator(RigidBodyReadOnly rootBody)
    {
        this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), true);
    }

    public JointTorqueRegressorCalculator(MultiBodySystemReadOnly input, boolean considerIgnoredSubtreesInertia)
    {
        int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
        int nBodies = 3; // TODO: find a way to programatically get this
        int nParams = 10 * nBodies;
        jointTorqueRegressorMatrix = new DMatrixRMaj(nDoFs, nParams);
    }

    public static DMatrixRMaj calculateRegressorColumn(MultiBodySystemBasics system, int bodyIndex, SpatialInertiaParameterBasis basis)
    {
        Random random = new Random(25);
        String bodyName = "Body";
        bodyName = bodyName + String.valueOf(bodyIndex);
        for (RigidBodyBasics body : system.getRootBody().subtreeArray())
        {
            if (Objects.equals(body.getName(), "RootBody"))  // Just pass if it's the root body, TODO why is the root body null?
                continue;
            if (Objects.equals(body.getName(), bodyName))
            {
//                System.out.println("------");
//                System.out.println("CHANGING BODY!!!!");
//                System.out.println("Before");
//                System.out.println(body.getInertia());
//                System.out.println("After");
                // Param 1
                body.getInertia().set(basis.getBasis(body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));
//                System.out.println(body.getInertia());
            }
            else
            {
//                System.out.println("------");
//                System.out.println("Before");
//                System.out.println(body.getInertia());
//                System.out.println("After");
                body.getInertia().setToZero();
//                System.out.println(body.getInertia());
            }
        }

        InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
//        inverseDynamicsCalculator.setGravitionalAcceleration(-9.81); // TODO BIG TODO THIS, RESULTS DON'T MATCH IF WE SET A NON-ZERO GRAVITY

        inverseDynamicsCalculator.compute();
        DMatrixRMaj output = inverseDynamicsCalculator.getJointTauMatrix();
        return output;
    }

    public static DMatrixRMaj toParameterVector(SpatialInertia spatialInertia)
    {
        DMatrixRMaj parameters = new DMatrixRMaj(10, 1);
        parameters.set(0, 0, spatialInertia.getMass());
        parameters.set(1, 0, spatialInertia.getCenterOfMassOffset().getX());
        parameters.set(2, 0, spatialInertia.getCenterOfMassOffset().getY());
        parameters.set(3, 0, spatialInertia.getCenterOfMassOffset().getZ());
        parameters.set(4, 0, spatialInertia.getMomentOfInertia().getM00());  // Ixx
        parameters.set(5, 0, spatialInertia.getMomentOfInertia().getM11());  // Iyy
        parameters.set(6, 0, spatialInertia.getMomentOfInertia().getM22());  // Izz
        parameters.set(7, 0, spatialInertia.getMomentOfInertia().getM01());  // Ixy
        parameters.set(8, 0, spatialInertia.getMomentOfInertia().getM02());  // Ixz
        parameters.set(9, 0, spatialInertia.getMomentOfInertia().getM12());  // Iyz
        return parameters;
    }

    public static void main(String[] args) {
        Random random = new Random(25);

        List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, 2);
        MultiBodySystemBasics system = MultiBodySystemBasics.toMultiBodySystemBasics(joints);
        System.out.println("system" + system);

        for (JointStateType stateToRandomize : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateToRandomize, system.getAllJoints());
        InverseDynamicsCalculator referenceCalculator = new InverseDynamicsCalculator(system);
        referenceCalculator.compute();
        DMatrixRMaj torqueResultWeWant = referenceCalculator.getJointTauMatrix();
        System.out.println("Result we want:");
        System.out.println(torqueResultWeWant);

        DMatrixRMaj parameterVector = new DMatrixRMaj(10 * 2, 1);  // TODO hardcoded
        for (int i = 0; i < 2; i++)  // TODO hardcoded
        {
            CommonOps_DDRM.insert(JointTorqueRegressorCalculator.toParameterVector((SpatialInertia) system.findRigidBody("Body" + String.valueOf(i)).getInertia()),
                    parameterVector, i * 10, 0);
        }
        System.out.println("Parameters:");
        System.out.println(parameterVector);

        // TODO: figure out why the root body has null inertial parameters, is it just a dummy world link
//        System.out.println(system.getRootBody());
//        System.out.println(system.getRootBody().getInertia().getMass());
//        System.out.println(system.getRootBody().getInertia().getCenterOfMassOffset());
//        System.out.println(system.getRootBody().getInertia().getMomentOfInertia());

//        JointTorqueRegressorCalculator calc = new JointTorqueRegressorCalculator(joints.get(0).getPredecessor());
//        System.out.println(calc);

        // TODO put this in a test, hand write what you expect the parameter bases to be
//        for (SpatialInertiaParameterBasis basis : SpatialInertiaParameterBasis.values())
//        {
//            System.out.println(basis);
//            System.out.println(basis.getBasis(null, null));
//        }

//        system.getAllJoints().forEach(joint -> System.out.println(joint.getSuccessor()));

        DMatrixRMaj regressorMatrix = new DMatrixRMaj(2, 20); // TODO hardcoded

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < SpatialInertiaParameterBasis.values().length; j++){
                DMatrixRMaj regressorColumn = JointTorqueRegressorCalculator.calculateRegressorColumn(system, i, SpatialInertiaParameterBasis.values()[j]);
                System.out.println(regressorColumn);
                CommonOps_DDRM.insert(regressorColumn, regressorMatrix, 0, (i * 10) + j);
            }
        }

        System.out.println("Regressor matrix:");
        System.out.println(regressorMatrix);

        DMatrixRMaj testResult = new DMatrixRMaj(2, 1);  // TODO hardcoded
        CommonOps_DDRM.mult(regressorMatrix, parameterVector, testResult);
        System.out.println("Test result:");
        System.out.println(testResult);

    }

}
