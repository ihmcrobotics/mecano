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

public class JointTorqueRegressorCalculator
{
    private final MultiBodySystemBasics input;

    private final DMatrixRMaj jointTorqueRegressorMatrix;

    private InverseDynamicsCalculator inverseDynamicsCalculator;

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

    public JointTorqueRegressorCalculator(RigidBodyBasics rootBody)
    {
        this(MultiBodySystemBasics.toMultiBodySystemBasics(rootBody), true);
    }

    public JointTorqueRegressorCalculator(MultiBodySystemBasics system)
    {
        this(system, true);
    }

    public JointTorqueRegressorCalculator(MultiBodySystemBasics input, boolean considerIgnoredSubtreesInertia)
    {
        this.input = input;
        inverseDynamicsCalculator = new InverseDynamicsCalculator(input, considerIgnoredSubtreesInertia);
        int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
        int nBodies = 2; // TODO find a way to programatically get this
        int nParams = 10 * nBodies;
        jointTorqueRegressorMatrix = new DMatrixRMaj(nDoFs, nParams);
    }

    public void compute()
    {
        // Generate regressor matrix by calling RNEA several times, one for each body / parameter combination
        for (int i = 0; i < 2; i++)  // TODO hardcoded
        {
            for (int j = 0; j < SpatialInertiaParameterBasis.values().length; j++){
                CommonOps_DDRM.insert(calculateRegressorColumn(input, i, SpatialInertiaParameterBasis.values()[j]),
                                      jointTorqueRegressorMatrix, 0, (i * 10) + j);
            }
        }
    }

    private DMatrixRMaj calculateRegressorColumn(MultiBodySystemBasics system, int bodyIndex, SpatialInertiaParameterBasis basis)
    {
        String bodyName = "Body";
        bodyName = bodyName + String.valueOf(bodyIndex);
        for (RigidBodyBasics body : system.getRootBody().subtreeArray())
        {
            if (Objects.equals(body.getName(), "RootBody"))  // Just pass if it's the root body, TODO why is the root body null?
                continue;
            if (Objects.equals(body.getName(), bodyName))
                body.getInertia().set(basis.getBasis(body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));
            else
                body.getInertia().setToZero();
        }

        inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
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

    public DMatrixRMaj getJointTorqueRegressorMatrix()
    {
        return  jointTorqueRegressorMatrix;
    }

    public static void main(String[] args)
    {
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
    }
}
