package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialInertiaParameterBasis;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.Objects;

public class JointTorqueRegressorCalculator
{
    /** Defines the multi-body system to use with this calculator. */
    private final MultiBodySystemBasics input;

    private final DMatrixRMaj jointTorqueRegressorMatrix;

    private final DMatrixRMaj parameterVector;

    private final int numberOfBodies;

    private final InverseDynamicsCalculator inverseDynamicsCalculator;

    private final SpatialInertiaParameterBasis spatialInertiaParameterBasis;

    enum SpatialInertiaParameterBasisOptions
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
    }

    public SpatialInertiaParameterBasis getBasis(SpatialInertiaParameterBasisOptions basis, RigidBodyBasics rigidBody)
    {
        spatialInertiaParameterBasis.setBodyFrame(rigidBody.getInertia().getBodyFrame());
        spatialInertiaParameterBasis.setReferenceFrame(rigidBody.getInertia().getReferenceFrame());
        spatialInertiaParameterBasis.setToZero();
        switch (basis)
        {
            case M -> spatialInertiaParameterBasis.setMass(1.0);
            case MCOM_X -> spatialInertiaParameterBasis.setCenterOfMassOffset(1.0, 0.0, 0.0);
            case MCOM_Y -> spatialInertiaParameterBasis.setCenterOfMassOffset(0.0, 1.0, 0.0);
            case MCOM_Z -> spatialInertiaParameterBasis.setCenterOfMassOffset(0.0, 0.0, 1.0);
            case I_XX -> spatialInertiaParameterBasis.setMomentOfInertia(1.0, 0.0, 0.0);
            case I_YY -> spatialInertiaParameterBasis.setMomentOfInertia(0.0, 1.0, 0.0);
            case I_ZZ -> spatialInertiaParameterBasis.setMomentOfInertia(0.0, 0.0, 1.0);
            case I_XY -> spatialInertiaParameterBasis.setMomentOfInertia(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
            case I_XZ -> spatialInertiaParameterBasis.setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
            case I_YZ -> spatialInertiaParameterBasis.setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        }
        return spatialInertiaParameterBasis;
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
        spatialInertiaParameterBasis = new SpatialInertiaParameterBasis();

        int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
        RigidBodyBasics[] bodies = input.getRootBody().subtreeArray();  // TODO this generates garbage, but acceptable in the constructor?
        numberOfBodies = bodies.length - 1;  // TODO Also -1 to dodge root body
        int nParams = 10 * numberOfBodies;

        jointTorqueRegressorMatrix = new DMatrixRMaj(nDoFs, nParams);
        parameterVector = new DMatrixRMaj(nParams, 1);
        for (int i = 1; i < bodies.length; i++)
        {
            CommonOps_DDRM.insert(toParameterVector(bodies[i].getInertia()), parameterVector, (i-1) * 10, 0); // TODO (i-1) because of the loop offset dodging the root body
        }
    }

    public void compute()
    {
        // Generate regressor matrix by calling RNEA several times, one for each body / parameter combination
        for (int i = 0; i < numberOfBodies; i++)
        {
            for (int j = 0; j < SpatialInertiaParameterBasisOptions.values().length; j++){  // TODO this assumes we're using every parameter
                CommonOps_DDRM.insert(calculateRegressorColumn(input, i, SpatialInertiaParameterBasisOptions.values()[j]),
                                      jointTorqueRegressorMatrix, 0, (i * 10) + j);
            }
        }
    }

    private DMatrixRMaj calculateRegressorColumn(MultiBodySystemBasics system, int bodyIndex, SpatialInertiaParameterBasisOptions basis)
    {
        String bodyName = "Body";
        bodyName = bodyName + String.valueOf(bodyIndex);
        for (RigidBodyBasics body : system.getRootBody().subtreeArray())
        {
            if (Objects.equals(body.getName(), "RootBody"))  // Just pass if it's the root body, TODO why is the root body null?
                continue;
            if (Objects.equals(body.getName(), bodyName)) {
                body.getInertia().set(getBasis(basis, body));
                inverseDynamicsCalculator.getBodyInertia(body).set(getBasis(basis, body));
            }
            else {
                body.getInertia().setToZero();
                inverseDynamicsCalculator.getBodyInertia(body).setToZero();
            }
        }

        inverseDynamicsCalculator.compute();
        return inverseDynamicsCalculator.getJointTauMatrix();
    }

    public void setGravitationalAcceleration(double gravity)
    {
        setGravitationalAcceleration(0.0, 0.0, gravity);
    }

    public void setGravitationalAcceleration(double gravityX, double gravityY, double gravityZ)
    {
        inverseDynamicsCalculator.setGravitionalAcceleration(gravityX, gravityY, gravityZ);
    }

    // TODO there is a huge caveat to using this method that you should note -- it should only be used just after
    //  construction, so that the spatial inertia values are still the correct (physical) values. If you call it later
    //  after messing around with the SpatialInertiaParameterBasisOptions, the parameters will probably be set to their
    //  basis values
    private static DMatrixRMaj toParameterVector(SpatialInertiaBasics spatialInertia)
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

    public DMatrixRMaj getParameterVector()
    {
        return parameterVector;
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
    }
}
