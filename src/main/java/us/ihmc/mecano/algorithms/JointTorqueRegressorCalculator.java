package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.SpatialInertiaParameterBasis;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.Objects;

public class JointTorqueRegressorCalculator
{
    private final MultiBodySystemBasics input;

    private final DMatrixRMaj jointTorqueRegressorMatrix;

    private final DMatrixRMaj parameterVector;

    private final int numberOfBodies;

    private InverseDynamicsCalculator inverseDynamicsCalculator;

    private FrameVector3D gravitationalAcceleration;

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

        public SpatialInertiaParameterBasis getBasis(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
        {
            SpatialInertiaParameterBasis spatialInertiaBasis = new SpatialInertiaParameterBasis(bodyFrame, expressedInFrame);
            spatialInertiaBasis.setToZero();
            switch (this)
            {
                case M -> spatialInertiaBasis.setMass(1.0);
                case MCOM_X -> spatialInertiaBasis.setCenterOfMassOffset(1.0, 0.0, 0.0);
                case MCOM_Y -> spatialInertiaBasis.setCenterOfMassOffset(0.0, 1.0, 0.0);
                case MCOM_Z -> spatialInertiaBasis.setCenterOfMassOffset(0.0, 0.0, 1.0);
                case I_XX -> spatialInertiaBasis.setMomentOfInertia(1.0, 0.0, 0.0);
                case I_YY -> spatialInertiaBasis.setMomentOfInertia(0.0, 1.0, 0.0);
                case I_ZZ -> spatialInertiaBasis.setMomentOfInertia(0.0, 0.0, 1.0);
                case I_XY -> spatialInertiaBasis.setMomentOfInertia(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
                case I_XZ -> spatialInertiaBasis.setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
                case I_YZ -> spatialInertiaBasis.setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            }
            return spatialInertiaBasis;
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
        gravitationalAcceleration = new FrameVector3D();

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
            if (Objects.equals(body.getName(), bodyName))
                body.getInertia().set(basis.getBasis(body.getInertia().getBodyFrame(), body.getInertia().getReferenceFrame()));
            else
                body.getInertia().setToZero();
        }

        inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
        inverseDynamicsCalculator.setGravitionalAcceleration(gravitationalAcceleration);
        inverseDynamicsCalculator.compute();
        DMatrixRMaj output = inverseDynamicsCalculator.getJointTauMatrix();
        return output;
    }

    public void setGravitationalAcceleration(double gravity)
    {
        setGravitationalAcceleration(0.0, 0.0, gravity);
    }

    public void setGravitationalAcceleration(double gravityX, double gravityY, double gravityZ)
    {
        gravitationalAcceleration.set(gravityX, gravityY, gravityZ);
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
