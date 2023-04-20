package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialInertiaParameterBasis;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.*;

public class JointTorqueRegressorCalculator
{
    /** Defines the multi-body system to use with this calculator. */
    private final MultiBodySystemBasics input;

    private final DMatrixRMaj jointTorqueRegressorMatrix;

    private final DMatrixRMaj parameterVector;

    private final InverseDynamicsCalculator inverseDynamicsCalculator;

    private final SpatialInertiaParameterBasis spatialInertiaParameterBasis;

    private final RecursionStep initialRecursionStep;

    private final Map<RigidBodyReadOnly, RecursionStep> rigidBodyToRecursionStepMap = new LinkedHashMap<>();

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

    public JointTorqueRegressorCalculator(MultiBodySystemBasics input)
    {
        this.input = input;
        inverseDynamicsCalculator = new InverseDynamicsCalculator(input);
        spatialInertiaParameterBasis = new SpatialInertiaParameterBasis();

        RigidBodyBasics rootBody = input.getRootBody();
        initialRecursionStep = new RecursionStep(rootBody, null, inverseDynamicsCalculator, null);
        rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
        buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore());

        int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
        int nBodies = rigidBodyToRecursionStepMap.size() - 1;  // -1 removes the root body
        int nParams = 10 * nBodies;

        jointTorqueRegressorMatrix = new DMatrixRMaj(nDoFs, nParams);
        parameterVector = new DMatrixRMaj(nParams, 1);
        collectParameterVectorFromRecursionSteps();
    }

    private void buildMultiBodyTree(RecursionStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
    {
        List<JointReadOnly> childrenJoints = new ArrayList<>(parent.rigidBody.getChildrenJoints());

        if (childrenJoints.size() > 1)
        { // Reorganize the joints in the children to ensure that loop closures are treated last.
            List<JointReadOnly> loopClosureAncestors = new ArrayList<>();

            for (int i = 0; i < childrenJoints.size();)
            {
                if (MultiBodySystemTools.doesSubtreeContainLoopClosure(childrenJoints.get(i).getSuccessor()))
                    loopClosureAncestors.add(childrenJoints.remove(i));
                else
                    i++;
            }
            childrenJoints.addAll(loopClosureAncestors);
        }

        for (JointReadOnly childJoint : childrenJoints)
        {
            if (jointsToIgnore.contains(childJoint))
                continue;

            if (childJoint.isLoopClosure())
            {
                System.out.println(getClass().getSimpleName() + ": This calculator does not support kinematic loops. Ignoring joint: " + childJoint.getName());
                continue;
            }

            RigidBodyBasics childBody = (RigidBodyBasics) childJoint.getSuccessor();
            if (childBody != null)
            {
                int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
                RecursionStep child = new RecursionStep(childBody, parent, inverseDynamicsCalculator, jointIndices);
                rigidBodyToRecursionStepMap.put(childBody, child);
                buildMultiBodyTree(child, jointsToIgnore);
            }
        }
    }

    public void compute()
    {
        // Set all spatial inertias to zero
        initialRecursionStep.setSpatialInertiasToZeroRecursively();

        initialRecursionStep.calculateRegressorColumnsRecursively();

        // Collect regressor results for all recursion steps together
        collectJointTorqueRegressorFromRecursionSteps();
    }

    public void collectJointTorqueRegressorFromRecursionSteps()
    {
        int i = 0;
        for (RecursionStep step : rigidBodyToRecursionStepMap.values())
        {
            if (step.rigidBody.getInertia() != null) {
                CommonOps_DDRM.insert(step.regressorMatrixBlock, jointTorqueRegressorMatrix, 0, i * 10);
                i += 1;
            }
        }
    }

    /** NOTE: you can only use this *BEFORE* performing any regressor calculations. The reason for this is that regressor
     * calculations involve manipulation of the multibody system's spatial inertia information, corrupting it from the
     * initial input state. For that reason, only use this method before any regressor calculation, preferably as soon
     * as possible after the constructor.
     */
    public void collectParameterVectorFromRecursionSteps()
    {
        int i = 0;
        for (RecursionStep step : rigidBodyToRecursionStepMap.values())
        {
            if (step.rigidBody.getInertia() != null) {
                CommonOps_DDRM.insert(spatialInertiaToParameterVector(step.rigidBody.getInertia()),
                                      parameterVector, i * 10, 0);
                i += 1;
            }
        }
    }

    public void setGravitationalAcceleration(double gravity)
    {
        setGravitationalAcceleration(0.0, 0.0, gravity);
    }

    public void setGravitationalAcceleration(double gravityX, double gravityY, double gravityZ)
    {
        inverseDynamicsCalculator.setGravitionalAcceleration(gravityX, gravityY, gravityZ);
    }

    public DMatrixRMaj getParameterVector()
    {
        return parameterVector;
    }

    public DMatrixRMaj getJointTorqueRegressorMatrix()
    {
        return  jointTorqueRegressorMatrix;
    }

    private static DMatrixRMaj spatialInertiaToParameterVector(SpatialInertiaBasics spatialInertia)
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

    static class SpatialInertiaParameterBasis extends SpatialInertia
    {
        public SpatialInertiaParameterBasis(RigidBodyBasics rigidBody)
        {
            super(rigidBody.getInertia().getBodyFrame(), rigidBody.getInertia().getReferenceFrame());
            setToZero();
        }

        public void setBasis(SpatialInertiaParameterBasisOptions basisOption)
        {
            setToZero();
            switch (basisOption)
            {
                case M -> setMass(1.0);
                case MCOM_X -> setCenterOfMassOffset(1.0, 0.0, 0.0);
                case MCOM_Y -> setCenterOfMassOffset(0.0, 1.0, 0.0);
                case MCOM_Z -> setCenterOfMassOffset(0.0, 0.0, 1.0);
                case I_XX -> setMomentOfInertia(1.0, 0.0, 0.0);
                case I_YY -> setMomentOfInertia(0.0, 1.0, 0.0);
                case I_ZZ -> setMomentOfInertia(0.0, 0.0, 1.0);
                case I_XY -> setMomentOfInertia(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
                case I_XZ -> setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
                case I_YZ -> setMomentOfInertia(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            }
        }

        @Override
        public void get(DMatrix matrixToPack)
        {
            get(0, 0, matrixToPack);
        }

        @Override
        public void get(int startRow, int startColumn, DMatrix matrixToPack)
        {
            // Set upper left block corresponding to moment of inertia as normal
            getMomentOfInertia().get(startRow, startColumn, matrixToPack);

            // For upper right block and lower left block, which correspond to CoM offsets, assume mass is 1.0 for
            // the time being
            MecanoTools.toTildeForm(1.0, getCenterOfMassOffset(), false, startRow, startColumn + 3, matrixToPack);
            MecanoTools.toTildeForm(1.0, getCenterOfMassOffset(), true, startRow + 3, startColumn, matrixToPack);

            // For lower right block, which is a diagonal matrix of mass values, as normal
            startRow += 3;
            startColumn += 3;

            for (int i = 0; i < 3; i++)
            {
                matrixToPack.set(startRow + i, startColumn + i, getMass());
                matrixToPack.set(startRow + i, startColumn + (i + 1) % 3, 0.0);
                matrixToPack.set(startRow + i, startColumn + (i + 2) % 3, 0.0);
            }
        }
    }

    private class RecursionStep
    {
        private final RigidBodyBasics rigidBody;

        private final RecursionStep parent;

        private final List<RecursionStep> children = new ArrayList<>();

        private InverseDynamicsCalculator inverseDynamicsCalculator;

        private SpatialInertiaParameterBasis spatialInertiaParameterBasis;

        /** Temporary variable to hold results of a column of the regressor matrix */
        private DMatrixRMaj regressorColumn;

        private DMatrixRMaj regressorMatrixBlock;

        public RecursionStep(RigidBodyBasics rigidBody, RecursionStep parent, InverseDynamicsCalculator inverseDynamicsCalculator, int[] jointsToIgnore)
        {
            this.rigidBody = rigidBody;
            this.parent = parent;
            this.inverseDynamicsCalculator = inverseDynamicsCalculator;

            if (parent != null)
            {
                parent.children.add(this);
                this.inverseDynamicsCalculator = inverseDynamicsCalculator;
                spatialInertiaParameterBasis = new SpatialInertiaParameterBasis(this.rigidBody);
                regressorColumn = new DMatrixRMaj(inverseDynamicsCalculator.getJointTauMatrix().numRows, 1);
                regressorMatrixBlock = new DMatrixRMaj(inverseDynamicsCalculator.getJointTauMatrix().numRows, 10);
            }
        }

        /** First sweep sets all spatial inertias to zero */
        public void setSpatialInertiasToZeroRecursively()
        {
            // Only make changes if the rigid body of concern isn't a null body like the root / elevator
            if (rigidBody.getInertia() != null) {
                rigidBody.getInertia().setToZero();
                inverseDynamicsCalculator.getBodyInertia(rigidBody).setToZero();
            }

            for (RecursionStep child : children) {
                child.setSpatialInertiasToZeroRecursively();
            }
        }

        /** Second pass, assuming that all spatial inertias have been zeroed in the first sweep (therefore there are
         * no spatial inertias earlier in the tree that we have to care about). Here, we pass in a basis and set the
         * spatial inertia of the current rigid body
         */
        public void calculateRegressorColumnsRecursively()
        {
            // Only calculate if the rigid body of concern isn't a null body like the root / elevator
            if (rigidBody.getInertia() != null)
            {
                for (SpatialInertiaParameterBasisOptions basis : SpatialInertiaParameterBasisOptions.values())
                {
                    // Set spatial inertia of this rigid body to be the desired basis
                    spatialInertiaParameterBasis.setBasis(basis);
                    rigidBody.getInertia().set(spatialInertiaParameterBasis);
                    inverseDynamicsCalculator.getBodyInertia(rigidBody).set(spatialInertiaParameterBasis);
                    inverseDynamicsCalculator.compute();
                    regressorColumn.set(inverseDynamicsCalculator.getJointTauMatrix());
                    setRegressorMatrixColumn(regressorColumn, basis);
                    rigidBody.getInertia().setToZero();
                    inverseDynamicsCalculator.getBodyInertia(rigidBody).setToZero();
                }
            }
            for (RecursionStep child : children) {
                child.calculateRegressorColumnsRecursively();
            }
        }

        public void setRegressorMatrixColumn(DMatrixRMaj regressorColumn, SpatialInertiaParameterBasisOptions basis)
        {
            CommonOps_DDRM.insert(regressorColumn, regressorMatrixBlock, 0, basis.ordinal());
        }
    }
}
