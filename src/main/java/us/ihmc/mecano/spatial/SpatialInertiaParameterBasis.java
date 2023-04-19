package us.ihmc.mecano.spatial;

import org.ejml.data.DMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.tools.MecanoTools;

public class SpatialInertiaParameterBasis extends SpatialInertia
{
    public SpatialInertiaParameterBasis()
    {
        super();
    }

    public SpatialInertiaParameterBasis(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
    {
        setToZero(bodyFrame, expressedInFrame);
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
