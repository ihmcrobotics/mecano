package us.ihmc.mecano.algorithms;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public interface ExplicitLoopClosureFunction
{
   List<? extends JointReadOnly> getKinematicLoopJoints();

   void recomputeJointEfforts(DenseMatrix64F tauMatrixToUpdate);

   void updateJointWrench(FixedFrameWrenchBasics jointWrenchToUpdate);

   WrenchReadOnly getWrenchForPredecessor();
}
