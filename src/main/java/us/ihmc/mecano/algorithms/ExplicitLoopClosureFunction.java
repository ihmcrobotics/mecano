package us.ihmc.mecano.algorithms;

import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public interface ExplicitLoopClosureFunction
{
   List<? extends JointReadOnly> getKinematicLoopJoints();

   void recomputeJointEfforts(Map<JointReadOnly, JointEffortData> jointDataToUpdate, SpatialForceReadOnly spatialForceFromChildren);

   WrenchReadOnly getWrenchForPredecessor();

   public static interface JointEffortData
   {
      DenseMatrix64F getTau();

      FixedFrameWrenchBasics getJointWrench();
   }
}
