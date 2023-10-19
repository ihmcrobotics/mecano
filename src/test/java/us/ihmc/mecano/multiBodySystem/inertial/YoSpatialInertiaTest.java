package us.ihmc.mecano.multiBodySystem.inertial;

import us.ihmc.mecano.spatial.SpatialInertiaBasicsTest;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.yoVariables.multiBodySystem.inertial.YoSpatialInertia;

public class YoSpatialInertiaTest extends SpatialInertiaBasicsTest<YoSpatialInertia>
{
   @Override
   public YoSpatialInertia copySpatialInertia(SpatialInertiaReadOnly other)
   {
      return new YoSpatialInertia(other, null, null);
   }
}
