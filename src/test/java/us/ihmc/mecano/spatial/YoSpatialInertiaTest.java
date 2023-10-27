package us.ihmc.mecano.spatial;

import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoSpatialInertia;

public class YoSpatialInertiaTest extends SpatialInertiaBasicsTest<YoSpatialInertia>
{
   @Override
   public YoSpatialInertia copySpatialInertia(SpatialInertiaReadOnly other)
   {
      return new YoSpatialInertia(other, null, null);
   }
}
