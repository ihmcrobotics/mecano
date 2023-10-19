package us.ihmc.mecano.spatial;

import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

public class SpatialInertiaTest extends SpatialInertiaBasicsTest<SpatialInertia>
{
   @Override
   public SpatialInertia copySpatialInertia(SpatialInertiaReadOnly other)
   {
      return new SpatialInertia(other);
   }
}
