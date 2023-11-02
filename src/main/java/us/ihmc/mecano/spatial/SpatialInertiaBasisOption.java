package us.ihmc.mecano.spatial;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Random;

/**
 * A {@code SpatialInertia} object is linear with respect to the ten inertial parameters that constitute it.
 * Therefore, we can create a basis for all spatial inertias with respect to these parameters.
 */
public enum SpatialInertiaBasisOption
{
   M, MCOM_X, MCOM_Y, MCOM_Z, I_XX, I_XY, I_XZ, I_YY, I_YZ, I_ZZ;

   public static final EnumSet<SpatialInertiaBasisOption> set = EnumSet.allOf(SpatialInertiaBasisOption.class);
   public static final SpatialInertiaBasisOption[] values = values();

   public static ArrayList<SpatialInertiaBasisOption> generateRandomBasisSetForLink(Random random)
   {
      ArrayList<SpatialInertiaBasisOption> result = new ArrayList<>();
      for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values())
      {
         if (random.nextBoolean())
            result.add(option);
      }
      return result;
   }
}