package us.ihmc.mecano.spatial;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.mecano.tools.MecanoRandomTools;

public class MomentumTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testComputeKineticCoEnergy()
   {
      Random random = new Random(1776L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame baseFrame = ReferenceFrame.getWorldFrame();
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame expressedInFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialInertia spatialInertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, baseFrame, expressedInFrame);
         double expected = spatialInertia.computeKineticCoEnergy(twist);

         Momentum momentum = new Momentum(expressedInFrame);
         momentum.compute(spatialInertia, twist);
         double actual = momentum.computeKineticCoEnergy(twist);

         assertEquals(expected, actual, EPSILON);
      }
   }
}
