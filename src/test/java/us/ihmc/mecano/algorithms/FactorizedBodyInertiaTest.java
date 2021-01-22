package us.ihmc.mecano.algorithms;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;

public class FactorizedBodyInertiaTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testSetFromInertiaAndTwist()
   {
      Random random = new Random(4366);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      {
         SpatialInertia spatialInertia = MecanoRandomTools.nextSpatialInertia(random, worldFrame, worldFrame);
         Twist bodyTwist = MecanoRandomTools.nextTwist(random, worldFrame, worldFrame, worldFrame);
         
         Wrench expectedWrench = new Wrench(worldFrame, worldFrame);
         spatialInertia.computeDynamicWrench(null, bodyTwist, expectedWrench);
         
         FactorizedBodyInertia factorizedBodyInertia = new FactorizedBodyInertia(worldFrame);
         factorizedBodyInertia.setIncludingFrame(spatialInertia, bodyTwist);
         Wrench actualWrench = new Wrench(worldFrame, worldFrame);
         factorizedBodyInertia.getAngularInertia().transform(bodyTwist.getAngularPart(), actualWrench.getAngularPart());
         factorizedBodyInertia.getTopRightInertia().addTransform(bodyTwist.getLinearPart(), actualWrench.getAngularPart());
         factorizedBodyInertia.getLinearInertia().transform(bodyTwist.getLinearPart(), actualWrench.getLinearPart());
         factorizedBodyInertia.getBottomLeftInertia().addTransform(bodyTwist.getAngularPart(), actualWrench.getLinearPart());
         
         MecanoTestTools.assertWrenchEquals(expectedWrench, actualWrench, EPSILON);
      }
   }

   @Test
   public void testApplyTransform()
   {
      Random random = new Random(4578);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         SpatialInertia spatialInertia = MecanoRandomTools.nextSpatialInertia(random, worldFrame, worldFrame);
         Twist bodyTwist = MecanoRandomTools.nextTwist(random, worldFrame, worldFrame, worldFrame);
         
         Wrench expectedWrench = new Wrench(worldFrame, worldFrame);
         spatialInertia.computeDynamicWrench(null, bodyTwist, expectedWrench);
         expectedWrench.applyTransform(transform);
         
         FactorizedBodyInertia factorizedBodyInertia = new FactorizedBodyInertia(worldFrame);
         factorizedBodyInertia.setIncludingFrame(spatialInertia, bodyTwist);
         factorizedBodyInertia.applyTransform(transform);
         bodyTwist.applyTransform(transform);

         Wrench actualWrench = new Wrench(worldFrame, worldFrame);
         factorizedBodyInertia.getAngularInertia().transform(bodyTwist.getAngularPart(), actualWrench.getAngularPart());
         factorizedBodyInertia.getTopRightInertia().addTransform(bodyTwist.getLinearPart(), actualWrench.getAngularPart());
         factorizedBodyInertia.getLinearInertia().transform(bodyTwist.getLinearPart(), actualWrench.getLinearPart());
         factorizedBodyInertia.getBottomLeftInertia().addTransform(bodyTwist.getAngularPart(), actualWrench.getLinearPart());
         
         MecanoTestTools.assertWrenchEquals(expectedWrench, actualWrench, EPSILON);
      }
   }

   @Test
   public void testApplyInverseTransform()
   {
      Random random = new Random(4578);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         SpatialInertia spatialInertia = MecanoRandomTools.nextSpatialInertia(random, worldFrame, worldFrame);
         Twist bodyTwist = MecanoRandomTools.nextTwist(random, worldFrame, worldFrame, worldFrame);
         
         Wrench expectedWrench = new Wrench(worldFrame, worldFrame);
         spatialInertia.computeDynamicWrench(null, bodyTwist, expectedWrench);
         expectedWrench.applyInverseTransform(transform);
         
         FactorizedBodyInertia factorizedBodyInertia = new FactorizedBodyInertia(worldFrame);
         factorizedBodyInertia.setIncludingFrame(spatialInertia, bodyTwist);
         factorizedBodyInertia.applyInverseTransform(transform);
         bodyTwist.applyInverseTransform(transform);

         Wrench actualWrench = new Wrench(worldFrame, worldFrame);
         factorizedBodyInertia.getAngularInertia().transform(bodyTwist.getAngularPart(), actualWrench.getAngularPart());
         factorizedBodyInertia.getTopRightInertia().addTransform(bodyTwist.getLinearPart(), actualWrench.getAngularPart());
         factorizedBodyInertia.getLinearInertia().transform(bodyTwist.getLinearPart(), actualWrench.getLinearPart());
         factorizedBodyInertia.getBottomLeftInertia().addTransform(bodyTwist.getAngularPart(), actualWrench.getLinearPart());
         
         MecanoTestTools.assertWrenchEquals(expectedWrench, actualWrench, EPSILON);
      }
   }
}
