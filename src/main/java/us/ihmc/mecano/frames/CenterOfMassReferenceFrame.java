package us.ihmc.mecano.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

/**
 * The center of mass reference frame has its origin located at the center of mass of a multi-body
 * system and its orientation coincide with the world coordinates at all times.
 */
public class CenterOfMassReferenceFrame extends ReferenceFrame
{
   /** The calculator used to update the position of the center of mass. */
   private final CenterOfMassCalculator centerOfMassCalculator;

   /**
    * Creates a new center of mass reference frame for the subtree defined by its root:
    * {@code rootBody}.
    * 
    * @param frameName   the name of this new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param rootBody    the center of mass is computed given {@code rootBody} and all its descendants.
    */
   public CenterOfMassReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyReadOnly rootBody)
   {
      this(frameName, new CenterOfMassCalculator(rootBody, parentFrame));
   }

   /**
    * Creates a new center of mass reference frame that uses the given {@code centerOfMassCalculator}
    * to update its position.
    * 
    * @param frameName              the name of this new frame.
    * @param centerOfMassCalculator the calculator to use to update the position of this frame.
    */
   public CenterOfMassReferenceFrame(String frameName, CenterOfMassCalculator centerOfMassCalculator)
   {
      super(frameName, centerOfMassCalculator.getReferenceFrame());
      this.centerOfMassCalculator = centerOfMassCalculator;
   }

   /** {@inheritDoc} */
   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      centerOfMassCalculator.reset();
      transformToParent.setIdentity();
      transformToParent.getTranslation().set(centerOfMassCalculator.getCenterOfMass());
   }
}