package us.ihmc.mecano.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Twist;

/**
 * {@code MovingCenterOfMassReferenceFrame} is a reference frame that is centered at the center of
 * mass of a multi-rigid-body system attached to a {@code rootBody}.
 * <p>
 * In addition to providing the center of mass location as a reference frame, the
 * {@code MovingCenterOfMassReferenceFrame} can be used to obtain the twist of the center of mass
 * with its angular part being zero, i.e. no angular velocity.
 * </p>
 */
public class MovingCenterOfMassReferenceFrame extends MovingReferenceFrame
{
   /** The calculator used to update the position and velocity of the center of mass. */
   private final CenterOfMassJacobian centerOfMassJacobian;

   /**
    * Creates a new moving center of mass reference frame for the subtree defined by its root:
    * {@code rootBody}.
    * 
    * @param frameName   the name of this new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param rootBody    the center of mass position and velocity are computed given {@code rootBody}
    *                    and all its descendants.
    */
   public MovingCenterOfMassReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyReadOnly rootBody)
   {
      this(frameName, new CenterOfMassJacobian(rootBody, parentFrame));
   }

   /**
    * Creates a new center of mass reference frame that uses the given {@code centerOfMassCalculator}
    * to update its position.
    * 
    * @param frameName            the name of this new frame.
    * @param centerOfMassJacobian the calculator to use to update the position and velocity of this
    *                             frame.
    */
   public MovingCenterOfMassReferenceFrame(String frameName, CenterOfMassJacobian centerOfMassJacobian)
   {
      super(frameName, centerOfMassJacobian.getReferenceFrame());
      this.centerOfMassJacobian = centerOfMassJacobian;
   }

   /** {@inheritDoc} */
   @Override
   public void update()
   {
      centerOfMassJacobian.reset();
      super.update();
   }

   /** {@inheritDoc} */
   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.setTranslationAndIdentityRotation(centerOfMassJacobian.getCenterOfMass());
   }

   /** {@inheritDoc} */
   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      twistRelativeToParentToPack.setToZero(this, getParent(), this);
      twistRelativeToParentToPack.getLinearPart().setMatchingFrame(centerOfMassJacobian.getCenterOfMassVelocity());
   }
}