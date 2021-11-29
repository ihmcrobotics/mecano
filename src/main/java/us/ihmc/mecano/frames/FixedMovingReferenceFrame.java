package us.ihmc.mecano.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.spatial.Twist;

/**
 * Similar to {@code FixedReferenceFrame}, {@code FixedMovingReferenceFrame} represents a reference
 * frame that is rigidly attached to its parent, i.e. the transform to its parent is constant.
 * 
 * @author Sylvain Bertrand
 */
public class FixedMovingReferenceFrame extends MovingReferenceFrame
{
   /**
    * Creates a new fixed-frame.
    * <p>
    * The new frame has the same orientation as its parent frame.
    * </p>
    * 
    * @param frameName                   the name of the new frame.
    * @param parentFrame                 the parent frame of the new reference frame.
    * @param translationOffsetFromParent describes the position of the new reference frame's origin
    *                                    expressed in the parent frame. Not modified.
    */
   public FixedMovingReferenceFrame(String frameName, ReferenceFrame parentFrame, Tuple3DReadOnly translationOffsetFromParent)
   {
      this(frameName, parentFrame, new RigidBodyTransform(EuclidCoreTools.neutralQuaternion, translationOffsetFromParent));
   }

   /**
    * Creates a new fixed-frame.
    * <p>
    * The {@code transformToParent} should describe the pose of the new frame expressed in its parent
    * frame.
    * </p>
    * 
    * @param frameName         the name of the new frame.
    * @param parentFrame       the parent frame of the new reference frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *                          frame to its parent frame. Not modified.
    */
   public FixedMovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      super(frameName, parentFrame, transformToParent, parentFrame.isZupFrame() && transformToParent.isRotation2D(), true);
   }

   @Override
   public void update()
   {
      checkIfRemoved();
      // Since it is fixed, the transform does not change.
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
   }
}
