package us.ihmc.mecano.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.spatial.Twist;

public class FixedMovingReferenceFrame extends MovingReferenceFrame
{
   public FixedMovingReferenceFrame(String frameName, ReferenceFrame parentFrame, Tuple3DReadOnly translationOffsetFromParent)
   {
      this(frameName, parentFrame, new RigidBodyTransform(EuclidCoreTools.neutralQuaternion, translationOffsetFromParent));
   }

   public FixedMovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      super(frameName, parentFrame, transformToParent, parentFrame.isZupFrame() && transformToParent.isRotation2D(), true);
   }

   @Override
   public void update()
   {
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
