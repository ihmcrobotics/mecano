package us.ihmc.mecano.algorithms.interfaces;

import java.util.function.Function;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

public interface RigidBodyTwistProvider
{
   TwistReadOnly getTwistOfBody(RigidBodyReadOnly body);

   TwistReadOnly getRelativeTwist(RigidBodyReadOnly base, RigidBodyReadOnly body);

   default FrameVector3DReadOnly getLinearVelocityOfBodyFixedPoint(RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint)
   {
      return getLinearVelocityOfBodyFixedPoint(null, body, bodyFixedPoint);
   }

   FrameVector3DReadOnly getLinearVelocityOfBodyFixedPoint(RigidBodyReadOnly base, RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint);

   ReferenceFrame getInertialFrame();

   public static RigidBodyTwistProvider toRigidBodyTwistProvider(Function<RigidBodyReadOnly, TwistReadOnly> twistFunction, ReferenceFrame inertialFrame)
   {
      return new RigidBodyTwistProvider()
      {
         private final Twist twist = new Twist();
         private final FrameVector3D linearVelocity = new FrameVector3D();

         @Override
         public TwistReadOnly getTwistOfBody(RigidBodyReadOnly body)
         {
            return twistFunction.apply(body);
         }

         @Override
         public TwistReadOnly getRelativeTwist(RigidBodyReadOnly base, RigidBodyReadOnly body)
         {
            MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();

            TwistReadOnly immutableBaseTwist = getTwistOfBody(base);
            if (immutableBaseTwist == null)
               return null;

            twist.setIncludingFrame(immutableBaseTwist);

            TwistReadOnly immutableBodyTwist = getTwistOfBody(body);
            if (immutableBodyTwist == null)
               return null;

            twist.changeFrame(bodyFrame);
            twist.sub(immutableBodyTwist);
            twist.invert();

            return twist;
         }

         @Override
         public FrameVector3DReadOnly getLinearVelocityOfBodyFixedPoint(RigidBodyReadOnly base, RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint)
         {
            if (base != null)
               getRelativeTwist(base, body).getLinearVelocityAt(bodyFixedPoint, linearVelocity);
            else
               getTwistOfBody(body).getLinearVelocityAt(bodyFixedPoint, linearVelocity);
            return linearVelocity;
         }

         @Override
         public ReferenceFrame getInertialFrame()
         {
            return inertialFrame;
         }
      };
   }
}
