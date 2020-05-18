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

/**
 * Interface for algorithms that can compute and provide the rigid-body twists of a multi-body
 * system.
 *
 * @author Sylvain Bertrand
 */
public interface RigidBodyTwistProvider
{
   /**
    * Updates if necessary and packs the twist of the given {@code body}.
    * <p>
    * The result is the twist of the {@code body.getBodyFixedFrame()}, with respect to the
    * {@code inertialFrame}, expressed in the {@code body.getBodyFixedFrame()}.
    * </p>
    *
    * @param body the rigid-body to get the twist of.
    * @return the twist of the {@code body}.
    */
   TwistReadOnly getTwistOfBody(RigidBodyReadOnly body);

   /**
    * Computes and packs the twist of the {@code body} relative to the given {@code base}.
    * <p>
    * The result is the twist of the {@code body.getBodyFixedFrame()} with respect to the
    * {@code base.getBodyFixedFrame()}, expressed in {@code body.getBodyFixedFrame()}.
    * </p>
    * <p>
    * The relative twist between the two rigid-bodies is calculated knowing their twists with respect
    * to the inertial frame using the method {@link #getTwistOfBody(RigidBodyReadOnly)}: <br>
    * T<sup>b2, b2</sup><sub>b1</sub> = T<sup>b2, b2</sup><sub>i</sub> - T<sup>b1, b2</sup><sub>i</sub>
    * </br>
    * with 'b1' being the {@code base}, 'b2' the {@code body}, and 'i' the {@code inertialFrame}.
    * </p>
    *
    * @param base the rigid-body with respect to which the twist is to be computed.
    * @param body the rigid-body to compute the twist of.
    * @return the twist of {@code body} with respect to {@code base}.
    */
   TwistReadOnly getRelativeTwist(RigidBodyReadOnly base, RigidBodyReadOnly body);

   /**
    * Computes and packs the linear twist of the point {@code bodyFixedPoint} that is attached to
    * {@code body} with respect to {@code inertialFrame}.
    *
    * @param body           the rigid-body to which {@code bodyFixedPoint} belongs.
    * @param bodyFixedPoint the point to compute the linear twist of. Not modified.
    * @return the linear twist of the body fixed point.
    */
   default FrameVector3DReadOnly getLinearVelocityOfBodyFixedPoint(RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint)
   {
      return getLinearVelocityOfBodyFixedPoint(null, body, bodyFixedPoint);
   }

   /**
    * Computes and packs the linear twist of the point {@code bodyFixedPoint} that is attached to
    * {@code body} with respect to {@code base}.
    *
    * @param base           the rigid-body with respect to which the twist is to be computed.
    * @param body           the rigid-body to which {@code bodyFixedPoint} belongs.
    * @param bodyFixedPoint the point to compute the linear twist of. Not modified.
    * @return the linear twist of the body fixed point.
    */
   FrameVector3DReadOnly getLinearVelocityOfBodyFixedPoint(RigidBodyReadOnly base, RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint);

   /**
    * Gets the inertial frame used with this twist provider.
    * <p>
    * The twist of each rigid-body is expressed with respect to this inertial frame, it is commonly
    * equal to {@link ReferenceFrame#getWorldFrame()}.
    * </p>
    *
    * @return the inertial frame.
    */
   ReferenceFrame getInertialFrame();

   /**
    * Factory for implementing a {@link RigidBodyTwistProvider} from a given function that computes
    * twist for any given rigid-body.
    * <p>
    * The function is used to implement {@link #getTwistOfBody(RigidBodyReadOnly)} and is enough to
    * implement the other twist getters of this interface.
    * </p>
    *
    * @param twistFunction the function that computes rigid-body twists.
    * @param inertialFrame the inertial frame with respect to which the body twists are expressed.
    * @return a new instance of {@code RigidBodyTwistProvider}.
    */
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
