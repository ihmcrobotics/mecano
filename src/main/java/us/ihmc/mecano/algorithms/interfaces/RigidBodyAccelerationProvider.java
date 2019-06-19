package us.ihmc.mecano.algorithms.interfaces;

import java.util.function.Function;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;

/**
 * Interface for algorithms that can compute and provide the rigid-body spatial acceleration of
 * multi-body system.
 * <p>
 * While the common calculator to compute rigid-body accelerations is the
 * {@link SpatialAccelerationCalculator}, a rigid-body provider can also be obtained from other
 * algorithms such as {@link InverseDynamicsCalculator} and {@link ForwardDynamicsCalculator}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface RigidBodyAccelerationProvider
{
   /**
    * Updates if necessary and packs the acceleration of the given {@code body}.
    * <p>
    * The result is the acceleration of the {@code body.getBodyFixedFrame()}, with respect to the
    * {@code inertialFrame}, expressed in the {@code body.getBodyFixedFrame()}.
    * </p>
    * 
    * @param body the rigid-body to get the acceleration of.
    * @return the acceleration of the {@code body}.
    */
   SpatialAccelerationReadOnly getAccelerationOfBody(RigidBodyReadOnly body);

   /**
    * Computes and packs the spatial acceleration of the {@code body} relative to the given
    * {@code base}. The resulting spatial acceleration is the acceleration of the
    * {@code body.getBodyFixedFrame()} with respect to the {@code base.getBodyFixedFrame()}, expressed
    * in {@code body.getBodyFixedFrame()}.
    * <p>
    * The relative acceleration between the two rigid-bodies is calculated knowing their accelerations
    * with respect to the inertial frame using the method
    * {@link #getAccelerationOfBody(RigidBodyReadOnly)}: <br>
    * A<sup>b2, b2</sup><sub>b1</sub> = A<sup>b2, b2</sup><sub>i</sub> - A<sup>b1, b2</sup><sub>i</sub>
    * </br>
    * with 'b1' being the {@code base}, 'b2' the {@code body}, and 'i' the {@code inertialFrame}.
    * </p>
    * 
    * @param base the rigid-body with respect to which the acceleration is to be computed.
    * @param body the rigid-body to compute the acceleration of.
    * @return the acceleration of {@code body} with respect to {@code base}.
    */
   SpatialAccelerationReadOnly getRelativeAcceleration(RigidBodyReadOnly base, RigidBodyReadOnly body);

   /**
    * Computes and packs the linear acceleration of the point {@code bodyFixedPoint} that is attached
    * to {@code body} with respect to {@code inertialFrame}.
    * 
    * @param body           the rigid-body to which {@code bodyFixedPoint} belongs.
    * @param bodyFixedPoint the point to compute the linear acceleration of. Not modified.
    * @return the linear acceleration of the body fixed point.
    */
   default FrameVector3DReadOnly getLinearAccelerationOfBodyFixedPoint(RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint)
   {
      return getLinearAccelerationOfBodyFixedPoint(null, body, bodyFixedPoint);
   }

   /**
    * Computes and packs the linear acceleration of the point {@code bodyFixedPoint} that is attached
    * to {@code body} with respect to {@code base}.
    * 
    * @param base           the rigid-body with respect to which the acceleration is to be computed.
    * @param body           the rigid-body to which {@code bodyFixedPoint} belongs.
    * @param bodyFixedPoint the point to compute the linear acceleration of. Not modified.
    * @return the linear acceleration of the body fixed point.
    */
   FrameVector3DReadOnly getLinearAccelerationOfBodyFixedPoint(RigidBodyReadOnly base, RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint);

   /**
    * Whether rigid-body accelerations resulting from centrifugal and Coriolis effects are considered
    * or ignored.
    * 
    * @return {@code true} if this provider considers the velocity terms, {@code false} otherwise.
    */
   default boolean areVelocitiesConsidered()
   {
      return true;
   }

   /**
    * Whether rigid-body accelerations resulting from joint accelerations are considered or ignored.
    * 
    * @return {@code true} if this provider considers joint accelerations, {@code false} otherwise.
    */
   default boolean areAccelerationsConsidered()
   {
      return true;
   }

   /**
    * Gets the inertial frame used with this acceleration provider.
    * <p>
    * The acceleration of each rigid-body is expressed with respect to this inertial frame, it is
    * commonly equal to {@link ReferenceFrame#getWorldFrame()}.
    * </p>
    *
    * @return the inertial frame.
    */
   default ReferenceFrame getInertialFrame()
   {
      return ReferenceFrame.getWorldFrame();
   }

   /**
    * Factory for implementing a {@link RigidBodyAccelerationProvider} from a given function that
    * computes spatial acceleration any given rigid-body.
    * <p>
    * The function is used to implement {@link #getAccelerationOfBody(RigidBodyReadOnly)} and is enough
    * to implement the other acceleration getters of this interface.
    * </p>
    * 
    * @param accelerationFunction the function that computes rigid-body accelerations.
    * @param inertialFrame        the inertial frame with respect to which the body accelerations are
    *                             expressed.
    * @return a new instance of {@code RigidBodyAccelerationProvider}.
    */
   public static RigidBodyAccelerationProvider toRigidBodyAccelerationProvider(Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> accelerationFunction,
                                                                               ReferenceFrame inertialFrame)
   {
      return toRigidBodyAccelerationProvider(accelerationFunction, inertialFrame, true, true);
   }

   /**
    * Factory for implementing a {@link RigidBodyAccelerationProvider} from a given function that
    * computes spatial acceleration any given rigid-body.
    * <p>
    * The function is used to implement {@link #getAccelerationOfBody(RigidBodyReadOnly)} and is enough
    * to implement the other acceleration getters of this interface.
    * </p>
    * 
    * @param accelerationFunction  the function that computes rigid-body accelerations.
    * @param inertialFrame         the inertial frame with respect to which the body accelerations are
    *                              expressed.
    * @param considerVelocities    whether the Coriolis and centrifugal resulting accelerations should
    *                              be considered.
    * @param considerAccelerations whether the joint accelerations are considered by the function.
    * @return a new instance of {@code RigidBodyAccelerationProvider}.
    */
   public static RigidBodyAccelerationProvider toRigidBodyAccelerationProvider(Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> accelerationFunction,
                                                                               ReferenceFrame inertialFrame, boolean considerVelocities,
                                                                               boolean considerAccelerations)
   {
      return new RigidBodyAccelerationProvider()
      {
         private final Twist deltaTwist = new Twist();
         private final SpatialAcceleration baseAcceleration = new SpatialAcceleration();
         private final SpatialAcceleration acceleration = new SpatialAcceleration();

         @Override
         public SpatialAccelerationReadOnly getAccelerationOfBody(RigidBodyReadOnly body)
         {
            return accelerationFunction.apply(body);
         }

         @Override
         public SpatialAccelerationReadOnly getRelativeAcceleration(RigidBodyReadOnly base, RigidBodyReadOnly body)
         {
            MovingReferenceFrame baseFrame = base.getBodyFixedFrame();
            MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();

            baseAcceleration.setIncludingFrame(getAccelerationOfBody(base));
            acceleration.setIncludingFrame(getAccelerationOfBody(body));

            if (areVelocitiesConsidered())
            {
               baseFrame.getTwistRelativeToOther(bodyFrame, deltaTwist);
               baseAcceleration.changeFrame(bodyFrame, deltaTwist, baseFrame.getTwistOfFrame());
            }
            else
            {
               baseAcceleration.changeFrame(bodyFrame);
            }

            acceleration.sub(baseAcceleration);

            return acceleration;
         }

         private final FrameVector3D linearAcceleration = new FrameVector3D();
         private final FramePoint3D localBodyFixedPoint = new FramePoint3D();
         private final Twist twistForLinearAcceleration = new Twist();

         @Override
         public FrameVector3DReadOnly getLinearAccelerationOfBodyFixedPoint(RigidBodyReadOnly base, RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint)
         {
            SpatialAccelerationReadOnly accelerationToUse = base != null ? getRelativeAcceleration(base, body) : getAccelerationOfBody(body);

            MovingReferenceFrame bodyFrame = body.getBodyFixedFrame();
            ReferenceFrame baseFrame = accelerationToUse.getBaseFrame();

            localBodyFixedPoint.setIncludingFrame(bodyFixedPoint);
            localBodyFixedPoint.changeFrame(bodyFrame);

            if (areVelocitiesConsidered())
               bodyFrame.getTwistRelativeToOther(baseFrame, twistForLinearAcceleration);
            else
               twistForLinearAcceleration.setToZero(bodyFrame, baseFrame, bodyFrame);

            accelerationToUse.getLinearAccelerationAt(twistForLinearAcceleration, localBodyFixedPoint, linearAcceleration);
            linearAcceleration.changeFrame(baseFrame);

            return linearAcceleration;
         }

         @Override
         public boolean areVelocitiesConsidered()
         {
            return considerVelocities;
         }

         @Override
         public boolean areAccelerationsConsidered()
         {
            return considerAccelerations;
         }

         @Override
         public ReferenceFrame getInertialFrame()
         {
            return inertialFrame;
         }
      };
   }
}
