package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

/**
 * Write and read interface for a spatial acceleration which reference frames can not be changed.
 * <p>
 * A spatial acceleration is a vector composed of 6 components with an angular part and a linear
 * part. It represents the time derivative of a twist. The angular part represents a 3D angular
 * acceleration and the linear part a 3D linear acceleration.
 * </p>
 * <p>
 * A spatial acceleration always describes the relative velocity of a body with respect to a base.
 * These two entities are referred to here by using two reference frames: a {@code bodyFrame} that
 * is considered to be rigidly attached to the body, and a {@code baseFrame} that is considered to
 * be rigidly attached to the base.
 * </p>
 * <p>
 * When using a {@code FixedFrameSpatialAccelerationBasics}, it is important to note that the
 * reference frame in which it is expressed does not only refer to the coordinate system in which
 * the angular and linear 3D vectors are expressed. The origin of the reference frame is also used
 * as the point where the acceleration is measured. While the angular part remains the same as the
 * point of measurement changes, the linear part does depend on its location.
 * </p>
 * <p>
 * This framework for representing in an accurate and safe manner spatial accelerations is based on
 * the Ph.D. thesis of Vincent Duindam entitled <i>"Port-Based Modeling and Control for Efficient
 * Bipedal Walking Robots"</i>. Duindam's publications can be found
 * <a href="http://sites.google.com/site/vincentduindam/publications">here</a>. Several references
 * to this work are spread throughout the code.
 * </p>
 * <p>
 * The convention when using a spatial vector in matrix operations is that the angular part occupies
 * the 3 first rows and the linear part the 3 last as follows:<br>
 *
 * <pre>
 *     / angularX \
 *     | angularY |
 *     | angularZ |
 * V = | linearX  |
 *     | linearY  |
 *     \ linearX  /
 * </pre>
 * </p>
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface FixedFrameSpatialAccelerationBasics extends SpatialAccelerationReadOnly, FixedFrameSpatialMotionBasics
{
   /**
    * Sets this spatial acceleration given the angular and linear acceleration of the body frame
    * perceived from the base frame.
    * <p>
    * It is often more intuitive to quantify a body acceleration from the perspective of the base
    * frame. However, when creating a spatial acceleration, the frame from which the acceleration is
    * perceived and the point of which the acceleration is measured are usually part of the same frame.
    * This implies an actual difference between the two representations due to Coriolis accelerations
    * solely introduced because the two reference frames are moving with respect to each other.
    * </p>
    * <p>
    * This method allows to change from the intuitive representation, to the one used in this
    * framework. This spatial acceleration is updated as follows:
    *
    * <pre>
    * &omega;'<sub>new</sub> = &omega;'<sub>arg</sub>
    * &alpha;<sub>new</sub> = &alpha;<sub>arg</sub> + &nu;<sub>twist</sub> &times; &omega;<sub>twist</sub>
    * </pre>
    * </p>
    *
    * @param angularAcceleration the angular acceleration of the body. Not modified.
    * @param originAcceleration  the linear acceleration of the body frame's origin perceived from the
    *                            base frame. Not modified.
    * @param bodyTwist           this is the twist of {@code this.bodyFrame} with respect to
    *                            {@code this.baseFrame} and expressed in {@code this.expressedInFrame}.
    * @throws ReferenceFrameMismatchException if {@code this.bodyFrame != this.expressedInFrame}, or if
    *                                         {@code bodyTwist} does not have the same frames as
    *                                         {@code this}.
    */
   default void setBasedOnOriginAcceleration(FrameVector3DReadOnly angularAcceleration, FrameVector3DReadOnly originAcceleration, TwistReadOnly bodyTwist)
   {
      getBodyFrame().checkReferenceFrameMatch(getReferenceFrame());
      checkReferenceFrameMatch(bodyTwist);

      getAngularPart().setMatchingFrame(angularAcceleration);
      getLinearPart().setMatchingFrame(originAcceleration);
      // Note that the cross terms are swapped with respect to the corresponding getter, this is to perform a subtraction.
      addCrossToLinearPart(bodyTwist.getLinearPart(), bodyTwist.getAngularPart());
   }
}
