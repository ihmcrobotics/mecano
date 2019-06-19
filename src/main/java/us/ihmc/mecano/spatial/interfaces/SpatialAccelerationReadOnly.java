package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Read-only interface for a spatial acceleration.
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
 * When using a {@code SpatialAccelerationVectorReadOnly}, it is important to note that the
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
public interface SpatialAccelerationReadOnly extends SpatialMotionReadOnly
{
   /**
    * Calculates the linear acceleration of a body-fixed point {@code bodyFixedPoint} perceived from
    * the base frame.
    * <p>
    * The "perceived from the base frame" means that the body-fixed point is observed from the base,
    * such that Coriolis acceleration resulting from the body velocity with respect to the base is
    * considered.
    * </p>
    * <p>
    * Effectively, the resulting linear acceleration &alpha;<sub>result</sub> is calculated as follows:
    *
    * <pre>
    * &alpha;<sub>result</sub> = &alpha;<sub>this</sub> + &omega;'<sub>this</sub> &times; P + &omega;<sub>twist</sub> &times; ((&omega;<sub>twist</sub> &times; P) + &nu;<sub>twist</sub>)
    * </pre>
    *
    * where &alpha;<sub>this</sub> and &omega;'<sub>this</sub> represent the angular and linear parts
    * of this spatial acceleration, &omega;<sub>twist</sub> and &nu;<sub>twist</sub> represent the
    * angular and linear parts of the body twist, and {@code P} is the {@code bodyFixedPoint}.
    * </p>
    *
    * @param bodyTwist                the twist of {@code this.bodyFrame} with respect to
    *                                 {@code this.baseFrame} and expressed in
    *                                 {@code this.expressedInFrame}. Not modified.
    * @param bodyFixedPoint           the position on the body where the linear acceleration is to be
    *                                 estimated. Not modified.
    * @param linearAccelerationToPack the vector used to store the result. Modified.
    * @throws ReferenceFrameMismatchException if the {@code bodyTwist} does not have the same reference
    *                                         frames as {@code this}, if the {@code bodyFixedPoint} is
    *                                         not expressed in the same reference frame as
    *                                         {@code this}, or if this spatial acceleration is not
    *                                         expressed in either the body frame of the base frame.
    */
   default void getLinearAccelerationAt(TwistReadOnly bodyTwist, FramePoint3DReadOnly bodyFixedPoint, FrameVector3DBasics linearAccelerationToPack)
   {
      /*
       * Without this check, the "expressed-in-frame" could possibly be moving with respect to both the
       * base and the body. In such context, this acceleration would consider additional acceleration
       * induced by the motion of the "expressed-in-frame" itself which is undesirable here.
       */
      if (getReferenceFrame() != getBodyFrame() && getReferenceFrame() != getBaseFrame())
         throw new ReferenceFrameMismatchException("This spatial acceleration has to either be expressed in base frame or body frame to use this feature: body frame = "
               + getBodyFrame() + ", base frame = " + getBaseFrame() + ", expressed in frame = " + getReferenceFrame());

      /*
       * A spatial acceleration can be changed from being expressed in the base frame to being expressed
       * in the body frame and vice versa without considering the twist between the two. Indeed when
       * looking at the equations provided in SpatialAccelerationVectorBasics.changeFrame(ReferenceFrame,
       * Twist, Twist), the bias acceleration generated by the velocity becomes zero in such condition.
       * Knowing that, there is no need to enforce a particular frame between the base frame and the body
       * frame, the resulting acceleration will be correct, just with components expressed in a different
       * coordinate system.
       */
      checkExpressedInFrameMatch(bodyFixedPoint.getReferenceFrame());
      checkReferenceFrameMatch(bodyTwist);

      linearAccelerationToPack.setIncludingFrame(bodyTwist.getLinearPart()); // v
      MecanoTools.addCrossToVector(bodyTwist.getAngularPart(), bodyFixedPoint, linearAccelerationToPack); // (w x p) + v
      linearAccelerationToPack.cross(bodyTwist.getAngularPart(), linearAccelerationToPack); // w x ((w x p) + v)
      MecanoTools.addCrossToVector(getAngularPart(), bodyFixedPoint, linearAccelerationToPack); // (wDot x p) + w x ((w x p) + v)
      linearAccelerationToPack.add(getLinearPart()); // vDot + (wDot x p) + w x ((w x p) + v)
   }

   /**
    * Calculates the linear acceleration of the body frame's origin perceived from the base frame.
    * <p>
    * This method is equivalent to calling
    * {@link #getLinearAccelerationAt(TwistReadOnly, FramePoint3DReadOnly, FrameVector3DBasics)} with
    * the {@code bodyfixedPoint} being zero. So effectively, the linear acceleration
    * &alpha;<sub>result</sub> is calculated as follows:
    *
    * <pre>
    * &alpha;<sub>result</sub> = &alpha;<sub>this</sub> + &omega;<sub>twist</sub> &times; &nu;<sub>twist</sub>
    * </pre>
    *
    * where &alpha;<sub>this</sub> represents the linear part of this spatial acceleration,
    * &omega;<sub>twist</sub> and &nu;<sub>twist</sub> represent the angular and linear parts of the
    * body twist.
    * </p>
    *
    * @param bodyTwist                the twist of {@code this.bodyFrame} with respect to
    *                                 {@code this.baseFrame} and expressed in
    *                                 {@code this.expressedInFrame}. Not modified.
    * @param linearAccelerationToPack the vector used to store the result. Modified.
    * @throws ReferenceFrameMismatchException if the {@code bodyTwist} does not have the same reference
    *                                         frames as {@code this}, or if this spatial acceleration
    *                                         is not expressed in the body frame.
    */
   default void getLinearAccelerationAtBodyOrigin(TwistReadOnly bodyTwist, FrameVector3DBasics linearAccelerationToPack)
   {
      getBodyFrame().checkReferenceFrameMatch(getReferenceFrame());
      checkReferenceFrameMatch(bodyTwist);

      linearAccelerationToPack.setIncludingFrame(getLinearPart());
      MecanoTools.addCrossToVector(bodyTwist.getAngularPart(), bodyTwist.getLinearPart(), linearAccelerationToPack);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same spatial acceleration to an
    * {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for spatial accelerations might evolve. In the meantime, the current
    * assumption is that two spatial accelerations are geometrically equal if both their 3D angular and
    * 3D linear accelerations are independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other spatial acceleration to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial accelerations represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   default boolean geometricallyEquals(SpatialAccelerationReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      if (!getAngularPart().geometricallyEquals(other.getAngularPart(), epsilon))
         return false;
      if (!getLinearPart().geometricallyEquals(other.getLinearPart(), epsilon))
         return false;
      return true;
   }
}
