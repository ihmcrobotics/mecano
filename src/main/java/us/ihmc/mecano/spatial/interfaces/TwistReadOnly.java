package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.Twist;

/**
 * Read-only interface for a twist.
 * <p>
 * A twist is a vector composed of 6 components with an angular part and a linear part. The angular
 * part represents a 3D angular velocity and the linear part a 3D linear velocity.
 * </p>
 * <p>
 * A twist always describes the relative velocity of a body with respect to a base. These two
 * entities are referred to here by using two reference frames: a {@code bodyFrame} that is
 * considered to be rigidly attached to the body, and a {@code baseFrame} that is considered to be
 * rigidly attached to the base.
 * </p>
 * <p>
 * When using a {@code TwistReadOnly}, it is important to note that the reference frame in which it
 * is expressed does not only refer to the coordinate system in which the angular and linear 3D
 * vectors are expressed. The origin of the reference frame is also used as the point where the
 * velocity is measured. While the angular part remains the same as the point of measurement
 * changes, the linear part does depend on its location. As an example for visualizing this effect,
 * imagine yourself standing on a platform quickly rotating: when standing at its center of
 * rotation, you do not feel any wind, but as you walk away from the center, the wind becomes
 * stronger. This highlights that in both situations the velocity of the platform is the same but
 * the linear velocity depends on where you are measuring it. See
 * {@link Twist#changeFrame(ReferenceFrame)} for more information.
 * </p>
 * <p>
 * This framework for representing in an accurate and safe manner spatial velocities, i.e. twists,
 * and spatial accelerations is based on the Ph.D. thesis of Vincent Duindam entitled <i>"Port-Based
 * Modeling and Control for Efficient Bipedal Walking Robots"</i>. Duindam's publications can be
 * found <a href="http://sites.google.com/site/vincentduindam/publications">here</a>. Several
 * references to this work are spread throughout the code.
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
public interface TwistReadOnly extends SpatialMotionReadOnly
{
   /**
    * Calculates the linear velocity resulting from this twist but measured at a given position
    * {@code observerPosition} on the body.
    * <p>
    * Effectively, the resulting linear velocity &nu;<sub>result</sub> is calculated as follows:
    *
    * <pre>
    * &nu;<sub>result</sub> = &nu;<sub>this</sub> - P &times; &omega;<sub>this</sub>
    * </pre>
    *
    * where &omega; and &nu; represent the angular and linear parts respectively, and {@code P} is the
    * {@code observerPosition}.
    * </p>
    *
    * @param observerPosition     the position on the body where the linear velocity is to be
    *                             estimated. Not modified.
    * @param linearVelocityToPack the vector used to store the result. Modified.
    * @throws ReferenceFrameMismatchException if {@code observerPosition} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void getLinearVelocityAt(FramePoint3DReadOnly observerPosition, FrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.setReferenceFrame(getReferenceFrame());
      getLinearVelocityAt(observerPosition, (FixedFrameVector3DBasics) linearVelocityToPack);
   }

   /**
    * Calculates the linear velocity resulting from this twist but measured at a given position
    * {@code observerPosition} on the body.
    * <p>
    * Effectively, the resulting linear velocity &nu;<sub>result</sub> is calculated as follows:
    *
    * <pre>
    * &nu;<sub>result</sub> = &nu;<sub>this</sub> - P &times; &omega;<sub>this</sub>
    * </pre>
    *
    * where &omega; and &nu; represent the angular and linear parts respectively, and {@code P} is the
    * {@code observerPosition}.
    * </p>
    *
    * @param observerPosition     the position on the body where the linear velocity is to be
    *                             estimated. Not modified.
    * @param linearVelocityToPack the vector used to store the result. Modified.
    * @throws ReferenceFrameMismatchException if either {@code observerPosition} or
    *                                         {@code linearVelocityToPack} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void getLinearVelocityAt(FramePoint3DReadOnly observerPosition, FixedFrameVector3DBasics linearVelocityToPack)
   {
      observerPosition.checkReferenceFrameMatch(getReferenceFrame());
      linearVelocityToPack.set(observerPosition);
      linearVelocityToPack.cross((Vector3DReadOnly) getAngularPart(), (Vector3DReadOnly) linearVelocityToPack);
      linearVelocityToPack.add(getLinearPart());
   }

   /**
    * Calculates and returns the value of the dot product of this twist with {@code wrench}.
    * <p>
    * Note that the dot product of a twist and a wrench results in calculating the instantaneous power.
    * </p>
    * <p>
    * For instance, the dot product of a twist T and a wrench W is defined as: <br>
    * T &sdot; W = W &sdot; T = &sum;<sub>i=1:6</sub>(T<sub>i</sub> &times; W<sub>i</sub>)
    * </p>
    *
    * @param wrench the wrench used for the dot product. Not modified.
    * @return the value of the dot product, the value of the instantaneous power resulting from this
    *         twist and the wrench.
    * @throws ReferenceFrameMismatchException if the two vectors are not expressed in the same
    *                                         reference frame.
    * @throws ReferenceFrameMismatchException if the two vectors are not associated with the same body
    *                                         frame.
    */
   default double dot(WrenchReadOnly wrench)
   {
      checkBodyFrameMatch(wrench.getBodyFrame());
      return SpatialMotionReadOnly.super.dot(wrench);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same twist to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for twists might evolve. In the meantime, the current assumption is that
    * two twists are geometrically equal if both their 3D angular and 3D linear velocities are
    * independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other twist to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two twists represent the same physical quantity, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   default boolean geometricallyEquals(TwistReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      if (!getAngularPart().geometricallyEquals(other.getAngularPart(), epsilon))
         return false;
      if (!getLinearPart().geometricallyEquals(other.getLinearPart(), epsilon))
         return false;
      return true;
   }
}
