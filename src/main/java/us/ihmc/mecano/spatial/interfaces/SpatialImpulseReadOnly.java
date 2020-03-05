package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.Wrench;

/**
 * Read-only interface for a impulse. An impulse is the integral of a wrench, i.e. force and/or
 * torque, over a time interval. While applying a wrench on a body causes it to accelerate, applying
 * an impulse results in a change of velocity of the body.
 * <p>
 * A spatial impulse is a vector composed of 6 components with an angular part and a linear part.
 * </p>
 * <p>
 * As a {@link Wrench}, a spatial impulse is applied to a body. In this framework, the body on which
 * the impulse is applied is referred to using a reference frame commonly named {@code bodyFrame}.
 * This reference frame is always assumed to be rigidly attached to the body.
 * </p>
 * <p>
 * When using a {@code SpatialImpulseReadOnly}, it is important to note that the reference frame in
 * which it is expressed does not only refer to the coordinate system in which the angular and
 * linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the impulse is measured. Let's consider two reference frames A and B which axes are
 * parallel but have different origins, changing the frame of a spatial impulse from A to B will not
 * affect the linear part but will affect the value of the angular part. See
 * {@link SpatialImpulse#changeFrame(ReferenceFrame)} for more information.
 * </p>
 * <p>
 * The convention when using a spatial impulse in matrix operations is that the angular part
 * occupies the 3 first rows and the linear part the 3 last as follows:<br>
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
 * @author Sylvain Bertrand
 */
public interface SpatialImpulseReadOnly extends SpatialForceReadOnly
{
   /**
    * Gets the reference frame that is rigidly attached to the body on which this spatial impulse is
    * applied.
    *
    * @return the body frame.
    */
   ReferenceFrame getBodyFrame();

   /**
    * Checks that all frames, i.e. the body frame and the "expressed in frame", are identical between
    * the two spatial impulses.
    *
    * @param other the other spatial impulse holding onto the reference frames to compare against the
    *              reference frames held by {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if the reference frames are not the same:
    *                                         {@code this.getBodyFrame() != other.getBodyFrame()} or
    *                                         {@code this.getReferenceFrame() != other.getReferenceFrame()}.
    */
   default void checkReferenceFrameMatch(SpatialImpulseReadOnly other) throws ReferenceFrameMismatchException
   {
      checkReferenceFrameMatch(other.getBodyFrame(), other.getReferenceFrame());
   }

   /**
    * Checks that the reference frames used by {@code this} correspond to the given ones.
    *
    * @param bodyFrame        the query for the body frame.
    * @param expressedInFrame the query for the "expressed in frame".
    * @throws ReferenceFrameMismatchException if the reference frames are not the same:
    *                                         {@code this.getBodyFrame() != bodyFrame} or
    *                                         {@code this.getReferenceFrame() != expressedInFrame}.
    */
   default void checkReferenceFrameMatch(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame) throws ReferenceFrameMismatchException
   {
      checkBodyFrameMatch(bodyFrame);
      checkExpressedInFrameMatch(expressedInFrame);
   }

   /**
    * Checks if the body frame held by {@code this} matches the query {@code bodyFrame}.
    *
    * @param bodyFrame the query to compare against the body frame held by {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if the two reference frames are not the same:
    *                                         {@code this.getBodyFrame() != bodyFrame}.
    */
   default void checkBodyFrameMatch(ReferenceFrame bodyFrame)
   {
      if (getBodyFrame() != bodyFrame)
         throw new ReferenceFrameMismatchException("bodyFrame mismatch: this.bodyFrame = " + getBodyFrame() + ", other bodyFrame = " + bodyFrame);
   }

   /**
    * Checks if the "expressed in frame" held by {@code this} matches the query
    * {@code expressedInFrame}.
    *
    * @param expressedInFrame the query to compare against the "expressed in frame" held by
    *                         {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if the two reference frames are not the same:
    *                                         {@code this.getReferenceFrame() != expressedInFrame}.
    */
   default void checkExpressedInFrameMatch(ReferenceFrame expressedInFrame)
   {
      if (getReferenceFrame() != expressedInFrame)
         throw new ReferenceFrameMismatchException("expressedInFrame mismatch: this.expressedInFrame = " + getReferenceFrame() + ", other expressedInFrame = "
               + expressedInFrame);
   }

   /**
    * Tests on a per component basis if this spatial impulse is equal to the given {@code other} to an
    * {@code epsilon} and both spatial impulses have the same frames.
    *
    * @param other   the other spatial impulse to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two spatial impulses are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(SpatialImpulseReadOnly other, double epsilon)
   {
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      return SpatialForceReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same spatial impulse to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for spatial impulse might evolve. In the meantime, the current
    * assumption is that two spatial impulses are geometrically equal if both their 3D angular and
    * angular parts are independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other spatial impulse to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial impulses represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   default boolean geometricallyEquals(SpatialImpulseReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      if (!getAngularPart().geometricallyEquals(getAngularPart(), epsilon))
         return false;
      if (!getLinearPart().geometricallyEquals(getLinearPart(), epsilon))
         return false;
      return true;
   }

   /**
    * Tests on a per component basis, if this spatial impulse is exactly equal to {@code other} and
    * with the same reference frames.
    *
    * @param other the other spatial impulse to compare against this. Not modified.
    * @return {@code true} if the two spatial impulses are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(SpatialImpulseReadOnly other)
   {
      if (other == null)
         return false;
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      return SpatialForceReadOnly.super.equals(other);
   }
}
