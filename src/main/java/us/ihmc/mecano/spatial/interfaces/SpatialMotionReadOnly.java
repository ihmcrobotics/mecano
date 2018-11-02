package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;

/**
 * Read-only interface for a spatial motion vector.
 * <p>
 * A spatial motion vector is a vector composed of 6 components with an angular part and a linear
 * part. The notion of a spatial motion vector is abstract and is only meant to group similarities
 * between velocities and accelerations.
 * </p>
 * <p>
 * The idea of motion always describes the relative movement of a body with respect to a base. These
 * two entities are referred to here by using two additional reference frames: a {@code bodyFrame}
 * that is considered to be rigidly attached to the body, and a {@code baseFrame} that is considered
 * to be rigidly attached to the base.
 * </p>
 * <p>
 * When using a {@code SpatialMotionVectorReadOnly}, it is important to note that the reference
 * frame in which it is expressed does not only refer to the coordinate system in which the angular
 * and linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the spatial motion is measured. While the angular part remains the same as the point of
 * measurement changes, the linear part does depend on its location. As an example for visualizing
 * this effect, imagine yourself standing on a platform quickly rotating: when standing at its
 * center of rotation, you do not feel any wind, but as you walk away from the center, the wind
 * becomes stronger. This highlights that in both situations the velocity of the platform is the
 * same but the linear velocity depends on where you are measuring it. See
 * {@link Twist#changeFrame(ReferenceFrame)} or
 * {@link SpatialAcceleration#changeFrame(ReferenceFrame)} for more information.
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
public interface SpatialMotionReadOnly extends SpatialVectorReadOnly
{
   /**
    * Gets the reference frame that is rigidly attached to the body this vector is describing the
    * motion of.
    *
    * @return the body frame.
    */
   ReferenceFrame getBodyFrame();

   /**
    * Gets the reference frame that is rigidly attached to the base.
    * <p>
    * The base is used as reference for describing the motion of the body.
    * </p>
    *
    * @return the base frame.
    */
   ReferenceFrame getBaseFrame();

   /**
    * Checks that all frames, i.e. the body frame, base frame, and the "expressed in frame", are
    * identical between the two spatial motion vectors.
    *
    * @param other the other object holding onto the reference frames to compare against the
    *           reference frames held by {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if the reference frames are not the same:
    *            {@code this.getBodyFrame() != other.getBodyFrame()},
    *            {@code this.getBaseFrame() != other.getBaseFrame()}, or
    *            {@code this.getReferenceFrame() != other.getReferenceFrame()}.
    */
   default void checkReferenceFrameMatch(SpatialMotionReadOnly other) throws ReferenceFrameMismatchException
   {
      checkReferenceFrameMatch(other.getBodyFrame(), other.getBaseFrame(), other.getReferenceFrame());
   }

   /**
    * Checks that the reference frames used by {@code this} correspond to the given ones.
    *
    * @param bodyFrame the query for the body frame.
    * @param baseFrame the query for the base frame.
    * @param expressedInFrame the query for the "expressed in frame".
    * @throws ReferenceFrameMismatchException if the reference frames are not the same:
    *            {@code this.getBodyFrame() != bodyFrame}, {@code this.getBaseFrame() != baseFrame},
    *            or {@code this.getReferenceFrame() != expressedInFrame}.
    */
   default void checkReferenceFrameMatch(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
         throws ReferenceFrameMismatchException
   {
      checkBodyFrameMatch(bodyFrame);
      checkBaseFrameMatch(baseFrame);
      checkExpressedInFrameMatch(expressedInFrame);
   }

   /**
    * Checks if the body frame held by {@code this} matches the query {@code bodyFrame}.
    *
    * @param bodyFrame the query to compare against the body frame held by {@code this}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if the two reference frames are not the same:
    *            {@code this.getBodyFrame() != bodyFrame}.
    */
   default void checkBodyFrameMatch(ReferenceFrame bodyFrame)
   {
      if (getBodyFrame() != bodyFrame)
         throw new ReferenceFrameMismatchException("bodyFrame mismatch: this.bodyFrame = " + getBodyFrame() + ", other bodyFrame = " + bodyFrame);
   }

   /**
    * Checks if the base frame held by {@code this} matches the query {@code baseFrame}.
    *
    * @param baseFrame the query to compare against the base frame held by {@code this}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if the two reference frames are not the same:
    *            {@code this.getBaseFrame() != baseFrame}.
    */
   default void checkBaseFrameMatch(ReferenceFrame baseFrame)
   {
      if (getBaseFrame() != baseFrame)
         throw new ReferenceFrameMismatchException("baseFrame mismatch: this.baseFrame = " + getBaseFrame() + ", other baseFrame = " + baseFrame);
   }

   /**
    * Checks if the "expressed in frame" held by {@code this} matches the query
    * {@code expressedInFrame}.
    *
    * @param expressedInFrame the query to compare against the "expressed in frame" held by
    *           {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if the two reference frames are not the same:
    *            {@code this.getReferenceFrame() != expressedInFrame}.
    */
   default void checkExpressedInFrameMatch(ReferenceFrame expressedInFrame)
   {
      if (getReferenceFrame() != expressedInFrame)
         throw new ReferenceFrameMismatchException("expressedInFrame mismatch: this.expressedInFrame = " + getReferenceFrame() + ", other expressedInFrame = "
               + expressedInFrame);
   }

   /**
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon} and both vectors have the same frames.
    *
    * @param other the other motion vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(SpatialMotionReadOnly other, double epsilon)
   {
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      if (getBaseFrame() != other.getBaseFrame())
         return false;
      return SpatialVectorReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this vector is exactly equal to {@code other} and have the
    * same reference frames.
    *
    * @param other the other vector to compare against this. Not modified.
    * @return {@code true} if the two vectors are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(SpatialMotionReadOnly other)
   {
      if (other == null)
         return false;
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      if (getBaseFrame() != other.getBaseFrame())
         return false;
      return SpatialVectorReadOnly.super.equals(other);
   }
}
