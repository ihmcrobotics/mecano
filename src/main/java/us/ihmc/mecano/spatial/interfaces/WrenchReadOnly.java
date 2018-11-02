package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.Wrench;

/**
 * Read-only interface for a wrench.
 * <p>
 * A wrench is a vector composed of 6 components with an angular part and a linear part. The angular
 * part represents a 3D torque and the linear part a 3D force.
 * </p>
 * <p>
 * The concept of a wrench is to describe an external spatial force, i.e. 3D torque and 3D force,
 * that is applied to a body. In this framework, the body on which the force is applied is referred
 * to using a reference frame commonly named {@code bodyFrame}. This reference frame is always
 * assumed to be rigidly attached to the body. As a result, a wrench is nothing more than a spatial
 * force to which a {@code bodyFrame} is associated.
 * </p>
 * <p>
 * When using a {@code WrenchReadOnly}, it is important to note that the reference frame in which it
 * is expressed does not only refer to the coordinate system in which the angular and linear 3D
 * vectors are expressed. The origin of the reference frame is also used as the point where the
 * spatial force is measured. Let's consider two reference frames A and B which axes are parallel
 * but have different origins, changing the frame of a spatial force vector from A to B will not
 * affect the linear part, i.e. the 3D force, but will still affect the value of the angular part,
 * i.e. the 3D moment. See {@link Wrench#changeFrame(ReferenceFrame)} for more information.
 * </p>
 * <p>
 * The convention when using a wrench in matrix operations is that the angular part occupies the 3
 * first rows and the linear part the 3 last as follows:<br>
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
public interface WrenchReadOnly extends SpatialForceReadOnly
{
   /**
    * Gets the reference frame that is rigidly attached to the body on which this wrench is applied.
    *
    * @return the body frame.
    */
   ReferenceFrame getBodyFrame();

   /**
    * Calculates and returns the value of the dot product of this wrench with {@code twist}.
    * <p>
    * Note that the dot product of a wrench and a twist results in calculating the instantaneous
    * power.
    * </p>
    * <p>
    * For instance, the dot product of a wrench W and a twist T is defined as: <br>
    * W &sdot; T = T &sdot; W = &sum;<sub>i=1:6</sub>(W<sub>i</sub> &times; T<sub>i</sub>)
    * </p>
    *
    * @param twist the twist used for the dot product. Not modified.
    * @return the value of the dot product, the value of the instantaneous power resulting from this
    *         wrench and the twist.
    * @throws ReferenceFrameMismatchException if the two vectors are not expressed in the same
    *            reference frame.
    * @throws ReferenceFrameMismatchException if the two vectors are not associated with the same
    *            body frame.
    */
   default double dot(TwistReadOnly twist)
   {
      return twist.dot(this);
   }

   /**
    * Checks that all frames, i.e. the body frame and the "expressed in frame", are identical
    * between the two wrenches.
    *
    * @param other the other wrench holding onto the reference frames to compare against the
    *           reference frames held by {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if the reference frames are not the same:
    *            {@code this.getBodyFrame() != other.getBodyFrame()} or
    *            {@code this.getReferenceFrame() != other.getReferenceFrame()}.
    */
   default void checkReferenceFrameMatch(WrenchReadOnly other) throws ReferenceFrameMismatchException
   {
      checkReferenceFrameMatch(other.getBodyFrame(), other.getReferenceFrame());
   }

   /**
    * Checks that the reference frames used by {@code this} correspond to the given ones.
    *
    * @param bodyFrame the query for the body frame.
    * @param expressedInFrame the query for the "expressed in frame".
    * @throws ReferenceFrameMismatchException if the reference frames are not the same:
    *            {@code this.getBodyFrame() != bodyFrame} or
    *            {@code this.getReferenceFrame() != expressedInFrame}.
    */
   default void checkReferenceFrameMatch(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame) throws ReferenceFrameMismatchException
   {
      checkBodyFrameMatch(bodyFrame);
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
    * @param other the other wrench to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(WrenchReadOnly other, double epsilon)
   {
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      return SpatialForceReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same wrench to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the
    * definition of "geometrically-equal" for wrenches might evolve. In the meantime, the current
    * assumption is that two wrenches are geometrically equal if both their 3D torque and 3D force
    * are independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other wrench to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two wrenches represent the same physical quantity, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *            respectively match the reference frames of {@code this}.
    */
   default boolean geometricallyEquals(WrenchReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      if (!getAngularPart().geometricallyEquals(getAngularPart(), epsilon))
         return false;
      if (!getLinearPart().geometricallyEquals(getLinearPart(), epsilon))
         return false;
      return true;
   }

   /**
    * Tests on a per component basis, if this wrench is exactly equal to {@code other} and with the
    * same reference frames.
    *
    * @param other the other wrench to compare against this. Not modified.
    * @return {@code true} if the two wrenches are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(WrenchReadOnly other)
   {
      if (other == null)
         return false;
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      return SpatialForceReadOnly.super.equals(other);
   }
}
