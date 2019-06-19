package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;

/**
 * Read-only interface for a spatial force vector.
 * <p>
 * A spatial force vector is a vector composed of 6 components with an angular part and a linear
 * part. The angular part represents a 3D torque and the linear part a 3D force.
 * </p>
 * <p>
 * When using a {@code SpatialForceVectorReadOnly}, it is important to note that the reference frame
 * in which it is expressed does not only refer to the coordinate system in which the angular and
 * linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the spatial force is measured. Let's consider two reference frames A and B which axes are
 * parallel but have different origins, changing the frame of a spatial force vector from A to B
 * will not affect the linear part, i.e. the 3D force, but will still affect the value of the
 * angular part, i.e. the 3D moment. See {@link SpatialForce#changeFrame(ReferenceFrame)} for more
 * information.
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
public interface SpatialForceReadOnly extends SpatialVectorReadOnly
{
   /**
    * Tests if {@code this} and {@code other} represent the same spatial force to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for spatial forces might evolve. In the meantime, the current assumption
    * is that two spatial forces are geometrically equal if both their 3D torque and 3D force are
    * independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other vector to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial forces represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   default boolean geometricallyEquals(SpatialVectorReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      if (!getAngularPart().geometricallyEquals(getAngularPart(), epsilon))
         return false;
      if (!getLinearPart().geometricallyEquals(getLinearPart(), epsilon))
         return false;
      return true;
   }
}
