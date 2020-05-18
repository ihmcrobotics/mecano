package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

/**
 * Write and read interface for a momentum that is expressed in an unchangeable reference frame.
 * <p>
 * A momentum is a vector composed of 6 components with an angular part and a linear part.
 * </p>
 * <p>
 * Even though a momentum is not a force, it belongs to the same space, reason why
 * {@code FixedFrameMomentumBasics} extends {@code FixedFrameSpatialForceBasics}.
 * </p>
 * <p>
 * As for a {@code FixedFrameSpatialForceBasics}, the reference frame in which the momentum is
 * expressed also refers to the coordinate at which the momentum is measured.
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
public interface FixedFrameMomentumBasics extends MomentumReadOnly, FixedFrameSpatialForceBasics
{
   /**
    * Sets this momentum to the product of the given inertia and twist:
    *
    * <pre>
    * h = I * T
    * </pre>
    *
    * @param spatialInertia the inertia to use for computing the momentum. Not modified.
    * @param twist          the twist to use for computing the momentum. Not modified.
    * @throws ReferenceFrameMismatchException if either argument is not expressed in the same reference
    *                                         frame as {@code this}.
    * @throws RuntimeException                if the base frame of the given {@code twist} is not an
    *                                         inertial frame.
    */
   default void compute(SpatialInertiaReadOnly spatialInertia, TwistReadOnly twist)
   {
      spatialInertia.checkReferenceFrameMatch(twist.getReferenceFrame());
      twist.checkExpressedInFrameMatch(getReferenceFrame());

      spatialInertia.getMomentOfInertia().transform(twist.getAngularPart(), getAngularPart());
      getLinearPart().set(twist.getLinearPart());

      if (!spatialInertia.isCenterOfMassOffsetZero())
      {
         double x = getAngularPartX();
         double y = getAngularPartY();
         double z = getAngularPartZ();

         getAngularPart().cross(spatialInertia.getCenterOfMassOffset(), twist.getLinearPart());
         getAngularPart().scale(spatialInertia.getMass());
         getAngularPart().add(x, y, z);

         addCrossToLinearPart(twist.getAngularPart(), spatialInertia.getCenterOfMassOffset());
      }

      getLinearPart().scale(spatialInertia.getMass());
   }
}
