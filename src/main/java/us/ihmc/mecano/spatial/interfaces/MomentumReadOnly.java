package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

/**
 * Read-only interface for a momentum.
 * <p>
 * A momentum is a vector composed of 6 components with an angular part and a linear part.
 * </p>
 * <p>
 * Even though a momentum is not a force, it belongs to the same space, reason why
 * {@code MomentumReadOnly} extends {@code SpatialForceReadOnly}.
 * </p>
 * <p>
 * As for a {@code SpatialForceReadOnly}, the reference frame in which the momentum is expressed
 * also refers to the coordinate at which the momentum is measured.
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
public interface MomentumReadOnly extends SpatialForceReadOnly
{
   /**
    * Calculates the kinetic co-energy as follows:
    * 
    * <pre>
    * E = h &middot; T
    *   = &sum;<sub>i=1:6</sub>(h<sub>i</sub> * T<sub>i</sub>)
    * </pre>
    * 
    * where <tt>h</tt> is the momentum and <tt>T</tt> is the given twist.
    * 
    * @param twist the twist to use for computing the kinetic co-energy. Not modified.
    * @return the value of the kinetic co-energy.
    * @throws ReferenceFrameMismatchException if the given {@code twist} is not expressed in the
    *            same reference frame as {@code this}.
    *            
    */
   default double computeKineticCoEnergy(TwistReadOnly twist)
   {
      twist.getBaseFrame().checkIsAStationaryFrame();
      return 0.5 * dot(twist);
   }
}
