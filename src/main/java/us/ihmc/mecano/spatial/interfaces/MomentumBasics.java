package us.ihmc.mecano.spatial.interfaces;

/**
 * Write and read interface for a momentum that is expressed in a changeable reference frame.
 * <p>
 * A momentum is a vector composed of 6 components with an angular part and a linear part.
 * </p>
 * <p>
 * Even though a momentum is not a force, it belongs to the same space, reason why
 * {@code MomentumBasics} extends {@code SpatialForceBasics}.
 * </p>
 * <p>
 * As for a {@code SpatialForceBasics}, the reference frame in which the momentum is expressed also
 * refers to the coordinate at which the momentum is measured.
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
public interface MomentumBasics extends FixedFrameMomentumBasics, SpatialForceBasics
{
}
