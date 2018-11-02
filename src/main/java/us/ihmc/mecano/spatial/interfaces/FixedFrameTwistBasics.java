package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Twist;

/**
 * Write and read interface for a twist which reference frames can not be changed.
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
 * When using a {@code TwistBasics}, it is important to note that the reference frame in which it is
 * expressed does not only refer to the coordinate system in which the angular and linear 3D vectors
 * are expressed. The origin of the reference frame is also used as the point where the velocity is
 * measured. While the angular part remains the same as the point of measurement changes, the linear
 * part does depend on its location. As an example for visualizing this effect, imagine yourself
 * standing on a platform quickly rotating: when standing at its center of rotation, you do not feel
 * any wind, but as you walk away from the center, the wind becomes stronger. This highlights that
 * in both situations the velocity of the platform is the same but the linear velocity depends on
 * where you are measuring it. See {@link Twist#changeFrame(ReferenceFrame)} for more information.
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
public interface FixedFrameTwistBasics extends TwistReadOnly, FixedFrameSpatialMotionBasics
{
}
