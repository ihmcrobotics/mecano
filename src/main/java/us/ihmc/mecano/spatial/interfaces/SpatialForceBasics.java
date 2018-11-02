package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;

/**
 * Write and read interface for a spatial force vector that is expressed in a changeable reference
 * frame.
 * <p>
 * A spatial force vector is a vector composed of 6 components with an angular part and a linear
 * part. The angular part represents a 3D moment and the linear part a 3D force.
 * </p>
 * <p>
 * When using a {@code SpatialForceVectorBasics}, it is important to note that the reference frame
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
public interface SpatialForceBasics extends FixedFrameSpatialForceBasics, SpatialVectorBasics, FrameChangeable
{

   /**
    * Sets the reference frame of this vector without changing the value of its components.
    */
   @Override
   void setReferenceFrame(ReferenceFrame expressedInFrame);

   /**
    * Sets this spatial force given a 3D moment and 3D force that are exerted at
    * {@code pointOfApplication} and updates this vector reference frame.
    * <p>
    * Effectively, this spatial force is updated as follow:
    *
    * <pre>
    * &tau;<sub>this</sub> = &tau;<sub>new</sub> + P &times; f<sub>new</sub>
    * f<sub>this</sub> = f<sub>new</sub>
    * </pre>
    *
    * where &tau; and f are the angular and linear parts respectively, and P is the
    * {@code pointOfApplication}.
    * </p>
    * <p>
    * When the given {@code angularPart} is {@code null}, it is assumed to be zero.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the arguments are expressed.
    * @param angularPart the 3D moment that is applied. Can be {@code null}. Not modified.
    * @param linearPart the 3D force that is applied. Not modified.
    * @param pointOfApplication the location where the force is exerted. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart,
                                  Point3DReadOnly pointOfApplication)
   {
      setReferenceFrame(expressedInFrame);
      set(angularPart, linearPart, pointOfApplication);
   }

   /**
    * Sets this spatial force given a 3D moment and 3D force that are exerted at
    * {@code pointOfApplication} and updates this vector reference frame.
    * <p>
    * Effectively, this spatial force is updated as follow:
    *
    * <pre>
    * &tau;<sub>this</sub> = &tau;<sub>new</sub> + P &times; f<sub>new</sub>
    * f<sub>this</sub> = f<sub>new</sub>
    * </pre>
    *
    * where &tau; and f are the angular and linear parts respectively, and P is the
    * {@code pointOfApplication}.
    * </p>
    * <p>
    * When the given {@code angularPart} is {@code null}, it is assumed to be zero.
    * </p>
    *
    * @param angularPart the 3D moment that is applied, it is expressed in the new
    *           "expressed-in-frame" to use for this spatial force vector. Can be {@code null}. Not
    *           modified.
    * @param linearPart the 3D force that is applied, it is expressed in the new
    *           "expressed-in-frame" to use for this spatial force vector. Not modified.
    * @param pointOfApplication the location where the force is exerted, it is expressed in the new
    *           "expressed-in-frame" to use for this spatial force vector. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same
    *            reference frame.
    */
   default void setIncludingFrame(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart, FramePoint3DReadOnly pointOfApplication)
   {
      if (angularPart != null)
         linearPart.checkReferenceFrameMatch(angularPart);
      linearPart.checkReferenceFrameMatch(pointOfApplication);
      setIncludingFrame(linearPart.getReferenceFrame(), angularPart, linearPart, pointOfApplication);
   }
}
