package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;

/**
 * Write and read interface for a spatial force vector that is expressed in an unchangeable
 * reference frame.
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
public interface FixedFrameSpatialForceBasics extends SpatialForceReadOnly, FixedFrameSpatialVectorBasics
{
   /**
    * Gets the reference to the angular part of this vector, i.e. the 3D moment.
    * <p>
    * Note that the frame of the angular part is always equal to this vector reference frame.
    * </p>
    *
    * @return the angular part.
    */
   @Override
   FixedFrameVector3DBasics getAngularPart();

   /**
    * Gets the reference to the linear part of this vector, i.e. the 3D force.
    * <p>
    * Note that the frame of the linear part is always equal to this vector reference frame.
    * </p>
    *
    * @return the linear part.
    */
   @Override
   FixedFrameVector3DBasics getLinearPart();

   /**
    * Sets this spatial force given a 3D moment and 3D force that are exerted at
    * {@code pointOfApplication}.
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
    * @param angularPart the 3D moment that is applied. Can be {@code null}. Not modified.
    * @param linearPart the 3D force that is applied. Not modified.
    * @param pointOfApplication the location where the force is exerted. Not modified.
    */
   default void set(Vector3DReadOnly angularPart, Vector3DReadOnly linearPart, Point3DReadOnly pointOfApplication)
   {
      getLinearPart().set(linearPart);

      if (angularPart != null)
         getAngularPart().set(angularPart);
      else
         getAngularPart().setToZero();
      addCrossToAngularPart(pointOfApplication, linearPart);
   }

   /**
    * Sets this spatial force given a 3D moment and 3D force that are exerted at
    * {@code pointOfApplication}.
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
    * @throws ReferenceFrameMismatchException if {@code expressedInFrame} is not equal to the
    *            reference frame in which this spatial force vector is currently expressed.
    */
   default void set(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart, Point3DReadOnly pointOfApplication)
   {
      checkReferenceFrameMatch(expressedInFrame);
      set(angularPart, linearPart, pointOfApplication);
   }

   /**
    * Sets this spatial force given a 3D moment and 3D force that are exerted at
    * {@code pointOfApplication}.
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
    * @param angularPart the 3D moment that is applied. Can be {@code null}. Not modified.
    * @param linearPart the 3D force that is applied. Not modified.
    * @param pointOfApplication the location where the force is exerted. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments are not expressed in the same
    *            reference frame as {@code this}.
    */
   default void set(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart, FramePoint3DReadOnly pointOfApplication)
   {
      if (angularPart != null)
         linearPart.checkReferenceFrameMatch(angularPart);
      linearPart.checkReferenceFrameMatch(pointOfApplication);
      set(linearPart.getReferenceFrame(), angularPart, linearPart, pointOfApplication);
   }

   /**
    * Transform this spatial force using the given transform.
    * <p>
    * Effectively, the new spatial force F<sub>des</sub> = [&tau;<sub>new</sub>; f<sub>new</sub>] is
    * calculated as follows:
    *
    * <pre>
    * &tau;<sub>new</sub> = R &tau; + P &times f<sub>new</sub>
    * f<sub>new</sub> = R f
    * </pre>
    *
    * where:
    * <ul>
    * <li>&tau; and f are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the given transform.
    * </ul>
    * </p>
    * 
    * @throws UnsupportedOperationException if the given transform is not a
    *            {@code RigidBodyTransform}.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransformReadOnly)
         applyTransform((RigidBodyTransformReadOnly) transform);
      else
         throw new UnsupportedOperationException("The feature applyTransform is not supported for the transform of the type: "
               + transform.getClass().getSimpleName());
   }

   /**
    * Transform this spatial force by the inverse of the given transform.
    * <p>
    * Effectively, the new spatial force F<sub>des</sub> = [&tau;<sub>new</sub>; f<sub>new</sub>] is
    * calculated as follows:
    *
    * <pre>
    * &tau;<sub>new</sub> = R<sup>T</sup> ( &tau; - P &times f )
    * f<sub>new</sub> = R<sup>T</sup> f
    * </pre>
    *
    * where:
    * <ul>
    * <li>&tau; and f are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the given transform.
    * </ul>
    * </p>
    * 
    * @throws UnsupportedOperationException if the given transform is not a
    *            {@code RigidBodyTransform}.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransformReadOnly)
         applyInverseTransform((RigidBodyTransformReadOnly) transform);
      else
         throw new UnsupportedOperationException("The feature applyInverseTransform is not supported for the transform of the type: "
               + transform.getClass().getSimpleName());
   }

   /**
    * Transform this spatial force using the given transform.
    * <p>
    * Effectively, the new spatial force F<sub>des</sub> = [&tau;<sub>new</sub>; f<sub>new</sub>] is
    * calculated as follows:
    *
    * <pre>
    * &tau;<sub>new</sub> = R &tau; + P &times f<sub>new</sub>
    * f<sub>new</sub> = R f
    * </pre>
    *
    * where:
    * <ul>
    * <li>&tau; and f are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the given transform.
    * </ul>
    * </p>
    * 
    * @param transform the transform to use on this. Not modified.
    */
   default void applyTransform(RigidBodyTransformReadOnly transform)
   {
      if (transform.hasRotation())
      {
         getAngularPart().applyTransform(transform);
         getLinearPart().applyTransform(transform);
      }

      if (transform.hasTranslation())
         addCrossToAngularPart(transform.getTranslation(), getLinearPart());
   }

   /**
    * Transform this spatial force by the inverse of the given transform.
    * <p>
    * Effectively, the new spatial force F<sub>des</sub> = [&tau;<sub>new</sub>; f<sub>new</sub>] is
    * calculated as follows:
    *
    * <pre>
    * &tau;<sub>new</sub> = R<sup>T</sup> ( &tau; - P &times f )
    * f<sub>new</sub> = R<sup>T</sup> f
    * </pre>
    *
    * where:
    * <ul>
    * <li>&tau; and f are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the given transform.
    * </ul>
    * </p>
    * 
    * @param transform the transform to use on this. Not modified.
    */
   default void applyInverseTransform(RigidBodyTransformReadOnly transform)
   {
      if (transform.hasTranslation())
         addCrossToAngularPart(getLinearPart(), transform.getTranslation());

      if (transform.hasRotation())
      {
         getAngularPart().applyInverseTransform(transform);
         getLinearPart().applyInverseTransform(transform);
      }
   }
}
