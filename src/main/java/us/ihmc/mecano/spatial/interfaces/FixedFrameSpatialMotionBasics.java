package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;

/**
 * Write and read interface for a spatial motion vector which reference frames can not be changed.
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
 * When using a {@code SpatialMotionVectorBasics}, it is important to note that the reference frame
 * in which it is expressed does not only refer to the coordinate system in which the angular and
 * linear 3D vectors are expressed. The origin of the reference frame is also used as the point
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
public interface FixedFrameSpatialMotionBasics extends SpatialMotionReadOnly, FixedFrameSpatialVectorBasics
{
   /**
    * Sets this motion vector to {@code other}.
    *
    * @param other the other vector to copy. Not modified.
    * @throws ReferenceFrameMismatchException if any of the reference frames in {@code other} do not
    *                                         match {@code this}.
    */
   default void set(SpatialMotionReadOnly other)
   {
      set(other.getBodyFrame(), other.getBaseFrame(), other.getReferenceFrame(), other.getAngularPart(), other.getLinearPart());
   }

   /**
    * Sets this motion vector to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(SpatialMotionReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other vector to copy. Not modified.
    * @throws ReferenceFrameMismatchException if either the body frame or base frame in {@code other}
    *                                         does not match {@code this}.
    */
   default void setMatchingFrame(SpatialMotionReadOnly other)
   {
      other.checkBodyFrameMatch(getBodyFrame());
      other.checkBaseFrameMatch(getBaseFrame());
      FixedFrameSpatialVectorBasics.super.setMatchingFrame(other);
   }

   /**
    * Sets this motion vector to {@code spatialVector}.
    *
    * @param bodyFrame     what we are specifying the motion of.
    * @param baseFrame     with respect to what we are specifying the motion.
    * @param spatialVector the spatial vector to copy values from. Not modified.
    * @throws ReferenceFrameMismatchException if any of the reference frames from the arguments do not
    *                                         match the current frames of {@code this}.
    */
   default void set(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, SpatialVectorReadOnly spatialVector)
   {
      set(bodyFrame, baseFrame, spatialVector.getReferenceFrame(), spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   /**
    * Sets this motion vector given an angular part and linear part.
    *
    * @param bodyFrame   what we are specifying the motion of.
    * @param baseFrame   with respect to what we are specifying the motion.
    * @param angularPart the vector holding the new values for the angular part. Not modified.
    * @param linearPart  the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if any of the reference frames from the arguments do not
    *                                         match the current frames of {@code this}.
    */
   default void set(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      set(bodyFrame, baseFrame, angularPart.getReferenceFrame(), angularPart, linearPart);
   }

   /**
    * Sets this motion vector given an angular part and linear part.
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param angularPart      the vector holding the new values for the angular part. Not modified.
    * @param linearPart       the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if any of the reference frames from the arguments do not
    *                                         match the current frames of {@code this}.
    */
   default void set(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart,
                    Vector3DReadOnly linearPart)
   {
      checkReferenceFrameMatch(bodyFrame, baseFrame, expressedInFrame);
      set(angularPart, linearPart);
   }

   /**
    * Sets this motion vector from a given motion measured at a different position.
    * <p>
    * Effectively, this motion is updated as follow:
    *
    * <pre>
    * &omega;<sub>this</sub> = &omega;<sub>new</sub>
    * &nu;<sub>this</sub> = &nu;<sub>new</sub> + P &times; &omega;<sub>new</sub>
    * </pre>
    *
    * where &omega; and &nu; represent the angular and linear parts respectively, and {@code P} is the
    * {@code observerPosition}.
    * </p>
    *
    * @param angularPart      the angular part of the motion. Not modified.
    * @param linearPart       the linear part of the motion measured at the observer position. Not
    *                         modified.
    * @param observerPosition the location at which the motion is measured. Not modified.
    */
   default void set(Vector3DReadOnly angularPart, Vector3DReadOnly linearPart, Point3DReadOnly observerPosition)
   {
      getAngularPart().set(angularPart);
      double linearPartX = linearPart.getX();
      double linearPartY = linearPart.getY();
      double linearPartZ = linearPart.getZ();
      getLinearPart().cross(observerPosition, angularPart);
      getLinearPart().add(linearPartX, linearPartY, linearPartZ);
   }

   /**
    * Sets this motion vector from a given motion measured at a different position.
    * <p>
    * Effectively, this motion is updated as follow:
    *
    * <pre>
    * &omega;<sub>this</sub> = &omega;<sub>new</sub>
    * &nu;<sub>this</sub> = &nu;<sub>new</sub> + P &times; &omega;<sub>new</sub>
    * </pre>
    *
    * where &omega; and &nu; represent the angular and linear parts respectively, and {@code P} is the
    * {@code observerPosition}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the given motion is expressed.
    * @param angularPart      the angular part of the motion. Not modified.
    * @param linearPart       the linear part of the motion measured at the observer position. Not
    *                         modified.
    * @param observerPosition the location at which the motion is measured. Not modified.
    * @throws ReferenceFrameMismatchException if {@code expressedInFrame} is not the same as
    *                                         {@code this.getReferenceFrame()}.
    */
   default void set(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart, Point3DReadOnly observerPosition)
   {
      checkExpressedInFrameMatch(expressedInFrame);
      set(angularPart, linearPart, observerPosition);
   }

   /**
    * Sets this motion vector from a given motion measured at a different position.
    * <p>
    * Effectively, this motion is updated as follow:
    *
    * <pre>
    * &omega;<sub>this</sub> = &omega;<sub>new</sub>
    * &nu;<sub>this</sub> = &nu;<sub>new</sub> + P &times; &omega;<sub>new</sub>
    * </pre>
    *
    * where &omega; and &nu; represent the angular and linear parts respectively, and {@code P} is the
    * {@code observerPosition}.
    * </p>
    *
    * @param angularPart      the angular part of the motion. Not modified.
    * @param linearPart       the linear part of the motion measured at the observer position. Not
    *                         modified.
    * @param observerPosition the location at which the motion is measured. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in
    *                                         {@code this.getReferenceFrame()}.
    */
   default void set(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart, FramePoint3DReadOnly observerPosition)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      angularPart.checkReferenceFrameMatch(observerPosition);
      set(angularPart.getReferenceFrame(), angularPart, linearPart, observerPosition);
   }

   /**
    * Transform this twist using the given transform. Effectively, the new twist
    * (T<sub>body</sub><sup>des, base</sup> = [&omega;<sub>new</sub>; &nu;<sub>new</sub>]) is
    * calculated as follows:
    *
    * <pre>
    * &omega;<sub>new</sub> = R &omega;
    * &nu;<sub>new</sub> = R &nu; + P &times; &omega;<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega; and &nu; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation parts of the given transform.
    * </ul>
    * </p>
    *
    * @throws UnsupportedOperationException if the given transform is not a {@code RigidBodyTransform}.
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
    * Transform this twist by the inverse of the given transform.
    * <p>
    * Effectively, the new twist (T<sub>body</sub><sup>des, base</sup> = [&omega;<sub>new</sub>;
    * &nu;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;<sub>new</sub> = R<sup>T</sup> &omega;
    * &nu;<sub>new</sub> = R<sup>T</sup> ( &nu; - P &times; &omega; )
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega; and &nu; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation parts of the given transform.
    * </ul>
    * </p>
    *
    * @throws UnsupportedOperationException if the given transform is not a {@code RigidBodyTransform}.
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
    * Transform this twist using the given transform. Effectively, the new twist
    * (T<sub>body</sub><sup>des, base</sup> = [&omega;<sub>new</sub>; &nu;<sub>new</sub>]) is
    * calculated as follows:
    *
    * <pre>
    * &omega;<sub>new</sub> = R &omega;
    * &nu;<sub>new</sub> = R &nu; + P &times; &omega;<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega; and &nu; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation parts of the given transform.
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
         addCrossToLinearPart(transform.getTranslation(), getAngularPart());
   }

   /**
    * Transform this twist by the inverse of the given transform.
    * <p>
    * Effectively, the new twist (T<sub>body</sub><sup>des, base</sup> = [&omega;<sub>new</sub>;
    * &nu;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;<sub>new</sub> = R<sup>T</sup> &omega;
    * &nu;<sub>new</sub> = R<sup>T</sup> ( &nu; - P &times; &omega; )
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega; and &nu; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation parts of the given transform.
    * </ul>
    * </p>
    *
    * @param transform the transform to use on this. Not modified.
    */
   default void applyInverseTransform(RigidBodyTransformReadOnly transform)
   {
      if (transform.hasTranslation())
         addCrossToLinearPart(getAngularPart(), transform.getTranslation());

      if (transform.hasRotation())
      {
         getAngularPart().applyInverseTransform(transform);
         getLinearPart().applyInverseTransform(transform);
      }
   }
}
