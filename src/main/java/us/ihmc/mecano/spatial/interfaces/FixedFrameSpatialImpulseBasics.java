package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.Wrench;

/**
 * Write and read interface for a spatial impulse which reference frames can not be changed. An
 * impulse is the integral of a wrench, i.e. force and/or torque, over a time interval. While
 * applying a wrench on a body causes it to accelerate, applying an impulse results in a change of
 * velocity of the body.
 * <p>
 * A spatial impulse is a vector composed of 6 components with an angular part and a linear part.
 * </p>
 * <p>
 * As a {@link Wrench}, a spatial impulse is applied to a body. In this framework, the body on which
 * the impulse is applied is referred to using a reference frame commonly named {@code bodyFrame}.
 * This reference frame is always assumed to be rigidly attached to the body.
 * </p>
 * <p>
 * When using a {@code SpatialImpulseBasics}, it is important to note that the reference frame in
 * which it is expressed does not only refer to the coordinate system in which the angular and
 * linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the impulse is measured. Let's consider two reference frames A and B which axes are
 * parallel but have different origins, changing the frame of a spatial impulse from A to B will not
 * affect the linear part but will affect the value of the angular part. See
 * {@link SpatialImpulse#changeFrame(ReferenceFrame)} for more information.
 * </p>
 * <p>
 * The convention when using a spatial impulse in matrix operations is that the angular part
 * occupies the 3 first rows and the linear part the 3 last as follows:<br>
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
 * @author Sylvain Bertrand
 */
public interface FixedFrameSpatialImpulseBasics extends SpatialImpulseReadOnly, FixedFrameSpatialForceBasics
{
   /**
    * Sets this spatial impulse to {@code other}.
    *
    * @param other the other spatial impulse to copy. Not modified.
    * @throws ReferenceFrameMismatchException if any of the reference frames in {@code other} do not
    *                                         match {@code this}.
    */
   default void set(SpatialImpulseReadOnly other)
   {
      set(other.getBodyFrame(), other.getReferenceFrame(), other.getAngularPart(), other.getLinearPart());
   }

   /**
    * Sets this spatial impulse to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(SpatialImpulseReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other spatial impulse to copy. Not modified.
    * @throws ReferenceFrameMismatchException if either the body frame or base frame in {@code other}
    *                                         does not match {@code this}.
    */
   default void setMatchingFrame(SpatialImpulseReadOnly other)
   {
      other.checkBodyFrameMatch(getBodyFrame());
      FixedFrameSpatialForceBasics.super.setMatchingFrame(other);
   }

   /**
    * Sets this spatial impulse to {@code spatialVector}.
    *
    * @param bodyFrame     the body frame associated with the given spatial force.
    * @param spatialVector the spatial vector to copy values from. Not modified.
    * @throws ReferenceFrameMismatchException if any of the reference frames from the arguments do not
    *                                         match the current frames of {@code this}.
    */
   default void set(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      set(bodyFrame, spatialVector.getReferenceFrame(), spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   /**
    * Sets this spatial impulse given an angular part and linear part.
    *
    * @param bodyFrame   the body frame associated with the given spatial force.
    * @param angularPart the vector holding the new values for the angular part. Not modified.
    * @param linearPart  the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if any of the reference frames from the arguments do not
    *                                         match the current frames of {@code this}.
    */
   default void set(ReferenceFrame bodyFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      set(bodyFrame, angularPart.getReferenceFrame(), angularPart, linearPart);
   }

   /**
    * Sets this spatial impulse given an angular part and linear part.
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the spatial force is expressed.
    * @param angularPart      the vector holding the new values for the angular part. Not modified.
    * @param linearPart       the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if any of the reference frames from the arguments do not
    *                                         match the current frames of {@code this}.
    */
   default void set(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      checkReferenceFrameMatch(bodyFrame, expressedInFrame);
      set(angularPart, linearPart);
   }

   /**
    * Sets this spatial impulse given a 3D moment and 3D force that are exerted at
    * {@code pointOfApplication}.
    * <p>
    * Effectively, this spatial impulse is updated as follow:
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
    * @param bodyFrame          the body frame associated with the given spatial force.
    * @param expressedInFrame   the reference frame in which the arguments are expressed.
    * @param angularPart        the 3D moment that is applied. Can be {@code null}. Not modified.
    * @param linearPart         the 3D force that is applied. Not modified.
    * @param pointOfApplication the location where the force is exerted. Not modified.
    * @throws ReferenceFrameMismatchException if {@code expressedInFrame} is not equal to the reference
    *                                         frame in which this spatial force vector is currently
    *                                         expressed.
    */
   default void set(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart,
                    Point3DReadOnly pointOfApplication)
   {
      checkReferenceFrameMatch(bodyFrame, expressedInFrame);
      set(angularPart, linearPart, pointOfApplication);
   }

   /**
    * Adds the given spatial impulse to {@code this} after performing the usual reference frame checks.
    * <p>
    * {@code this += other}
    * </p>
    *
    * @param other the other spatial impulse to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if the reference frame of {@code other} do not match this
    *                                         wrench's reference frames.
    */
   default void add(SpatialImpulseReadOnly other)
   {
      add(other.getBodyFrame(), other);
   }

   /**
    * Adds the given spatial vector to {@code this} providing the expected {@code bodyFrame} for
    * additional safety.
    * <p>
    * {@code this += spatialVector}
    * </p>
    *
    * @param bodyFrame     the body frame associated with the given vector.
    * @param spatialVector the spatial vector to add to {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code bodyFrame != this.getBodyFrame()} or if the
    *                                         given vector is not expressed in the same frame as this
    *                                         wrench.
    */
   default void add(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      add(bodyFrame, spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   /**
    * Adds the given angular and linear parts to {@code this} providing the expected {@code bodyFrame}
    * for additional safety.
    * <p>
    * {@code this.angularPart += angular}<br>
    * {@code this.linearPart += linear}
    * </p>
    *
    * @param bodyFrame the body frame associated with the given spatial force.
    * @param angular   the vector to add to this vector's angular part. Not modified.
    * @param linear    the vector to add to this vector's linear part. Not modified.
    * @throws ReferenceFrameMismatchException if {@code bodyFrame != this.getBodyFrame()} or if either
    *                                         {@code angular} or {@code linear} are not expressed in
    *                                         the same frame as this wrench.
    */
   default void add(ReferenceFrame bodyFrame, FrameVector3DReadOnly angular, FrameVector3DReadOnly linear)
   {
      checkReferenceFrameMatch(bodyFrame, angular.getReferenceFrame());
      checkReferenceFrameMatch(linear);
      add((Vector3DReadOnly) angular, (Vector3DReadOnly) linear);
   }

   /**
    * Subtracts the given spatial impulse to {@code this} after performing the usual reference frame
    * checks.
    * <p>
    * {@code this -= other}
    * </p>
    *
    * @param other the other spatial impulse to subtract to this. Not modified.
    * @throws ReferenceFrameMismatchException if the reference frame of {@code other} do not match this
    *                                         wrench's reference frames.
    */
   default void sub(SpatialImpulseReadOnly other)
   {
      sub(other.getBodyFrame(), other);
   }

   /**
    * Subtracts the given spatial vector to {@code this} providing the expected {@code bodyFrame} for
    * additional safety.
    * <p>
    * {@code this -= spatialVector}
    * </p>
    *
    * @param bodyFrame     the body frame associated with the given vector.
    * @param spatialVector the spatial vector to subtract to {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code bodyFrame != this.getBodyFrame()} or if the
    *                                         given vector is not expressed in the same frame as this
    *                                         wrench.
    */
   default void sub(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      sub(bodyFrame, spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   /**
    * Subtracts the given angular and linear parts to {@code this} providing the expected
    * {@code bodyFrame} for additional safety.
    * <p>
    * {@code this.angularPart -= angular}<br>
    * {@code this.linearPart -= linear}
    * </p>
    *
    * @param bodyFrame the body frame associated with the given spatial force.
    * @param angular   the vector to subtract to this vector's angular part. Not modified.
    * @param linear    the vector to subtract to this vector's linear part. Not modified.
    * @throws ReferenceFrameMismatchException if {@code bodyFrame != this.getBodyFrame()} or if either
    *                                         {@code angular} or {@code linear} are not expressed in
    *                                         the same frame as this wrench.
    */
   default void sub(ReferenceFrame bodyFrame, FrameVector3DReadOnly angular, FrameVector3DReadOnly linear)
   {
      checkReferenceFrameMatch(bodyFrame, angular.getReferenceFrame());
      checkReferenceFrameMatch(linear);
      sub((Vector3DReadOnly) angular, (Vector3DReadOnly) linear);
   }
}
