package us.ihmc.mecano.spatial.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;

/**
 * Write and read interface for a spatial motion vector which reference frames can be changed.
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
public interface SpatialMotionBasics extends FixedFrameSpatialMotionBasics, SpatialVectorBasics
{
   /**
    * Sets the frame attached to the body that this vector describes the motion of.
    * <p>
    * This method does not modify anything but the body frame.
    * </p>
    * <p>
    * When there is no relative motion, i.e. velocity for twists or acceleration for spatial
    * accelerations, between the old and new body frames, this spatial motion vector remains correct.
    * This is a consequence of Duindam, <i>"Port-Based Modeling and Control for Efficient Bipedal
    * Walking Robots"</i>, page 25, lemma 2.8 (a).
    * </p>
    *
    * @param bodyFrame the new body frame.
    */
   void setBodyFrame(ReferenceFrame bodyFrame);

   /**
    * Sets the frame attached to the base which is used as the reference when describing the motion of
    * a body.
    * <p>
    * This method does not modify anything but the base frame.
    * </p>
    * <p>
    * When there is no relative motion, i.e. velocity for twists or acceleration for spatial
    * accelerations, between the old and new base frames, this spatial motion vector remains correct.
    * This is a consequence of Duindam, <i>"Port-Based Modeling and Control for Efficient Bipedal
    * Walking Robots"</i>, page 25, lemma 2.8 (a).
    * </p>
    *
    * @param baseFrame the new base frame.
    */
   void setBaseFrame(ReferenceFrame baseFrame);

   /**
    * Sets all the components of this vector to zero and updates its reference frames.
    *
    * @param bodyFrame        the new body frame.
    * @param baseFrame        the new base frame.
    * @param expressedInFrame the new reference frame in which this motion is expressed.
    */
   default void setToZero(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setToZero(expressedInFrame);
   }

   /**
    * Sets all the components of this vector to {@link Double#NaN} and sets its reference frames.
    *
    * @param bodyFrame        the new body frame.
    * @param baseFrame        the new base frame.
    * @param expressedInFrame the new reference frame in which this motion is expressed.
    */
   default void setToNaN(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setToNaN(expressedInFrame);
   }

   /**
    * Sets this motion vector from a given motion measured at a different position and updates the
    * reference frame in which {@code this} is expressed in.
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
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart, Point3DReadOnly observerPosition)
   {
      setReferenceFrame(expressedInFrame);
      set(angularPart, linearPart, observerPosition);
   }

   /**
    * Sets this motion vector from a given motion measured at a different position and updates the
    * reference frame in which {@code this} is expressed in.
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
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   default void setIncludingFrame(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart, FramePoint3DReadOnly observerPosition)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      angularPart.checkReferenceFrameMatch(observerPosition);
      setIncludingFrame(angularPart.getReferenceFrame(), angularPart, linearPart, observerPosition);
   }

   /**
    * Sets this motion vector to {@code other} including the reference frames.
    *
    * @param other the other motion vector used to update {@code this}. Not modified.
    */
   default void setIncludingFrame(SpatialMotionReadOnly other)
   {
      setIncludingFrame(other.getBodyFrame(), other.getBaseFrame(), other.getReferenceFrame(), other.getAngularPart(), other.getLinearPart());
   }

   /**
    * Sets this motion vector to {@code spatialVector} and updates all its reference frames.
    *
    * @param bodyFrame     what we are specifying the motion of.
    * @param baseFrame     with respect to what we are specifying the motion.
    * @param spatialVector the spatial vector to copy values from. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, SpatialVectorReadOnly spatialVector)
   {
      setIncludingFrame(bodyFrame, baseFrame, spatialVector.getReferenceFrame(), spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   /**
    * Sets this motion vector given an angular part and linear part and updates all its reference
    * frames.
    *
    * @param bodyFrame   what we are specifying the motion of.
    * @param baseFrame   with respect to what we are specifying the motion.
    * @param angularPart the vector holding the new values for the angular part, it is expressed in the
    *                    new "expressed-in-frame" to use for this spatial motion vector. Not modified.
    * @param linearPart  the vector holding the new values for the linear part, it is expressed in the
    *                    new "expressed-in-frame" to use for this spatial motion vector. Not modified.
    * @throws ReferenceFrameMismatchException if the given {@code angularPart} and {@code linearPart}
    *                                         are not expressed in the same reference frame.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      setIncludingFrame(bodyFrame, baseFrame, angularPart.getReferenceFrame(), angularPart, linearPart);
   }

   /**
    * Sets this motion vector given an angular part and linear part and updates all its reference
    * frames.
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param angularPart      the vector holding the new values for the angular part. Not modified.
    * @param linearPart       the vector holding the new values for the linear part. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart,
                                  Vector3DReadOnly linearPart)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setIncludingFrame(expressedInFrame, angularPart, linearPart);
   }

   /**
    * Sets this motion vector's components from the given array {@code array} and updates all its
    * reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param array            the array containing the new values for this motion vector's components.
    *                         Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setIncludingFrame(expressedInFrame, array);
   }

   /**
    * Sets this motion vector's components from the given array {@code array} and updates all its
    * reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param startIndex       the first index to start reading from in the array.
    * @param array            the array containing the new values for this motion vector's components.
    *                         Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, int startIndex, double[] array)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setIncludingFrame(expressedInFrame, startIndex, array);
   }

   /**
    * Sets this motion vector's components from the given array {@code array} and updates all its
    * reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param array            the array containing the new values for this motion vector's components.
    *                         Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, float[] array)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setIncludingFrame(expressedInFrame, array);
   }

   /**
    * Sets this motion vector's components from the given array {@code array} and updates all its
    * reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param startIndex       the first index to start reading from in the array.
    * @param array            the array containing the new values for this motion vector's components.
    *                         Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, int startIndex, float[] array)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setIncludingFrame(expressedInFrame, startIndex, array);
   }

   /**
    * Sets this motion vector's components from the given column vector starting to read from
    * {@code startRow} and updates all its reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param matrix           the column vector containing the new values for this motion vector's
    *                         components. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DMatrix matrix)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setIncludingFrame(expressedInFrame, matrix);
   }

   /**
    * Sets this motion vector's components from the given column vector starting to read from
    * {@code startRow} at the column index {@code column} and updates all its reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param startRow         the first row index to start reading in the dense-matrix.
    * @param matrix           the column vector containing the new values for this motion vector's
    *                         components. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, int startRow, DMatrix matrix)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setIncludingFrame(expressedInFrame, startRow, matrix);
   }

   /**
    * Sets this motion vector's components from the given column vector starting to read from its first
    * row index and updates all its reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the motion of.
    * @param baseFrame        with respect to what we are specifying the motion.
    * @param expressedInFrame in which reference frame the motion is expressed.
    * @param startRow         the first row index to start reading in the dense-matrix.
    * @param column           the column index to read in the dense-matrix.
    * @param matrix           the column vector containing the new values for this motion vector's
    *                         components. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, int startRow, int column,
                                  DMatrix matrix)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setIncludingFrame(expressedInFrame, startRow, column, matrix);
   }

   /**
    * Inverts this spatial motion vector such that after calling this method, this motion vector
    * describes the motion of the base with respect to the body.
    * <p>
    * In other words: given the spatial motion of frame A with respect to frame B, expressed in frame
    * C, this method computes the spatial motion of frame B with respect to frame A, expressed in frame
    * C (or vice versa), by just taking the additive inverse.
    * </p>
    * <p>
    * See Duindam, <i>Port-Based Modeling and Control for Efficient Bipedal Walking Robots</i>, page
    * 25, lemma 2.8 (d) (and (e), for generalizing to any expressedInFrame).
    * </p>
    * <p>
    * While Duindam proves this fact for twists only, it can be easily proved to be accurate for
    * derivatives of twists.
    * </p>
    */
   default void invert()
   {
      negate();
      ReferenceFrame oldBaseFrame = getBaseFrame();
      setBaseFrame(getBodyFrame());
      setBodyFrame(oldBaseFrame);
   }
}
