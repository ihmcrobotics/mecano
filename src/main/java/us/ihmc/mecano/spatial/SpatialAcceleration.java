package us.ihmc.mecano.spatial;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialMotionReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;

/**
 * A {@code SpatialAccelerationVector} is a vector composed of 6 components with an angular part and
 * a linear part. It represents the time derivative of a twist. The angular part represents a 3D
 * angular acceleration and the linear part a 3D linear acceleration.
 * <p>
 * A spatial acceleration always describes the relative velocity of a body with respect to a base.
 * These two entities are referred to here by using two reference frames: a {@code bodyFrame} that
 * is considered to be rigidly attached to the body, and a {@code baseFrame} that is considered to
 * be rigidly attached to the base.
 * </p>
 * <p>
 * When using a {@code SpatialAccelerationVector}, it is important to note that the reference frame
 * in which it is expressed does not only refer to the coordinate system in which the angular and
 * linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the acceleration is measured. While the angular part remains the same as the point of
 * measurement changes, the linear part does depend on its location.
 * </p>
 * <p>
 * This framework for representing in an accurate and safe manner spatial accelerations is based on
 * the Ph.D. thesis of Vincent Duindam entitled <i>"Port-Based Modeling and Control for Efficient
 * Bipedal Walking Robots"</i>. Duindam's publications can be found
 * <a href="http://sites.google.com/site/vincentduindam/publications">here</a>. Several references
 * to this work are spread throughout the code.
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
public class SpatialAcceleration implements SpatialAccelerationBasics, GeometryObject<SpatialAcceleration>
{
   /**
    * Reference frame rigidly attached to the body that this spatial acceleration describes the motion
    * of.
    */
   private ReferenceFrame bodyFrame;
   /**
    * Reference frame rigidly attached to the base that this spatial acceleration uses as reference for
    * quantifying the body's acceleration.
    */
   private ReferenceFrame baseFrame;
   /** This is where we store the internal data. */
   private final SpatialVector spatialVector = new SpatialVector();

   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D observerPosition = new Point3D();
   /** Variable to store intermediate results for garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new spatial acceleration with its components set to zero and its reference frames set
    * to {@code null}.
    */
   public SpatialAcceleration()
   {
      setToZero(null, null, null);
   }

   /**
    * Creates a new spatial acceleration with its components set to zero and initializes its reference
    * frames.
    *
    * @param bodyFrame        what we are specifying the spatial acceleration of.
    * @param baseFrame        with respect to what we are specifying the spatial acceleration.
    * @param expressedInFrame in which reference frame the spatial acceleration is expressed.
    */
   public SpatialAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      setToZero(bodyFrame, baseFrame, expressedInFrame);
   }

   /**
    * Creates a new spatial acceleration and initializes its components and reference frames.
    *
    * @param bodyFrame        what we are specifying the spatial acceleration of.
    * @param baseFrame        with respect to what we are specifying the spatial acceleration.
    * @param expressedInFrame in which reference frame the spatial acceleration is expressed.
    * @param angularPart      the vector holding the values for the angular part. Not modified.
    * @param linearPart       the vector holding the values for the linear part. Not modified.
    */
   public SpatialAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart,
                              Vector3DReadOnly linearPart)
   {
      setIncludingFrame(bodyFrame, baseFrame, expressedInFrame, angularPart, linearPart);
   }

   /**
    * Creates a new spatial acceleration and initializes its components and reference frames.
    *
    * @param bodyFrame   what we are specifying the spatial acceleration of.
    * @param baseFrame   with respect to what we are specifying the spatial acceleration.
    * @param angularPart the vector holding the values for the angular part, it is expressed in the
    *                    "expressed-in-frame" to use for this spatial acceleration vector. Not
    *                    modified.
    * @param linearPart  the vector holding the values for the linear part, it is expressed in the
    *                    "expressed-in-frame" to use for this spatial acceleration vector. Not
    *                    modified.
    */
   public SpatialAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      setIncludingFrame(bodyFrame, baseFrame, angularPart, linearPart);
   }

   /**
    * Creates a new spatial acceleration and initializes its components and reference frames.
    *
    * @param bodyFrame     what we are specifying the spatial acceleration of.
    * @param baseFrame     with respect to what we are specifying the spatial acceleration.
    * @param spatialVector the vector holding the values, it is expressed in the "expressed-in-frame"
    *                      to use for this spatial acceleration vector. Not modified.
    */
   public SpatialAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, SpatialVectorReadOnly spatialVector)
   {
      setIncludingFrame(bodyFrame, baseFrame, spatialVector);
   }

   /**
    * Creates a new spatial acceleration from the given reference frames and matrix.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the spatial acceleration of.
    * @param baseFrame        with respect to what we are specifying the spatial acceleration.
    * @param expressedInFrame in which reference frame the spatial acceleration is expressed.
    * @param matrix           the column vector containing the values for this vector's components. Not
    *                         modified.
    */
   public SpatialAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setIncludingFrame(bodyFrame, baseFrame, expressedInFrame, matrix);
   }

   /**
    * Creates a new spatial acceleration from the given reference frames and array.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        what we are specifying the spatial acceleration of.
    * @param baseFrame        with respect to what we are specifying the spatial acceleration.
    * @param expressedInFrame in which reference frame the spatial acceleration is expressed.
    * @param array            the array containing the new values for this vector's components. Not
    *                         modified.
    */
   public SpatialAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      setIncludingFrame(bodyFrame, baseFrame, expressedInFrame, array);
   }

   /**
    * Copy constructor.
    *
    * @param other the other spatial motion vector to copy. Not modified.
    */
   public SpatialAcceleration(SpatialMotionReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(SpatialAcceleration other)
   {
      SpatialAccelerationBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {
      this.bodyFrame = bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setBaseFrame(ReferenceFrame baseFrame)
   {
      this.baseFrame = baseFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      spatialVector.setReferenceFrame(expressedInFrame);
   }

   /**
    * Transforms this vector such that the result is the same physical spatial acceleration but
    * expressed in a different frame.
    * <p>
    * <b> Use this method only if there is no relative motion between the given desired frame and the
    * frame in which this vector is currently expressed. If there is a relative velocity, use
    * {@link #changeFrame(ReferenceFrame, TwistReadOnly, TwistReadOnly)} </b>
    * </p>
    * <p>
    * Once this spatial acceleration is transformed, the reference frame "expressed-in-frame" is
    * updated to {@code desiredFrame}. In the case, {@code this.expressedInFrame == desiredFrame}, this
    * method does nothing.
    * </p>
    * <p>
    * Note that in addition to transforming the angular and linear parts so their components are
    * expressed in {@code desiredFrame}, the linear part is also affected by the position of the new
    * frame.
    * </p>
    * <p>
    * Effectively, the new spatial acceleration (A<sub>body</sub><sup>des, base</sup> =
    * [&omega;'<sub>new</sub>; &alpha;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;'<sub>new</sub> = R &omega;'
    * &alpha;<sub>new</sub> = R &alpha; + P &times; &omega;'<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega;' and &alpha; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the current "expressed-in-frame" with respect to
    * the desired frame.
    * </ul>
    * </p>
    * <p>
    * These equations can be derived by calculating the time derivative of the identity presented in
    * Duindam, <i>Port-Based Modeling and Control for Efficient Bipedal Walking Robots</i>, page 25,
    * lemma 2.8 (c), see {@link #changeFrame(ReferenceFrame, TwistReadOnly, TwistReadOnly)}.
    * </p>
    *
    * @param desiredFrame the new reference frame in which this spatial acceleration is to be
    *                     expressed.
    */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // trivial case:
      if (getReferenceFrame() == desiredFrame)
      {
         return;
      }

      getReferenceFrame().getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      setReferenceFrame(desiredFrame);
   }

   /**
    * Transform this spatial acceleration using the given transform.
    * <p>
    * Effectively, the new spatial acceleration (A<sub>body</sub><sup>des, base</sup> =
    * [&omega;'<sub>new</sub>; &alpha;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;'<sub>new</sub> = R &omega;'
    * &alpha;<sub>new</sub>  = R &alpha; + P &times; &omega;'<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega;' and &alpha; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation parts of the given transform.
    * </ul>
    * </p>
    */
   @Override
   public void applyTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransform)
      {
         applyTransform((RigidBodyTransform) transform);
      }
      else
      {
         observerPosition.setToZero();
         observerPosition.applyTransform(transform);
         spatialVector.applyTransform(transform);
         addCrossToLinearPart(observerPosition, getAngularPart());
      }
   }

   /**
    * Transform this spatial acceleration by the inverse of the given transform.
    * <p>
    * Effectively, the new spatial acceleration (A<sub>body</sub><sup>des, base</sup> =
    * [&omega;'<sub>new</sub>; &alpha;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;'<sub>new</sub> = R<sup>T</sup> &omega;'
    * &alpha;<sub>new</sub>  = R<sup>T</sup> &alpha; - (R<sup>T</sup> P) &times; &omega;'<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega;' and &alpha; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation parts of the given transform.
    * </ul>
    * </p>
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransform)
      {
         applyInverseTransform((RigidBodyTransform) transform);
      }
      else
      {
         observerPosition.setToZero();
         observerPosition.applyInverseTransform(transform);
         spatialVector.applyInverseTransform(transform);
         addCrossToLinearPart(observerPosition, getAngularPart());
      }
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getBaseFrame()
   {
      return baseFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return spatialVector.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getAngularPart()
   {
      return spatialVector.getAngularPart();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getLinearPart()
   {
      return spatialVector.getLinearPart();
   }

   /**
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon} and both vectors have the same frames.
    *
    * @param other   the other motion vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(SpatialAcceleration other, double epsilon)
   {
      return SpatialAccelerationBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same spatial acceleration to an
    * {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for spatial accelerations might evolve. In the meantime, the current
    * assumption is that two spatial accelerations are geometrically equal if both their 3D angular and
    * 3D linear accelerations are independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other spatial acceleration to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial accelerations represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   @Override
   public boolean geometricallyEquals(SpatialAcceleration other, double epsilon)
   {
      return SpatialAccelerationBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@code this.equals((SpatialAccelerationReadOnly) object)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof SpatialAccelerationReadOnly)
         return SpatialAccelerationBasics.super.equals((SpatialAccelerationReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this spatial acceleration vector as follows:<br>
    * Spatial acceleration of bodyFrame, with respect to baseFrame: [angular = (x, y, z), linear = (x,
    * y, z)] - expressedInFrame
    *
    * @return the {@code String} representing this spatial acceleration vector.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getSpatialAccelerationString(this);
   }
}
