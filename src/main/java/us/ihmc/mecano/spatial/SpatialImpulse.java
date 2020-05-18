package us.ihmc.mecano.spatial;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;

/**
 * A {@code SpatialImpulse} is vector composed of 6 components with an angular part and a linear
 * part. An impulse is the integral of a wrench, i.e. force and/or torque, over a time interval.
 * While applying a wrench on a body causes it to accelerate, applying an impulse results in a
 * change of velocity of the body.
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
public class SpatialImpulse implements SpatialImpulseBasics, GeometryObject<SpatialImpulse>
{
   /**
    * The reference of the frame rigidly attached to the body on which this spatial impulse is applied.
    */
   private ReferenceFrame bodyFrame;
   /** This is where we store the internal data. */
   private final SpatialForce spatialForceVector = new SpatialForce();

   /**
    * Creates a new spatial impulse with its components set to zero and its reference frames set to
    * {@code null}.
    */
   public SpatialImpulse()
   {
      setToZero(null, null);
   }

   /**
    * Creates a new spatial impulse with its components set to zero and initializes its reference
    * frames.
    *
    * @param bodyFrame        the frame rigidly attached to the body on which this spatial impulse is
    *                         applied.
    * @param expressedInFrame the frame in which this spatial impulse is expressed.
    */
   public SpatialImpulse(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setToZero(bodyFrame, expressedInFrame);
   }

   /**
    * Creates a new spatial impulse and initializes its components and reference frames.
    *
    * @param bodyFrame        the frame rigidly attached to the body on which this spatial impulse is
    *                         applied.
    * @param expressedInFrame the frame in which this spatial impulse is expressed.
    * @param angularPart      the vector holding the values for the angular part. Not modified.
    * @param linearPart       the vector holding the values for the linear part. Not modified.
    */
   public SpatialImpulse(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, angularPart, linearPart);
   }

   /**
    * Creates a new spatial impulse from the given reference frames and matrix.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the frame rigidly attached to the body on which this spatial impulse is
    *                         applied.
    * @param expressedInFrame the frame in which this spatial impulse is expressed.
    * @param matrix           the column vector containing the values for this vector's components. Not
    *                         modified.
    */
   public SpatialImpulse(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, matrix);
   }

   /**
    * Creates a new spatial impulse from the given reference frames and array.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the frame rigidly attached to the body on which this spatial impulse is
    *                         applied.
    * @param expressedInFrame the frame in which this spatial impulse is expressed.
    * @param array            the array containing the new values for this vector's components. Not
    *                         modified.
    */
   public SpatialImpulse(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, array);
   }

   /**
    * Creates a new spatial impulse from the given {@code bodyFrame} and spatial vector.
    *
    * @param bodyFrame     the frame rigidly attached to the body on which this spatial impulse is
    *                      applied.
    * @param spatialVector the vector used to initialize this spatial impulse value. Not modified.
    */
   public SpatialImpulse(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      setIncludingFrame(bodyFrame, spatialVector);
   }

   /**
    * Copy constructor.
    *
    * @param other the other spatial impulse to copy. Not modified.
    */
   public SpatialImpulse(SpatialImpulseReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(SpatialImpulse other)
   {
      SpatialImpulseBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {
      this.bodyFrame = bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      spatialForceVector.setReferenceFrame(expressedInFrame);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return spatialForceVector.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getAngularPart()
   {
      return spatialForceVector.getAngularPart();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getLinearPart()
   {
      return spatialForceVector.getLinearPart();
   }

   /**
    * See {@link SpatialForce#changeFrame(ReferenceFrame)}.
    */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      spatialForceVector.changeFrame(desiredFrame);
   }

   /**
    * See {@link SpatialForce#applyTransform(Transform)}.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      spatialForceVector.applyTransform(transform);
   }

   /**
    * See {@link SpatialForce#applyInverseTransform(Transform)}.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      spatialForceVector.applyInverseTransform(transform);
   }

   /**
    * Tests on a per component basis if this spatial impulse is equal to the given {@code other} to an
    * {@code epsilon} and both vectors have the same frames.
    *
    * @param other   the other spatial impulse to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(SpatialImpulse other, double epsilon)
   {
      return SpatialImpulseBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same spatial impulse to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for spatial impulses might evolve. In the meantime, the current
    * assumption is that two spatial impulses are geometrically equal if both their angular and linear
    * parts are independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other spatial impulse to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial impulses represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   @Override
   public boolean geometricallyEquals(SpatialImpulse other, double epsilon)
   {
      return SpatialImpulseBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@code this.equals((SpatialImpulseReadOnly) object)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof SpatialImpulseReadOnly)
         return SpatialImpulseBasics.super.equals((SpatialImpulseReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this spatial impulse vector as follows:<br>
    * Spatial impulse exerted on bodyFrame: [angular = (x, y, z), linear = (x, y, z)] -
    * expressedInFrame
    *
    * @return the {@code String} representing this spatial impulse.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getSpatialImpulseString(this);
   }
}
