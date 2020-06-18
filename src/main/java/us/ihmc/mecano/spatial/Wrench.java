package us.ihmc.mecano.spatial;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;

/**
 * A {@code Wrench} is a vector composed of 6 components with an angular part and a linear part. The
 * angular part represents a 3D torque and the linear part a 3D force.
 * <p>
 * The concept of a wrench is to describe an external spatial force, i.e. 3D torque and 3D force,
 * that is applied to a body. In this framework, the body on which the force is applied is referred
 * to using a reference frame commonly named {@code bodyFrame}. This reference frame is always
 * assumed to be rigidly attached to the body. As a result, a wrench is nothing more than a spatial
 * force to which a {@code bodyFrame} is associated.
 * </p>
 * <p>
 * When using a {@code Wrench}, it is important to note that the reference frame in which it is
 * expressed does not only refer to the coordinate system in which the angular and linear 3D vectors
 * are expressed. The origin of the reference frame is also used as the point where the spatial
 * force is measured. Let's consider two reference frames A and B which axes are parallel but have
 * different origins, changing the frame of a spatial force vector from A to B will not affect the
 * linear part, i.e. the 3D force, but will still affect the value of the angular part, i.e. the 3D
 * moment. See {@link #changeFrame(ReferenceFrame)} for more information.
 * </p>
 * <p>
 * The convention when using a wrench in matrix operations is that the angular part occupies the 3
 * first rows and the linear part the 3 last as follows:<br>
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
public class Wrench implements WrenchBasics, GeometryObject<Wrench>
{
   /**
    * The reference of the frame rigidly attached to the body on which this wrench is applied.
    */
   private ReferenceFrame bodyFrame;
   /** This is where we store the internal data. */
   private final SpatialForce spatialForceVector = new SpatialForce();

   /**
    * Creates a new wrench with its components set to zero and its reference frames set to
    * {@code null}.
    */
   public Wrench()
   {
      setToZero(null, null);
   }

   /**
    * Creates a new wrench with its components set to zero and initializes its reference frames.
    *
    * @param bodyFrame        the frame rigidly attached to the body on which this wrench is applied.
    * @param expressedInFrame the frame in which this wrench is expressed.
    */
   public Wrench(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setToZero(bodyFrame, expressedInFrame);
   }

   /**
    * Creates a new wrench and initializes its components and reference frames.
    *
    * @param bodyFrame        the frame rigidly attached to the body on which this wrench is applied.
    * @param expressedInFrame the frame in which this wrench is expressed.
    * @param angularPart      the vector holding the values for the angular part. Not modified.
    * @param linearPart       the vector holding the values for the linear part. Not modified.
    */
   public Wrench(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, angularPart, linearPart);
   }

   /**
    * Creates a new wrench from the given reference frames and matrix.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the frame rigidly attached to the body on which this wrench is applied.
    * @param expressedInFrame the frame in which this wrench is expressed.
    * @param matrix           the column vector containing the values for this vector's components. Not
    *                         modified.
    */
   public Wrench(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, DMatrix matrix)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, matrix);
   }

   /**
    * Creates a new wrench from the given reference frames and array.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the frame rigidly attached to the body on which this wrench is applied.
    * @param expressedInFrame the frame in which this wrench is expressed.
    * @param array            the array containing the new values for this vector's components. Not
    *                         modified.
    */
   public Wrench(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, array);
   }

   /**
    * Creates a new wrench from the given {@code bodyFrame} and spatial vector.
    *
    * @param bodyFrame     the frame rigidly attached to the body on which this wrench is applied.
    * @param spatialVector the vector used to initialize this wrench value. Not modified.
    */
   public Wrench(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      setIncludingFrame(bodyFrame, spatialVector);
   }

   /**
    * Copy constructor.
    *
    * @param other the other wrench to copy. Not modified.
    */
   public Wrench(WrenchReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(Wrench other)
   {
      WrenchBasics.super.set(other);
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
    * Tests on a per component basis if this wrench is equal to the given {@code other} to an
    * {@code epsilon} and both vectors have the same frames.
    *
    * @param other   the other wrench to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Wrench other, double epsilon)
   {
      return WrenchBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same wrench to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for wrenches might evolve. In the meantime, the current assumption is
    * that two wrenches are geometrically equal if both their 3D torque and 3D force are independently
    * geometrically equal, see {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other wrench to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two wrenches represent the same physical quantity, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   @Override
   public boolean geometricallyEquals(Wrench other, double epsilon)
   {
      return WrenchBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@code this.equals((WrenchReadOnly) object)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof WrenchReadOnly)
         return WrenchBasics.super.equals((WrenchReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this wrench as follows:<br>
    * Wrench exerted on bodyFrame: [angular = (x, y, z), linear = (x, y, z)] - expressedInFrame
    *
    * @return the {@code String} representing this wrench.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getWrenchString(this);
   }
}
