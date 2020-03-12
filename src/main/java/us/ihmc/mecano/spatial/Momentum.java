package us.ihmc.mecano.spatial;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.MomentumBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;

/**
 * A {@code Momentum} is a vector composed of 6 components with an angular part and a linear part.
 * <p>
 * Even though a momentum is not a force, it belongs to the same space, reason why {@code Momentum}
 * implements {@code SpatialForceBasics}.
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
public class Momentum implements MomentumBasics, GeometryObject<Momentum>
{
   /** This is where we store the internal data. */
   private final SpatialForce spatialForceVector = new SpatialForce();

   /**
    * Creates a new momentum with its components set to zero and its reference frame set to
    * {@code null}.
    */
   public Momentum()
   {
      setToZero(null);
   }

   /**
    * Creates a new momentum vector with its components set to zero and initializes its reference
    * frame.
    *
    * @param expressedInFrame the initial frame in which this vector is expressed.
    */
   public Momentum(ReferenceFrame expressedInFrame)
   {
      setToZero(expressedInFrame);
   }

   /**
    * Creates a new momentum vector and initializes its components and the reference frame it is
    * expressed in.
    *
    * @param expressedInFrame the initial frame in which this vector is expressed.
    * @param angularPart      the vector holding the values for the angular part. Not modified.
    * @param linearPart       the vector holding the values for the linear part. Not modified.
    */
   public Momentum(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setIncludingFrame(expressedInFrame, linearPart, angularPart);
   }

   /**
    * Creates a new momentum vector from the given reference frame and matrix.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param matrix           the column vector containing the values for this vector's components. Not
    *                         modified.
    */
   public Momentum(ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setIncludingFrame(expressedInFrame, matrix);
   }

   /**
    * Creates a new momentum vector from the given reference frame and array.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param array            the array containing the new values for this vector's components. Not
    *                         modified.
    */
   public Momentum(ReferenceFrame expressedInFrame, double[] array)
   {
      setIncludingFrame(expressedInFrame, array);
   }

   /**
    * Copy constructor.
    *
    * @param other the other momentum vector to copy. Not modified.
    */
   public Momentum(MomentumReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(Momentum other)
   {
      MomentumBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      spatialForceVector.setReferenceFrame(expressedInFrame);
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

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      spatialForceVector.applyTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      spatialForceVector.applyInverseTransform(transform);
   }

   /**
    * @see SpatialForce#changeFrame(ReferenceFrame)
    */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      spatialForceVector.changeFrame(desiredFrame);
   }

   /**
    * @see SpatialForce#epsilonEquals(SpatialForce, double)
    */
   @Override
   public boolean epsilonEquals(Momentum other, double epsilon)
   {
      return spatialForceVector.epsilonEquals(other, epsilon);
   }

   /**
    * @see SpatialForce#geometricallyEquals(SpatialForce, double)
    */
   @Override
   public boolean geometricallyEquals(Momentum other, double epsilon)
   {
      return spatialForceVector.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@code this.equals((MomentumReadOnly) object)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof MomentumReadOnly)
         return equals((MomentumReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this momentum vector as follows:<br>
    * Momentum: [angular = (x, y, z), linear = (x, y, z)] - expressedInFrame
    *
    * @return the {@code String} representing this momentum.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getMomentumString(this);
   }
}
