package us.ihmc.mecano.spatial;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MecanoIOTools;

/**
 * A {@code SpatialVector} is a vector composed of 6 components with an angular part and a linear
 * part.
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
 * @author Sylvain Bertrand
 */
public class SpatialVector implements SpatialVectorBasics
{
   /** The reference frame in which this spatial vector is expressed. */
   private ReferenceFrame expressedInFrame = ReferenceFrame.getWorldFrame();
   /** The 3D vector representing the angular part of this spatial vector. */
   private final FixedFrameVector3DBasics angularPart = MecanoFactories.newFixedFrameVector3DBasics(this);
   /** The 3D vector representing the linear part of this spatial vector. */
   private final FixedFrameVector3DBasics linearPart = MecanoFactories.newFixedFrameVector3DBasics(this);

   /**
    * Creates a new spatial vector with its components set to zero and its reference frame set to
    * {@code ReferenceFrame.getWorldFrame()}.
    */
   public SpatialVector()
   {
   }

   /**
    * Creates a new spatial vector with its components set to zero and initializes its reference frame.
    *
    * @param expressedInFrame the initial frame in which this vector is expressed.
    */
   public SpatialVector(ReferenceFrame expressedInFrame)
   {
      setToZero(expressedInFrame);
   }

   /**
    * Creates a new spatial vector and initializes its components and the reference frame it is
    * expressed in.
    *
    * @param expressedInFrame the initial frame in which this vector is expressed.
    * @param angularPart      the vector holding the values for the angular part. Not modified.
    * @param linearPart       the vector holding the values for the linear part. Not modified.
    */
   public SpatialVector(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setIncludingFrame(expressedInFrame, angularPart, linearPart);
   }

   /**
    * Creates a new spatial vector and initializes its components and the reference frame it is
    * expressed in.
    *
    * @param angularPart the vector holding the new values for the angular part. Not modified.
    * @param linearPart  the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   public SpatialVector(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      setIncludingFrame(angularPart, linearPart);
   }

   /**
    * Copy constructor.
    *
    * @param other the other spatial vector to copy. Not modified.
    */
   public SpatialVector(SpatialVectorReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return expressedInFrame;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getAngularPart()
   {
      return angularPart;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getLinearPart()
   {
      return linearPart;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      this.expressedInFrame = expressedInFrame;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(SpatialVectorReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof SpatialVectorReadOnly)
         return SpatialVectorBasics.super.equals((SpatialVectorReadOnly) object);
      else
         return super.equals(object);
   }

   /**
    * Provides a {@code String} representation of this spatial vector as follows:<br>
    * Spatial Vector: [angular = (x, y, z), linear = (x, y, z)] - expressedInFrame
    *
    * @return the {@code String} representing this spatial vector.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getSpatialVectorString(this);
   }
}
