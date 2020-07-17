package us.ihmc.mecano.yoVariables.spatial;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Implementation of {@link FixedFrameSpatialVectorBasics} backed by {@link YoVariable}s.
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
public class YoFixedFrameSpatialVector implements FixedFrameSpatialVectorBasics
{
   /** The 3D vector representing the angular part of this spatial vector. */
   private final YoFrameVector3D angularPart;
   /** The 3D vector representing the linear part of this spatial vector. */
   private final YoFrameVector3D linearPart;

   /**
    * Creates a new spatial vector with its components set to zero and initializes its reference frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param expressedInFrame in which reference frame the spatial vector is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameSpatialVector(String namePrefix, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this(namePrefix, "", expressedInFrame, registry);
   }

   /**
    * Creates a new spatial vector with its components set to zero and initializes its reference frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param nameSuffix       a string to use as the suffix for child variable names.
    * @param expressedInFrame in which reference frame the spatial vector is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameSpatialVector(String namePrefix, String nameSuffix, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this(new YoFrameVector3D(namePrefix + "Angular", nameSuffix, expressedInFrame, registry),
           new YoFrameVector3D(namePrefix + "Linear", nameSuffix, expressedInFrame, registry));
   }

   /**
    * Creates a new spatial vector given its angular and linear parts and initializes its reference
    * frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param angularPart the vector to use for the angular part.
    * @param linearPart  the vector to use for the linear part.
    */
   public YoFixedFrameSpatialVector(YoFrameVector3D angularPart, YoFrameVector3D linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);

      this.angularPart = angularPart;
      this.linearPart = linearPart;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return angularPart.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameVector3D getAngularPart()
   {
      return angularPart;
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameVector3D getLinearPart()
   {
      return linearPart;
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
         return FixedFrameSpatialVectorBasics.super.equals((SpatialVectorReadOnly) object);
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
