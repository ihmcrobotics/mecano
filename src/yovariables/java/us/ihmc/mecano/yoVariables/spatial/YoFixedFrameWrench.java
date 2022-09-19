package us.ihmc.mecano.yoVariables.spatial;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Implementation of {@link FixedFrameWrenchBasics} backed by {@link YoVariable}s.
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
 * force is measured.
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
public class YoFixedFrameWrench implements FixedFrameWrenchBasics, Settable<YoFixedFrameWrench>
{
   /**
    * The reference of the frame rigidly attached to the body on which this wrench is applied.
    */
   private final ReferenceFrame bodyFrame;
   /** This is where we store the internal data. */
   private final YoFixedFrameSpatialForce spatialForceVector;

   /**
    * Creates a new wrench with its components set to zero and initializes its reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param bodyFrame        the frame rigidly attached to the body on which this wrench is applied.
    * @param expressedInFrame the frame in which this wrench is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameWrench(String namePrefix, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this(bodyFrame, new YoFixedFrameSpatialVector(namePrefix, expressedInFrame, registry));
   }

   /**
    * Creates a new wrench with its components set to zero and initializes its reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param nameSuffix       a string to use as the suffix for child variable names.
    * @param bodyFrame        the frame rigidly attached to the body on which this wrench is applied.
    * @param expressedInFrame the frame in which this wrench is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameWrench(String namePrefix, String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this(bodyFrame, new YoFixedFrameSpatialVector(namePrefix, nameSuffix, expressedInFrame, registry));
   }

   /**
    * Creates a new wrench given its angular and linear parts and initializes its reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param bodyFrame   the frame rigidly attached to the body on which this wrench is applied.
    * @param angularPart the vector to use for the angular part.
    * @param linearPart  the vector to use for the linear part.
    */
   public YoFixedFrameWrench(ReferenceFrame bodyFrame, YoFrameVector3D angularPart, YoFrameVector3D linearPart)
   {
      this(bodyFrame, new YoFixedFrameSpatialVector(angularPart, linearPart));
   }

   /**
    * Creates a new wrench given the spatial vector holding data and initializes its reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param bodyFrame     the frame rigidly attached to the body on which this wrench is applied.
    * @param spatialVector the spatial vector to use for holding data.
    */
   public YoFixedFrameWrench(ReferenceFrame bodyFrame, YoFixedFrameSpatialVector spatialVector)
   {
      this.bodyFrame = bodyFrame;
      spatialForceVector = new YoFixedFrameSpatialForce(spatialVector);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoFixedFrameWrench other)
   {
      FixedFrameWrenchBasics.super.set(other);
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
   public YoFrameVector3D getAngularPart()
   {
      return spatialForceVector.getAngularPart();
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameVector3D getLinearPart()
   {
      return spatialForceVector.getLinearPart();
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
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(WrenchReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof WrenchReadOnly)
         return FixedFrameWrenchBasics.super.equals((WrenchReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this spatial force vector as follows:<br>
    * Wrench exerted on bodyFrame: [angular = (x, y, z), linear = (x, y, z)] - expressedInFrame
    *
    * @return the {@code String} representing this spatial force vector.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getWrenchString(this);
   }
}
