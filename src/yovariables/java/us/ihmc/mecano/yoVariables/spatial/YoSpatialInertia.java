package us.ihmc.mecano.yoVariables.spatial;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.yoVariables.euclid.YoMatrix3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * A YoVariable backed version of {@link SpatialInertia} (see that documentation for more details).
 *
 * @author James Foster
 */
public class YoSpatialInertia implements SpatialInertiaBasics, Settable<SpatialInertia>
{
   /** YoVariable-backed total mass of the body. */
   private final YoDouble mass;
   /** YoVariable-backed offset of the body's center of mass position and the origin of {@link #expressedInFrame}. */
   private final YoFrameVector3D centerOfMassOffset;
   /** YoVariable-backed moment of inertia, or rotational inertia. */
   private final YoMatrix3D momentOfInertia;

   /** Reference frame rigidly attached to the body that this spatial inertia matrix describes the inertia of. */
   private ReferenceFrame bodyFrame;
   /** The reference frame in which this inertia is expressed. */
   private ReferenceFrame expressedInFrame;

   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D translation = new Point3D();

   /**
    * Creates a new YoVariable-backed spatial inertia with its components set to zero and initializes its reference frames.
    *
    * @param bodyFrame        what we are specifying the spatial inertia of.
    * @param expressedInFrame in which reference frame the spatial inertia is expressed.
    * @param registry         the registry to add the YoVariables to.
    */
   public YoSpatialInertia(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this("", bodyFrame, expressedInFrame, registry);
   }

   /**
    * Creates a new YoVariable-backed spatial inertia matrix from an existing {@link SpatialInertiaReadOnly}, copying {@code input}s components and
    * reference frames.
    *
    * @param input      the spatial inertia to copy.
    * @param nameSuffix a string to append to the end of the YoVariables that will be constructed.
    * @param registry   the registry to add the YoVariables to.
    */
   public YoSpatialInertia(SpatialInertiaReadOnly input, String nameSuffix, YoRegistry registry)
   {
      this(nameSuffix, input.getBodyFrame(), input.getReferenceFrame(), registry);

      mass.set(input.getMass());
      centerOfMassOffset.set(input.getCenterOfMassOffset());
      momentOfInertia.set(input.getMomentOfInertia());
   }

   /**
    * Creates a new YoVariable-backed spatial inertia matrix, initializing its reference frames but setting the components to zero.
    *
    * @param nameSuffix       a string to append to the end of the YoVariables that will be constructed.
    * @param bodyFrame        what we are specifying the spatial inertia of.
    * @param expressedInFrame in which reference frame the spatial inertia is expressed.
    * @param registry         the registry to add the YoVariables to.
    */
   public YoSpatialInertia(String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this.bodyFrame = bodyFrame;
      this.expressedInFrame = expressedInFrame;

      mass = new YoDouble(bodyFrame.getName() + "_mass" + nameSuffix, registry);
      centerOfMassOffset = new YoFrameVector3D(bodyFrame.getName() + "_centerOfMassOffset", nameSuffix, expressedInFrame, registry);
      momentOfInertia = new YoMatrix3D(bodyFrame.getName() + "_momentOfInertia", nameSuffix, registry);
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransformReadOnly)
      {
         applyTransform((RigidBodyTransformReadOnly) transform);
      }
      else
      { // General transform, only applying rotation and translation.
         translation.setToZero();
         translation.applyTransform(transform);
         momentOfInertia.applyTransform(transform);
         centerOfMassOffset.applyTransform(transform);
         MecanoTools.translateMomentOfInertia(mass.getDoubleValue(), centerOfMassOffset, false, translation, momentOfInertia);
         centerOfMassOffset.add(translation);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransformReadOnly)
      {
         applyInverseTransform((RigidBodyTransformReadOnly) transform);
      }
      else
      { // General transform, only applying rotation and translation.
         translation.setToZero();
         translation.applyTransform(transform);
         MecanoTools.translateMomentOfInertia(mass.getDoubleValue(), centerOfMassOffset, true, translation, momentOfInertia);
         centerOfMassOffset.sub(translation);
         momentOfInertia.applyInverseTransform(transform);
         centerOfMassOffset.applyInverseTransform(transform);
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
   public ReferenceFrame getReferenceFrame()
   {
      return expressedInFrame;
   }

   /** {@inheritDoc} */
   @Override
   public double getMass()
   {
      return mass.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getCenterOfMassOffset()
   {
      return centerOfMassOffset;
   }

   /** {@inheritDoc} */
   @Override
   public Matrix3DBasics getMomentOfInertia()
   {
      return momentOfInertia;
   }

   /** {@inheritDoc} */
   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {
      this.bodyFrame = bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.expressedInFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setMass(double mass)
   {
      this.mass.set(mass);
   }

   /** {@inheritDoc} */
   @Override
   public void setCenterOfMassOffset(Tuple3DReadOnly offset)
   {
      centerOfMassOffset.set(offset);
   }

   /** {@inheritDoc} */
   @Override
   public void setCenterOfMassOffset(double x, double y, double z)
   {
      centerOfMassOffset.set(x, y, z);
   }

   /** {@inheritDoc} */
   @Override
   public void set(SpatialInertia other)
   {
      mass.set(other.getMass());
      centerOfMassOffset.set(other.getCenterOfMassOffset());
      momentOfInertia.set(other.getMomentOfInertia());
   }
}
