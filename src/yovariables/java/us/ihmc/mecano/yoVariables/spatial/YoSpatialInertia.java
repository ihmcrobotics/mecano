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

public class YoSpatialInertia implements SpatialInertiaBasics, Settable<SpatialInertia>
{
   private final YoDouble mass;
   private final YoFrameVector3D centerOfMassOffset;
   private final YoMatrix3D momentOfInertia;

   private ReferenceFrame bodyFrame;
   private ReferenceFrame expressedInFrame;

   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D translation = new Point3D();

   public YoSpatialInertia(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this("", bodyFrame, expressedInFrame, registry);
   }

   public YoSpatialInertia(SpatialInertiaReadOnly input, String nameSuffix, YoRegistry registry)
   {
      this(nameSuffix, input.getBodyFrame(), input.getReferenceFrame(), registry);

      mass.set(input.getMass());
      centerOfMassOffset.set(input.getCenterOfMassOffset());
      momentOfInertia.set(input.getMomentOfInertia());
   }

   public YoSpatialInertia(String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this.bodyFrame = bodyFrame;
      this.expressedInFrame = expressedInFrame;

      mass = new YoDouble(bodyFrame.getName() + "_mass" + nameSuffix, registry);
      centerOfMassOffset = new YoFrameVector3D(bodyFrame.getName() + "_centerOfMassOffset" + nameSuffix, expressedInFrame, registry);
      momentOfInertia = new YoMatrix3D(bodyFrame.getName() + "_momentOfInertia", registry);
   }

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

   public YoDouble getYoMass()
   {
      return mass;
   }

   public YoFrameVector3D getYoCenterOfMassOffset()
   {
      return centerOfMassOffset;
   }

   public YoMatrix3D getYoMomentOfInertia()
   {
      return momentOfInertia;
   }

   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return expressedInFrame;
   }

   @Override
   public double getMass()
   {
      return mass.getDoubleValue();
   }

   @Override
   public FixedFrameVector3DBasics getCenterOfMassOffset()
   {
      return centerOfMassOffset;
   }

   @Override
   public Matrix3DBasics getMomentOfInertia()
   {
      return momentOfInertia;
   }

   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {
      this.bodyFrame = bodyFrame;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.expressedInFrame = referenceFrame;
   }

   @Override
   public void setMass(double mass)
   {
      this.mass.set(mass);
   }

   @Override
   public void setCenterOfMassOffset(Tuple3DReadOnly offset)
   {
      centerOfMassOffset.set(offset);
   }

   @Override
   public void setCenterOfMassOffset(double x, double y, double z)
   {
      centerOfMassOffset.set(x, y, z);
   }

   @Override
   public void set(SpatialInertia other)
   {
      mass.set(other.getMass());
      centerOfMassOffset.set(other.getCenterOfMassOffset());
      momentOfInertia.set(other.getMomentOfInertia());
   }
}
