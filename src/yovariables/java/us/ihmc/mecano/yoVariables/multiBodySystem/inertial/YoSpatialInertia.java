package us.ihmc.mecano.yoVariables.multiBodySystem.inertial;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSpatialInertia implements SpatialInertiaBasics
{
   private final YoDouble mass, Ixx, Ixy, Iyy, Ixz, Iyz, Izz;

   private final YoFrameVector3D centerOfMassOffset;

   private final SpatialInertia spatialInertia;

   public YoSpatialInertia(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this("", bodyFrame, expressedInFrame, registry);
   }

   public YoSpatialInertia(SpatialInertiaReadOnly input, String nameSuffix, YoRegistry registry)
   {
      this(nameSuffix, input.getBodyFrame(), input.getReferenceFrame(), registry);

      mass.set(input.getMass());

      centerOfMassOffset.set(input.getCenterOfMassOffset());

      Ixx.set(input.getMomentOfInertia().getM00());
      Ixy.set(input.getMomentOfInertia().getM01());
      Iyy.set(input.getMomentOfInertia().getM11());
      Ixz.set(input.getMomentOfInertia().getM02());
      Iyz.set(input.getMomentOfInertia().getM12());
      Izz.set(input.getMomentOfInertia().getM22());

      spatialInertia.set(input);
   }

   public YoSpatialInertia(String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      mass = new YoDouble(bodyFrame.getName() + "_mass_" + nameSuffix, registry);

      centerOfMassOffset = new YoFrameVector3D(bodyFrame.getName() + "_centerOfMassOffset_" + nameSuffix, expressedInFrame, registry);

      Ixx = new YoDouble(bodyFrame.getName() + "_Ixx_" + nameSuffix, registry);
      Ixy = new YoDouble(bodyFrame.getName() + "_Ixy_" + nameSuffix, registry);
      Iyy = new YoDouble(bodyFrame.getName() + "_Iyy_" + nameSuffix, registry);
      Ixz = new YoDouble(bodyFrame.getName() + "_Ixz_" + nameSuffix, registry);
      Iyz = new YoDouble(bodyFrame.getName() + "_Iyz_" + nameSuffix, registry);
      Izz = new YoDouble(bodyFrame.getName() + "_Izz_" + nameSuffix, registry);
      spatialInertia = new SpatialInertia(bodyFrame, expressedInFrame);
   }

   @Override
   public ReferenceFrame getBodyFrame()
   {
      return spatialInertia.getBodyFrame();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return spatialInertia.getReferenceFrame();
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
      return spatialInertia.getMomentOfInertia();
   }

   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {
      spatialInertia.setBodyFrame(bodyFrame);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      spatialInertia.setReferenceFrame(referenceFrame);
   }

   @Override
   public void setMass(double mass)
   {
      spatialInertia.setMass(mass);
      this.mass.set(mass);
   }

   @Override
   public void setCenterOfMassOffset(Tuple3DReadOnly offset)
   {
      spatialInertia.setCenterOfMassOffset(offset);
      centerOfMassOffset.set(offset);
   }

   @Override
   public void setCenterOfMassOffset(double x, double y, double z)
   {
      spatialInertia.setCenterOfMassOffset(x, y, z);
      centerOfMassOffset.set(x, y, z);
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      spatialInertia.setMomentOfInertia(Ixx, Iyy, Izz);
      this.Ixx.set(Ixx);
      this.Iyy.set(Iyy);
      this.Izz.set(Izz);
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Ixy, double Ixz, double Iyy, double Iyz, double Izz)
   {
      spatialInertia.setMomentOfInertia(Ixx, Ixy, Ixz, Iyy, Iyz, Izz);
      this.Ixx.set(Ixx);
      this.Ixy.set(Ixy);
      this.Ixz.set(Ixz);
      this.Iyy.set(Iyy);
      this.Iyz.set(Iyz);
      this.Izz.set(Izz);
   }

   @Override
   public void setToZero(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      spatialInertia.setToZero(bodyFrame, expressedInFrame);
      mass.set(0);
      centerOfMassOffset.setToZero();
      Ixx.set(0);
      Ixy.set(0);
      Ixz.set(0);
      Iyy.set(0);
      Iyz.set(0);
      Izz.set(0);
   }
}
