package us.ihmc.mecano.yoVariables.multiBodySystem.inertial;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSpatialInertia implements SpatialInertiaBasics
{
   private final ReferenceFrame bodyFrame;

   private final ReferenceFrame expressedInFrame;

   private final YoDouble mass, Ixx, Ixy, Iyy, Ixz, Iyz, Izz;

   private final YoFrameVector3D centerOfMassOffset;

   private final Matrix3DBasics momentOfInertia;  // for storing intermediate values

   public YoSpatialInertia(SpatialInertiaReadOnly input, YoRegistry registry)
   {
      bodyFrame = input.getBodyFrame();
      expressedInFrame = input.getReferenceFrame();

      mass = new YoDouble(bodyFrame.getName() + "_mass", registry);

      centerOfMassOffset = new YoFrameVector3D(bodyFrame.getName() + "_centerOfMassOffset", expressedInFrame, registry);

      Ixx = new YoDouble(bodyFrame.getName() + "_Ixx", registry);
      Ixy = new YoDouble(bodyFrame.getName() + "_Ixy", registry);
      Iyy = new YoDouble(bodyFrame.getName() + "_Iyy", registry);
      Ixz = new YoDouble(bodyFrame.getName() + "_Ixz", registry);
      Iyz = new YoDouble(bodyFrame.getName() + "_Iyz", registry);
      Izz = new YoDouble(bodyFrame.getName() + "_Izz", registry);

      momentOfInertia = new Matrix3D(input.getMomentOfInertia());
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return expressedInFrame;
   }

   @Override
   public Matrix3DBasics getMomentOfInertia()
   {
      momentOfInertia.set(Ixx.getDoubleValue(), Ixy.getDoubleValue(), Ixz.getDoubleValue(),
                          Ixy.getDoubleValue(), Iyy.getDoubleValue(), Iyz.getDoubleValue(),
                          Ixz.getDoubleValue(), Iyz.getDoubleValue(), Izz.getDoubleValue());
      return momentOfInertia;
   }

   @Override
   public double getMass()
   {
      return mass.getDoubleValue();
   }

   @Override
   public void setMass(double mass)
   {
      this.mass.set(mass);
   }

   @Override
   public FixedFrameVector3DBasics getCenterOfMassOffset()
   {
      return centerOfMassOffset;
   }

   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {

   }
}
