package us.ihmc.mecano.yoVariables.tools;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class provides a varieties of factories to create Euclid types and vectors that are backed
 * by {@code YoVariable}s.
 *
 * @author Sylvain Bertrand
 */
public class YoMecanoFactories
{
   /**
    * Creates a new pose 3D that is restricted to the XZ-plane and which components are backed by
    * {@code YoVariable}s.
    *
    * @param yoPitch the {@code YoVariable} used for the pitch-component of the orientation.
    * @param yoX     the {@code YoVariable} used for the x-component of the position.
    * @param yoZ     the {@code YoVariable} used for the z-component of the position.
    * @return the new pose 3D.
    */
   public static Pose3DBasics newPlanarYoPose3DBasics(YoDouble yoPitch, YoDouble yoX, YoDouble yoZ)
   {
      return new Pose3DBasics()
      {
         private final QuaternionBasics jointRotation = newPitchOnlyYoQuaternionBasics(yoPitch);
         private final Point3DBasics jointTranslation = newXZOnlyYoPoint3DBasics(yoX, yoZ);

         @Override
         public Point3DBasics getPosition()
         {
            return jointTranslation;
         }

         @Override
         public QuaternionBasics getOrientation()
         {
            return jointRotation;
         }
      };
   }

   /**
    * Creates quaternion that is restricted to describe a rotation around the y-axis and which angle is
    * backed by a {@code YoVariable}.
    *
    * @param yoPitch the {@code YoVariable} used for the angle.
    * @return the new quaternion.
    */
   public static QuaternionBasics newPitchOnlyYoQuaternionBasics(YoDouble yoPitch)
   {
      return new QuaternionBasics()
      {
         private double y, s;

         @Override
         public double getX()
         {
            return 0.0;
         }

         @Override
         public double getY()
         {
            return y;
         }

         @Override
         public double getZ()
         {
            return 0.0;
         }

         @Override
         public double getS()
         {
            return s;
         }

         @Override
         public void setUnsafe(double qx, double qy, double qz, double qs)
         {
            double pitchArgument = 2.0 * (qs * qy - qx * qz);

            if (pitchArgument > 1.0)
               pitchArgument = 1.0;
            else if (pitchArgument < -1.0)
               pitchArgument = -1.0;

            yoPitch.set(Math.asin(pitchArgument));

            double halfPitch = 0.5 * yoPitch.getValue();
            y = Math.sin(halfPitch);
            s = Math.cos(halfPitch);
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple4DString(this);
         }
      };
   }

   /**
    * Creates a new point that is restricted to the XZ-plane and which components are backed by
    * {@code YoVariable}s.
    *
    * @param yoX the {@code YoVariable} used for the x-component.
    * @param yoZ the {@code YoVariable} used for the z-component.
    * @return the point.
    */
   public static Point3DBasics newXZOnlyYoPoint3DBasics(YoDouble yoX, YoDouble yoZ)
   {
      return new Point3DBasics()
      {
         @Override
         public double getX()
         {
            return yoX.getValue();
         }

         @Override
         public double getY()
         {
            return 0.0;
         }

         @Override
         public double getZ()
         {
            return yoZ.getValue();
         }

         @Override
         public void setX(double x)
         {
            yoX.set(x);
         }

         @Override
         public void setY(double y)
         {
         }

         @Override
         public void setZ(double z)
         {
            yoZ.set(z);
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a vector that is restricted to the XZ-plane and which components are backed by
    * {@code YoVariable}s.
    *
    * @param yoX            the {@code YoVariable} to use for the x-component.
    * @param yoZ            the {@code YoVariable} to use for the z-component.
    * @param referenceFrame the reference frame of the new vector.
    * @return the new vector.
    */
   public static FixedFrameVector3DBasics newXZOnlyYoFixedFrameVector3DBasics(YoDouble yoX, YoDouble yoZ, ReferenceFrame referenceFrame)
   {
      return new FixedFrameVector3DBasics()
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrame;
         }

         @Override
         public double getX()
         {
            return yoX.getValue();
         }

         @Override
         public double getY()
         {
            return 0.0;
         }

         @Override
         public double getZ()
         {
            return yoZ.getValue();
         }

         @Override
         public void setX(double x)
         {
            yoX.set(x);
         }

         @Override
         public void setY(double y)
         {
         }

         @Override
         public void setZ(double z)
         {
            yoZ.set(z);
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a vector that is restricted to the y-axis and which component is backed by a
    * {@code YoVariable}.
    *
    * @param yoY            the {@code YoVariable} to use for the y-component.
    * @param referenceFrame the reference frame of the new vector.
    * @return the new vector.
    */
   public static FixedFrameVector3DBasics newYOnlyYoFixedFrameVector3DBasics(YoDouble yoY, ReferenceFrame referenceFrame)
   {
      return new FixedFrameVector3DBasics()
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrame;
         }

         @Override
         public double getX()
         {
            return 0.0;
         }

         @Override
         public double getY()
         {
            return yoY.getValue();
         }

         @Override
         public double getZ()
         {
            return 0.0;
         }

         @Override
         public void setX(double x)
         {
         }

         @Override
         public void setY(double y)
         {
            yoY.set(y);
         }

         @Override
         public void setZ(double z)
         {
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a twist that is constrained to the XZ-plane and which components are backed by
    * {@code YoVariable}s.
    * <p>
    * The frames of the new twist cannot be changed.
    * </p>
    *
    * @param angularY         the {@code YoVariable} to use for the y-component of the angular part.
    * @param linearX          the {@code YoVariable} to use for the x-component of the linear part.
    * @param linearZ          the {@code YoVariable} to use for the z-component of the linear part.
    * @param bodyFrame        the twist's body frame.
    * @param baseFrame        the twist's base frame.
    * @param expressedInFrame the twist
    * @return the new twist.
    */
   public static FixedFrameTwistBasics newPlanarYoFixedFrameTwistBasics(YoDouble angularY, YoDouble linearX, YoDouble linearZ, ReferenceFrame bodyFrame,
                                                                        ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      return new FixedFrameTwistBasics()
      {
         private final FixedFrameVector3DBasics angularPart = newYOnlyYoFixedFrameVector3DBasics(angularY, expressedInFrame);
         private final FixedFrameVector3DBasics linearPart = newXZOnlyYoFixedFrameVector3DBasics(linearX, linearZ, expressedInFrame);

         @Override
         public ReferenceFrame getBodyFrame()
         {
            return bodyFrame;
         }

         @Override
         public ReferenceFrame getBaseFrame()
         {
            return baseFrame;
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return expressedInFrame;
         }

         @Override
         public FixedFrameVector3DBasics getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FixedFrameVector3DBasics getLinearPart()
         {
            return linearPart;
         }

         @Override
         public String toString()
         {
            return MecanoIOTools.getTwistString(this);
         }
      };
   }

   /**
    * Creates a acceleration that is constrained to the XZ-plane and which components are backed by
    * {@code YoVariable}s.
    * <p>
    * The frames of the new acceleration cannot be changed.
    * </p>
    *
    * @param angularY         the {@code YoVariable} to use for the y-component of the angular part.
    * @param linearX          the {@code YoVariable} to use for the x-component of the linear part.
    * @param linearZ          the {@code YoVariable} to use for the z-component of the linear part.
    * @param bodyFrame        the acceleration's body frame.
    * @param baseFrame        the acceleration's base frame.
    * @param expressedInFrame the acceleration
    * @return the new acceleration.
    */
   public static FixedFrameSpatialAccelerationBasics newPlanarYoFixedFrameSpatialAccelerationVectorBasics(YoDouble angularY, YoDouble linearX, YoDouble linearZ,
                                                                                                          ReferenceFrame bodyFrame, ReferenceFrame baseFrame,
                                                                                                          ReferenceFrame expressedInFrame)
   {
      return new FixedFrameSpatialAccelerationBasics()
      {
         private final FixedFrameVector3DBasics angularPart = newYOnlyYoFixedFrameVector3DBasics(angularY, expressedInFrame);
         private final FixedFrameVector3DBasics linearPart = newXZOnlyYoFixedFrameVector3DBasics(linearX, linearZ, expressedInFrame);

         @Override
         public ReferenceFrame getBodyFrame()
         {
            return bodyFrame;
         }

         @Override
         public ReferenceFrame getBaseFrame()
         {
            return baseFrame;
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return expressedInFrame;
         }

         @Override
         public FixedFrameVector3DBasics getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FixedFrameVector3DBasics getLinearPart()
         {
            return linearPart;
         }

         @Override
         public String toString()
         {
            return MecanoIOTools.getSpatialAccelerationString(this);
         }
      };
   }

   /**
    * Creates a wrench that is constrained to the XZ-plane and which components are backed by
    * {@code YoVariable}s.
    * <p>
    * The frames of the new wrench cannot be changed.
    * </p>
    *
    * @param angularY         the {@code YoVariable} to use for the y-component of the angular part.
    * @param linearX          the {@code YoVariable} to use for the x-component of the linear part.
    * @param linearZ          the {@code YoVariable} to use for the z-component of the linear part.
    * @param bodyFrame        the wrench's body frame.
    * @param expressedInFrame the wrench
    * @return the new wrench.
    */
   public static FixedFrameWrenchBasics newPlanarYoFixedFrameWrenchBasics(YoDouble angularY, YoDouble linearX, YoDouble linearZ, ReferenceFrame bodyFrame,
                                                                          ReferenceFrame expressedInFrame)
   {
      return new FixedFrameWrenchBasics()
      {
         private final FixedFrameVector3DBasics angularPart = newYOnlyYoFixedFrameVector3DBasics(angularY, expressedInFrame);
         private final FixedFrameVector3DBasics linearPart = newXZOnlyYoFixedFrameVector3DBasics(linearX, linearZ, expressedInFrame);

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
         public FixedFrameVector3DBasics getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FixedFrameVector3DBasics getLinearPart()
         {
            return linearPart;
         }

         @Override
         public String toString()
         {
            return MecanoIOTools.getWrenchString(this);
         }
      };
   }
}
