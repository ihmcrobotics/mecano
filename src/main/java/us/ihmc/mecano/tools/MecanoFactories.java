package us.ihmc.mecano.tools;

import java.util.function.DoubleSupplier;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

/**
 * This class provides a varieties of factories to create Euclid types and vectors.
 *
 * @author Sylvain Bertrand
 */
public class MecanoFactories
{
   private static final double TRANSFORM_UPDATER_EPSILON = 1.0e-7;

   /**
    * Creates the reference frame that is at the joint and rigidly attached to the joint's predecessor.
    *
    * @param joint             the joint to which the new frame is for. Not modified.
    * @param transformToParent the transform from the new frame to the frame after the parent joint.
    *                          Not modified.
    * @return the new frame before joint.
    */
   public static MovingReferenceFrame newFrameBeforeJoint(JointReadOnly joint, RigidBodyTransformReadOnly transformToParent)
   {
      String beforeJointName = "before" + MecanoTools.capitalize(joint.getName());

      return newJointFrame(joint, transformToParent, beforeJointName);
   }

   /**
    * Creates the reference frame that is at the joint and rigidly attached to the joint's predecessor.
    *
    * @param joint             the joint to which the new frame is for. Not modified.
    * @param transformToParent the transform from the new frame to the frame after the parent joint.
    *                          Not modified.
    * @param beforeJointName   the name of the new frame.
    * @return the new frame before joint.
    */
   public static MovingReferenceFrame newJointFrame(JointReadOnly joint, RigidBodyTransformReadOnly transformToParent, String beforeJointName)
   {
      MovingReferenceFrame parentFrame;
      RigidBodyReadOnly parentBody = joint.getPredecessor();
      if (parentBody.isRootBody())
      {
         parentFrame = parentBody.getBodyFixedFrame();

         /*
          * TODO Special case to keep the beforeJointFrame of the SixDoFJoint to be the elevatorFrame. This
          * should probably removed, might cause reference frame exceptions though.
          */
         if (transformToParent == null)
            return parentFrame;
      }
      else
         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      return MovingReferenceFrame.constructFrameFixedInParent(beforeJointName, parentFrame, transformToParent);
   }

   /**
    * Creates the reference frame that is at the joint and rigidly attached to the joint's successor.
    *
    * @param joint the joint to which the new frame is for. Not modified.
    * @return the new frame after joint.
    */
   public static MovingReferenceFrame newFrameAfterJoint(JointReadOnly joint)
   {
      MovingReferenceFrame frameBeforeJoint = joint.getFrameBeforeJoint();

      if (frameBeforeJoint == null)
         throw new NullPointerException("The frameBeforeJoint has to be created before the frameAfterJoint.");

      return new MovingReferenceFrame("after" + MecanoTools.capitalize(joint.getName()), frameBeforeJoint)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            joint.getJointConfiguration(transformToParent);
         }

         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            twistRelativeToParentToPack.setIncludingFrame(joint.getJointTwist());
         }
      };
   }

   /**
    * Creates a new transform updater that is optimized when the joint axis is aligned with either the
    * x, y, or z axis of the local frame.
    * 
    * @param joint the joint to create the updater for. Not modified.
    * @return the joint transform updater.
    */
   public static RevoluteJointTransformUpdater newRevoluteJointTransformUpdater(RevoluteJointReadOnly joint)
   {
      FrameVector3DReadOnly jointAxis = joint.getJointAxis();

      RevoluteJointTransformUpdater jointTransformUpdater;

      if (jointAxis.geometricallyEquals(Axis.X, TRANSFORM_UPDATER_EPSILON))
      {
         jointTransformUpdater = transform -> transform.setRotationRollAndZeroTranslation(joint.getQ());
      }
      else if (jointAxis.geometricallyEquals(Axis.Y, TRANSFORM_UPDATER_EPSILON))
      {
         jointTransformUpdater = transform -> transform.setRotationPitchAndZeroTranslation(joint.getQ());
      }
      else if (jointAxis.geometricallyEquals(Axis.Z, TRANSFORM_UPDATER_EPSILON))
      {
         jointTransformUpdater = transform -> transform.setRotationYawAndZeroTranslation(joint.getQ());
      }
      else
      {
         AxisAngle axisAngle = new AxisAngle();
         jointTransformUpdater = transform ->
         {
            axisAngle.set(joint.getJointAxis(), joint.getQ());
            transform.setRotationAndZeroTranslation(axisAngle);
         };
      }

      return jointTransformUpdater;
   }

   /**
    * Implementations of this interface can provide a custom updater that benefits from the joint
    * property and allows to optimize the joint transform calculation.
    * 
    * @author Sylvain Bertrand
    */
   public static interface RevoluteJointTransformUpdater
   {
      /**
       * Updates the joint transform given its current configuration.
       * 
       * @param jointTransformToUpdate the transform to update. Modified.
       */
      void updateJointTransform(RigidBodyTransform jointTransformToUpdate);
   }

   /**
    * Creates quaternion that is restricted to describe a rotation around the y-axis.
    *
    * @return the new quaternion.
    */
   public static QuaternionBasics newPitchOnlyQuaternionBasics()
   {
      return new QuaternionBasics()
      {
         private double x, y, z, s;

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return y;
         }

         @Override
         public double getZ()
         {
            return z;
         }

         @Override
         public double getS()
         {
            return s;
         }

         @Override
         public void setUnsafe(double qx, double qy, double qz, double qs)
         {
            x = qx;
            y = qy;
            z = qz;
            s = qs;

            if (Math.abs(x) > 1.0e-5 || Math.abs(z) > 1.0e-5)
               setToPitchOrientation(getPitch());
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple4DString(this);
         }
      };
   }

   /**
    * Creates a new point that is restricted to the XZ-plane.
    *
    * @return the point.
    */
   public static Point3DBasics newXZOnlyPoint3DBasics()
   {
      return new Point3DBasics()
      {
         private double x, z;

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return 0.0;
         }

         @Override
         public double getZ()
         {
            return z;
         }

         @Override
         public void setX(double x)
         {
            this.x = x;
         }

         @Override
         public void setY(double y)
         {
         }

         @Override
         public void setZ(double z)
         {
            this.z = z;
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a vector that is restricted to the XZ-plane.
    *
    * @param referenceFrame the reference frame of the new vector.
    * @return the new vector.
    */
   public static FixedFrameVector3DBasics newXZOnlyFixedFrameVector3DBasics(ReferenceFrame referenceFrame)
   {
      return new FixedFrameVector3DBasics()
      {
         private double x, z;

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrame;
         }

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return 0.0;
         }

         @Override
         public double getZ()
         {
            return z;
         }

         @Override
         public void setX(double x)
         {
            this.x = x;
         }

         @Override
         public void setY(double y)
         {
         }

         @Override
         public void setZ(double z)
         {
            this.z = z;
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   /**
    * Creates a vector that is restricted to the y-axis.
    *
    * @param referenceFrame the reference frame of the new vector.
    * @return the new vector.
    */
   public static FixedFrameVector3DBasics newYOnlyFixedFrameVector3DBasics(ReferenceFrame referenceFrame)
   {
      return new FixedFrameVector3DBasics()
      {
         private double y;

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
            return y;
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
            this.y = y;
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
    * Creates a new instance of a fixed-frame vector 3D which frame is linked to the frame of the given
    * {@code referenceFrameHolder}.
    *
    * @param referenceFrameHolder the object that will be used to obtain the frame of the new vector.
    * @return a new instance of a fixed-frame vector 3D.
    */
   public static FixedFrameVector3DBasics newFixedFrameVector3DBasics(ReferenceFrameHolder referenceFrameHolder)
   {
      return new FixedFrameVector3DBasics()
      {
         private double x, y, z;

         @Override
         public void setX(double x)
         {
            this.x = x;
         }

         @Override
         public void setY(double y)
         {
            this.y = y;
         }

         @Override
         public void setZ(double z)
         {
            this.z = z;
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return x;
         }

         @Override
         public double getY()
         {
            return y;
         }

         @Override
         public double getZ()
         {
            return z;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FixedFrameVector3DBasics)
               return super.equals(object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this) + "-" + getReferenceFrame();
         }
      };
   }

   /**
    * Creates a new point that is linked to the {@code referenceTuple} as follows:
    *
    * <pre>
    * newPoint = scale * referenceTuple
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier  the supplier to get the scale.
    * @param referenceTuple the reference tuple to scale. Not modified.
    * @return the new point linked to {@code referenceTuple}.
    */
   public static FramePoint3DReadOnly newFramePoint3DReadOnly(DoubleSupplier scaleSupplier, FrameTuple3DReadOnly referenceTuple)
   {
      return new FramePoint3DReadOnly()
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceTuple.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return scaleSupplier.getAsDouble() * referenceTuple.getX();
         }

         @Override
         public double getY()
         {
            return scaleSupplier.getAsDouble() * referenceTuple.getY();
         }

         @Override
         public double getZ()
         {
            return scaleSupplier.getAsDouble() * referenceTuple.getZ();
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this) + "-" + getReferenceFrame();
         }
      };
   }

   /**
    * Creates a new vector that is linked to the {@code referenceVector} as follows:
    *
    * <pre>
    * newVector = scale * referenceVector
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier   the supplier to get the scale.
    * @param referenceVector the reference vector to scale. Not modified.
    * @return the new vector linked to {@code referenceVector}.
    */
   public static FrameVector3DReadOnly newFrameVector3DReadOnly(DoubleSupplier scaleSupplier, FrameVector3DReadOnly referenceVector)
   {
      return new FrameVector3DReadOnly()
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceVector.getReferenceFrame();
         }

         @Override
         public double getX()
         {
            return scaleSupplier.getAsDouble() * referenceVector.getX();
         }

         @Override
         public double getY()
         {
            return scaleSupplier.getAsDouble() * referenceVector.getY();
         }

         @Override
         public double getZ()
         {
            return scaleSupplier.getAsDouble() * referenceVector.getZ();
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this) + "-" + getReferenceFrame();
         }
      };
   }

   /**
    * Creates a new pose 3D that is restricted to the XZ-plane.
    *
    * @return the new pose 3D.
    */
   public static Pose3DBasics newPlanarPose3DBasics()
   {
      return new Pose3DBasics()
      {
         private final QuaternionBasics jointRotation = newPitchOnlyQuaternionBasics();
         private final Point3DBasics jointTranslation = newXZOnlyPoint3DBasics();

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
    * Creates a new twist that is linked to the given {@code referenceTwist} as follows:
    *
    * <pre>
    * newTwist = scale * referenceTwist
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier  the supplier to get the scale.
    * @param referenceTwist the reference twist to scale. Not modified.
    * @return the new twist linked to the {@code referenceTwist}.
    */
   public static TwistReadOnly newTwistReadOnly(DoubleSupplier scaleSupplier, TwistReadOnly referenceTwist)
   {
      return new TwistReadOnly()
      {
         private final FrameVector3DReadOnly angularPart = newFrameVector3DReadOnly(scaleSupplier, referenceTwist.getAngularPart());
         private final FrameVector3DReadOnly linearPart = newFrameVector3DReadOnly(scaleSupplier, referenceTwist.getLinearPart());

         @Override
         public ReferenceFrame getBodyFrame()
         {
            return referenceTwist.getBodyFrame();
         }

         @Override
         public ReferenceFrame getBaseFrame()
         {
            return referenceTwist.getBaseFrame();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceTwist.getReferenceFrame();
         }

         @Override
         public FrameVector3DReadOnly getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FrameVector3DReadOnly getLinearPart()
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
    * Creates a new twist that is linked to the given {@code angularPart} and {@code linearPart}.
    *
    * @param bodyFrame   the twist's body frame.
    * @param baseFrame   the twist's base frame.
    * @param angularPart the vector holding the angular part the twist should be linked to.
    * @param linearPart  the vector holding the linear part the twist should be linked to.
    * @return the new twist linked to the two vectors.
    */
   public static TwistReadOnly newTwistReadOnly(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector3DReadOnly angularPart,
                                                FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);

      return new TwistReadOnly()
      {
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
            angularPart.checkReferenceFrameMatch(linearPart);
            return angularPart.getReferenceFrame();
         }

         @Override
         public FrameVector3DReadOnly getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FrameVector3DReadOnly getLinearPart()
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
    * Creates a twist that is constrained to the XZ-plane.
    * <p>
    * The frames of the new twist cannot be changed.
    * </p>
    *
    * @param bodyFrame        the twist's body frame.
    * @param baseFrame        the twist's base frame.
    * @param expressedInFrame the twist
    * @return the new twist.
    */
   public static FixedFrameTwistBasics newPlanarFixedFrameTwistBasics(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      return new FixedFrameTwistBasics()
      {
         private final FixedFrameVector3DBasics angularPart = newYOnlyFixedFrameVector3DBasics(expressedInFrame);
         private final FixedFrameVector3DBasics linearPart = newXZOnlyFixedFrameVector3DBasics(expressedInFrame);

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
    * Creates a new acceleration that is linked to the given {@code referenceAcceleration} as follows:
    *
    * <pre>
    * newAcceleration = scale * referenceAcceleration
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier         the supplier to get the scale.
    * @param referenceAcceleration the reference acceleration. Not modified.
    * @return the new acceleration linked to the {@code referenceAcceleration}.
    */
   public static SpatialAccelerationReadOnly newSpatialAccelerationVectorReadOnly(DoubleSupplier scaleSupplier,
                                                                                  SpatialAccelerationReadOnly referenceAcceleration)
   {
      return new SpatialAccelerationReadOnly()
      {
         private final FrameVector3DReadOnly angularPart = newFrameVector3DReadOnly(scaleSupplier, referenceAcceleration.getAngularPart());
         private final FrameVector3DReadOnly linearPart = newFrameVector3DReadOnly(scaleSupplier, referenceAcceleration.getLinearPart());

         @Override
         public ReferenceFrame getBodyFrame()
         {
            return referenceAcceleration.getBodyFrame();
         }

         @Override
         public ReferenceFrame getBaseFrame()
         {
            return referenceAcceleration.getBaseFrame();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceAcceleration.getReferenceFrame();
         }

         @Override
         public FrameVector3DReadOnly getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FrameVector3DReadOnly getLinearPart()
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
    * Creates a new acceleration that is linked to the given {@code angularPart} and
    * {@code linearPart}.
    *
    * @param bodyFrame   the acceleration's body frame.
    * @param baseFrame   the acceleration's base frame.
    * @param angularPart the vector holding the angular part the acceleration should be linked to.
    * @param linearPart  the vector holding the linear part the acceleration should be linked to.
    * @return the new acceleration linked to the two vectors.
    */
   public static SpatialAccelerationReadOnly newSpatialAccelerationVectorReadOnly(ReferenceFrame bodyFrame, ReferenceFrame baseFrame,
                                                                                  FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);

      return new SpatialAccelerationReadOnly()
      {
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
            angularPart.checkReferenceFrameMatch(linearPart);
            return angularPart.getReferenceFrame();
         }

         @Override
         public FrameVector3DReadOnly getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FrameVector3DReadOnly getLinearPart()
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
    * Creates a acceleration that is constrained to the XZ-plane.
    * <p>
    * The frames of the new acceleration cannot be changed.
    * </p>
    *
    * @param bodyFrame        the acceleration's body frame.
    * @param baseFrame        the acceleration's base frame.
    * @param expressedInFrame the acceleration
    * @return the new acceleration.
    */
   public static FixedFrameSpatialAccelerationBasics newPlanarFixedFrameSpatialAccelerationVectorBasics(ReferenceFrame bodyFrame, ReferenceFrame baseFrame,
                                                                                                        ReferenceFrame expressedInFrame)
   {
      return new FixedFrameSpatialAccelerationBasics()
      {
         private final FixedFrameVector3DBasics angularPart = newYOnlyFixedFrameVector3DBasics(expressedInFrame);
         private final FixedFrameVector3DBasics linearPart = newXZOnlyFixedFrameVector3DBasics(expressedInFrame);

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
    * Creates a new wrench that is linked to the given {@code referenceWrench} as follows:
    *
    * <pre>
    * newWrench = scale * referenceWrench
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier   the supplier to get the scale.
    * @param referenceWrench the reference wrench to scale. Not modified.
    * @return the new wrench linked to the {@code referenceWrench}.
    */
   public static WrenchReadOnly newWrenchReadOnly(DoubleSupplier scaleSupplier, WrenchReadOnly referenceWrench)
   {
      return new WrenchReadOnly()
      {
         private final FrameVector3DReadOnly angularPart = newFrameVector3DReadOnly(scaleSupplier, referenceWrench.getAngularPart());
         private final FrameVector3DReadOnly linearPart = newFrameVector3DReadOnly(scaleSupplier, referenceWrench.getLinearPart());

         @Override
         public ReferenceFrame getBodyFrame()
         {
            return referenceWrench.getBodyFrame();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceWrench.getReferenceFrame();
         }

         @Override
         public FrameVector3DReadOnly getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FrameVector3DReadOnly getLinearPart()
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

   /**
    * Creates a new wrench that is linked to the given {@code angularPart} and {@code linearPart}.
    *
    * @param bodyFrame   the wrench's body frame.
    * @param angularPart the vector holding the angular part the wrench should be linked to.
    * @param linearPart  the vector holding the linear part the wrench should be linked to.
    * @return the new wrench linked to the two vectors.
    */
   public static WrenchReadOnly newWrenchReadOnly(ReferenceFrame bodyFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);

      return new WrenchReadOnly()
      {
         @Override
         public ReferenceFrame getBodyFrame()
         {
            return bodyFrame;
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            angularPart.checkReferenceFrameMatch(linearPart);
            return angularPart.getReferenceFrame();
         }

         @Override
         public FrameVector3DReadOnly getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FrameVector3DReadOnly getLinearPart()
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

   /**
    * Creates a new spatial impulse that is linked to the given {@code referenceSpatialImpulse} as
    * follows:
    *
    * <pre>
    * newWrench = scale * referenceSpatialImpulse
    * </pre>
    *
    * where the scale is obtained from the given {@code scaleSupplier}.
    *
    * @param scaleSupplier   the supplier to get the scale.
    * @param referenceSpatialImpulse the reference spatial impulse to scale. Not modified.
    * @return the new spatial impulse linked to the {@code referenceSpatialImpulse}.
    */
   public static SpatialImpulseReadOnly newSpatialImpulseReadOnly(DoubleSupplier scaleSupplier, SpatialImpulseReadOnly referenceSpatialImpulse)
   {
      return new SpatialImpulseReadOnly()
      {
         private final FrameVector3DReadOnly angularPart = newFrameVector3DReadOnly(scaleSupplier, referenceSpatialImpulse.getAngularPart());
         private final FrameVector3DReadOnly linearPart = newFrameVector3DReadOnly(scaleSupplier, referenceSpatialImpulse.getLinearPart());

         @Override
         public ReferenceFrame getBodyFrame()
         {
            return referenceSpatialImpulse.getBodyFrame();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceSpatialImpulse.getReferenceFrame();
         }

         @Override
         public FrameVector3DReadOnly getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FrameVector3DReadOnly getLinearPart()
         {
            return linearPart;
         }

         @Override
         public String toString()
         {
            return MecanoIOTools.getSpatialImpulseString(this);
         }
      };
   }

   /**
    * Creates a wrench that is constrained to the XZ-plane.
    * <p>
    * The frames of the new wrench cannot be changed.
    * </p>
    *
    * @param bodyFrame        the wrench's body frame.
    * @param expressedInFrame the wrench
    * @return the new wrench.
    */
   public static FixedFrameWrenchBasics newPlanarFixedFrameWrenchBasics(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      return new FixedFrameWrenchBasics()
      {
         private final FixedFrameVector3DBasics angularPart = newYOnlyFixedFrameVector3DBasics(expressedInFrame);
         private final FixedFrameVector3DBasics linearPart = newXZOnlyFixedFrameVector3DBasics(expressedInFrame);

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

   /**
    * Creates a new spatial acceleration that can be used to describe the gravitational acceleration
    * that a multi-system is under.
    * 
    * @param rootBody the very first body of the multi-body system.
    * @param gravity  the magnitude of the gravitational acceleration along the z-axis.
    * @return the spatial acceleration representing the gravitational acceleration.
    */
   public static SpatialAccelerationReadOnly newGravitationalSpatialAcceleration(RigidBodyReadOnly rootBody, double gravity)
   {
      Vector3D gravitationalAcceleration = new Vector3D(0.0, 0.0, gravity);
      Vector3D zero = new Vector3D();
      MovingReferenceFrame bodyFixedFrame = rootBody.getBodyFixedFrame();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      return new SpatialAcceleration(bodyFixedFrame, worldFrame, bodyFixedFrame, zero, gravitationalAcceleration);
   }

   /**
    * Creates a new matrix 3D that is a read-only view of the transpose of the given {@code original}.
    * 
    * @param original the original matrix to create linked transpose matrix for. Not modified.
    * @return the transpose read-only view of {@code original}.
    */
   public static Matrix3DReadOnly createTransposeLinkedMatrix3DReadOnly(Matrix3DBasics original)
   {
      return new Matrix3DReadOnly()
      {
         @Override
         public double getM00()
         {
            return original.getM00();
         }

         @Override
         public double getM01()
         {
            return original.getM10();
         }

         @Override
         public double getM02()
         {
            return original.getM20();
         }

         @Override
         public double getM10()
         {
            return original.getM01();
         }

         @Override
         public double getM11()
         {
            return original.getM11();
         }

         @Override
         public double getM12()
         {
            return original.getM21();
         }

         @Override
         public double getM20()
         {
            return original.getM02();
         }

         @Override
         public double getM21()
         {
            return original.getM12();
         }

         @Override
         public double getM22()
         {
            return original.getM22();
         }

         @Override
         public void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
         {
            Matrix3DTools.transform(this, matrixOriginal, matrixTransformed);
         }

         @Override
         public void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         {
            Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed);
         }

         @Override
         public void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
         {
            Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
         }

         @Override
         public void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         {
            Matrix3DTools.inverseTransform(this, vectorOriginal, vectorTransformed);
         }

         @Override
         public void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
         {
            Matrix3DTools.inverseTransform(this, matrixOriginal, matrixTransformed);
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getMatrixString(this);
         }
      };
   }
}