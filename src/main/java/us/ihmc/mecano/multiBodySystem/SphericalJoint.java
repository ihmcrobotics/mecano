package us.ihmc.mecano.multiBodySystem;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * A {@code SphericalJoint} has 3 degrees of freedom of rotation.
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public class SphericalJoint extends Joint implements SphericalJointBasics
{
   /** The current orientation of this joint. */
   private final Quaternion jointOrientation = new Quaternion();
   /** The current angular velocity of this joint. */
   private final FixedFrameVector3DBasics jointAngularVelocity;
   /** The current angular acceleration of this joint. */
   private final FixedFrameVector3DBasics jointAngularAcceleration;
   /** The current torque of this joint. */
   private final FixedFrameVector3DBasics jointTorque;

   /**
    * This joint twist. Note that this field is automatically updated when
    * {@link #jointAngularVelocity} changes.
    */
   private final TwistReadOnly jointTwist;
   /**
    * This joint spatial acceleration. Note that this field is automatically updated when
    * {@link #jointAngularAcceleration} changes.
    */
   private final SpatialAccelerationReadOnly jointAcceleration;
   /**
    * This joint resulting wrench on its successor. Note that this field is automatically updated
    * when {@link #jointTorque} changes.
    */
   private WrenchReadOnly successorWrench;
   /**
    * The list containing the unit-twist for each degree of freedom of this joint. In the current
    * framework, the unit-twists are calculated once at construction and remain constant.
    */
   private final List<TwistReadOnly> unitTwists;

   /**
    * Creates a new spherical joint.
    * 
    * @param name the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param jointOffset the offset in translation with respect to the frame after the parent joint.
    *           Not modified.
    */
   public SphericalJoint(String name, RigidBodyBasics predecessor, Tuple3DReadOnly jointOffset)
   {
      this(name, predecessor, new RigidBodyTransform(new Quaternion(), jointOffset));
   }

   /**
    * Creates a new spherical joint.
    * 
    * @param name the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    */
   public SphericalJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
   {
      super(name, predecessor, transformToParent);
      jointAngularVelocity = new FrameVector3D(afterJointFrame);
      jointAngularAcceleration = new FrameVector3D(afterJointFrame);
      jointTwist = MecanoFactories.newTwistReadOnly(afterJointFrame, beforeJointFrame, jointAngularVelocity, new FrameVector3D(afterJointFrame));
      jointAcceleration = MecanoFactories.newSpatialAccelerationVectorReadOnly(afterJointFrame, beforeJointFrame, jointAngularAcceleration,
                                                                                          new FrameVector3D(afterJointFrame));
      unitTwists = MecanoTools.computeSphericalJointMotionSubspace(beforeJointFrame, afterJointFrame);
      jointTorque = new FrameVector3D(afterJointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      successorWrench = MecanoFactories.newWrenchReadOnly(successorFrame, jointTorque, new FrameVector3D(afterJointFrame));
   }

   /** {@inheritDoc} */
   @Override
   public QuaternionBasics getJointOrientation()
   {
      return jointOrientation;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getJointAngularVelocity()
   {
      return jointAngularVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getJointAngularAcceleration()
   {
      return jointAngularAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getJointTorque()
   {
      return jointTorque;
   }

   /** {@inheritDoc} */
   @Override
   public TwistReadOnly getJointTwist()
   {
      return jointTwist;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getJointAcceleration()
   {
      return jointAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public WrenchReadOnly getJointWrench()
   {
      return successorWrench;
   }

   /** {@inheritDoc} */
   @Override
   public List<TwistReadOnly> getUnitTwists()
   {
      return unitTwists;
   }

   @Override
   public String toString()
   {
      return super.toString() + ", orientation: " + jointOrientation + ", velocity" + EuclidCoreIOTools.getTuple3DString(jointAngularVelocity)
            + ", acceleration: " + EuclidCoreIOTools.getTuple3DString(jointAngularAcceleration) + ", torque: "
            + EuclidCoreIOTools.getTuple3DString(jointTorque);
   }
}
