package us.ihmc.mecano.multiBodySystem;

import java.util.List;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * A {@code SixDoFJoint} can be used to represent a 3D floating joint.
 *
 * @author Robert Griffin
 * @author Sylvain Bertrand
 */
public class SixDoFJoint extends Joint implements SixDoFJointBasics
{
   /** The 3D pose of this joint. */
   private final Pose3D jointPose = new Pose3D();
   /** The twist of this joint. */
   private final FixedFrameTwistBasics jointTwist;
   /** The spatial acceleration of this joint. */
   private final FixedFrameSpatialAccelerationBasics jointAcceleration;
   /** The wrench of this joint. */
   private FixedFrameWrenchBasics jointWrench;
   /**
    * The list containing the unit-twist for each degree of freedom of this joint. In the current
    * framework, the unit-twists are calculated once at construction and remain constant.
    */
   private final List<TwistReadOnly> unitTwists;

   /**
    * Creates a new 6-DoF joint.
    * <p>
    * This constructor is typically used for creating a root floating joint, i.e. the first joint of
    * the multi-body system.
    * </p>
    * 
    * @param name the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    */
   public SixDoFJoint(String name, RigidBodyBasics predecessor)
   {
      this(name, predecessor, null);
   }

   /**
    * Creates a new 6-DoF joint.
    * 
    * @param name the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    */
   public SixDoFJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, predecessor, transformToParent);
      jointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAcceleration = new SpatialAcceleration(afterJointFrame, beforeJointFrame, afterJointFrame);
      unitTwists = MecanoTools.computeSixDoFJointMotionSubspace(beforeJointFrame, afterJointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      jointWrench = new Wrench(successorFrame, afterJointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public Pose3DBasics getJointPose()
   {
      return jointPose;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameTwistBasics getJointTwist()
   {
      return jointTwist;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameSpatialAccelerationBasics getJointAcceleration()
   {
      return jointAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameWrenchBasics getJointWrench()
   {
      return jointWrench;
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
      return super.toString() + ", configuration: " + jointPose + ", velocity: " + jointTwist + ", acceleration: " + jointAcceleration + ", wrench"
            + jointWrench;
   }
}
