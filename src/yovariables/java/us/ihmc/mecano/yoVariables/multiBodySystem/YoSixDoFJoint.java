package us.ihmc.mecano.yoVariables.multiBodySystem;

import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialAcceleration;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * A {@code SixDoFJoint} can be used to represent a 3D floating joint and has its state backed by
 * {@code YoVariable}s.
 *
 * @author Sylvain Bertrand
 */
public class YoSixDoFJoint extends Joint implements SixDoFJointBasics
{
   /** Registry in which the {@code YoVariable}s created for this joint are attached to. */
   private final YoRegistry registry;

   /** The 3D pose of this joint. */
   private final YoFramePose3D jointPose;
   /** The twist of this joint. */
   private final YoFixedFrameTwist jointTwist;
   /** The spatial acceleration of this joint. */
   private final YoFixedFrameSpatialAcceleration jointAcceleration;
   /** The wrench of this joint. */
   private YoFixedFrameWrench jointWrench;
   /**
    * The list containing the unit-twist for each degree of freedom of this joint. In the current
    * framework, the unit-twists are calculated once at construction and remain constant.
    */
   private final List<TwistReadOnly> unitTwists;

   /**
    * Creates a new 6-DoF joint which state is backed by {@code YoVariable}s.
    * <p>
    * This constructor is typically used for creating a root floating joint, i.e. the first joint of
    * the multi-body system.
    * </p>
    *
    * @param name        the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param registry    the registry to register child variables to.
    */
   public YoSixDoFJoint(String name, RigidBodyBasics predecessor, YoRegistry registry)
   {
      this(name, predecessor, null, registry);
   }

   /**
    * Creates a new 6-DoF joint which state is backed by {@code YoVariable}s.
    *
    * @param name              the name for the new joint.
    * @param predecessor       the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    * @param registry          the registry to register child variables to.
    */
   public YoSixDoFJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent, YoRegistry registry)
   {
      super(name, predecessor, transformToParent);
      this.registry = registry;
      jointPose = new YoFramePose3D(name, beforeJointFrame, registry);
      jointTwist = new YoFixedFrameTwist(name + "Twist", afterJointFrame, beforeJointFrame, afterJointFrame, registry);
      jointAcceleration = new YoFixedFrameSpatialAcceleration(name + "Acceleration", afterJointFrame, beforeJointFrame, afterJointFrame, registry);
      unitTwists = MecanoTools.computeSixDoFJointMotionSubspace(beforeJointFrame, afterJointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      if (jointWrench == null)
         jointWrench = new YoFixedFrameWrench(name + "Wrench", successorFrame, afterJointFrame, registry);
      else
         jointWrench.checkBodyFrameMatch(successorFrame);
   }

   /** {@inheritDoc} */
   @Override
   public YoFramePose3D getJointPose()
   {
      return jointPose;
   }

   /** {@inheritDoc} */
   @Override
   public YoFixedFrameTwist getJointTwist()
   {
      return jointTwist;
   }

   /** {@inheritDoc} */
   @Override
   public YoFixedFrameSpatialAcceleration getJointAcceleration()
   {
      return jointAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public YoFixedFrameWrench getJointWrench()
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
