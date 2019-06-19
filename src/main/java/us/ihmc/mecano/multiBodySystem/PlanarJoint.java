package us.ihmc.mecano.multiBodySystem;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * A {@code PlanarJoint} can be used to represent a 2D floating joint in the XZ-plane.
 *
 * @author Robert Griffin
 * @author Sylvain Bertrand
 */
public class PlanarJoint extends Joint implements PlanarJointBasics
{
   /** The 3D pose of this joint restricted in the XZ-plane. */
   private final Pose3DBasics jointPose = MecanoFactories.newPlanarPose3DBasics();
   /** The twist of this joint restricted in the XZ-plane. */
   private final FixedFrameTwistBasics jointTwist;
   /** The spatial acceleration of this joint restricted in the XZ-plane. */
   private final FixedFrameSpatialAccelerationBasics jointAcceleration;
   /** The wrench of this joint restricted in the XZ-plane. */
   private FixedFrameWrenchBasics jointWrench;
   /**
    * The list containing the unit-twist for each degree of freedom of this joint. In the current
    * framework, the unit-twists are calculated once at construction and remain constant.
    */
   private final List<TwistReadOnly> unitTwists;

   /**
    * Creates a new planar joint.
    * <p>
    * This constructor is typically used for creating a root floating joint.
    * </p>
    * 
    * @param name        the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    */
   public PlanarJoint(String name, RigidBodyBasics predecessor)
   {
      this(name, predecessor, null);
   }

   /**
    * Creates a new planar joint.
    * 
    * @param name              the name for the new joint.
    * @param predecessor       the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    */
   public PlanarJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, predecessor, transformToParent);

      jointTwist = MecanoFactories.newPlanarFixedFrameTwistBasics(afterJointFrame, beforeJointFrame, afterJointFrame);
      jointAcceleration = MecanoFactories.newPlanarFixedFrameSpatialAccelerationVectorBasics(afterJointFrame, beforeJointFrame, afterJointFrame);
      unitTwists = MecanoTools.computePlanarJointMotionSubspace(beforeJointFrame, afterJointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      jointWrench = MecanoFactories.newPlanarFixedFrameWrenchBasics(successorFrame, afterJointFrame);
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
}
