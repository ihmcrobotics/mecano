package us.ihmc.mecano.yoVariables.multiBodySystem;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.yoVariables.tools.YoMecanoFactories;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * A {@code PlanarJoint} can be used to represent a 2D floating joint in the XZ-plane and has its
 * state backed by {@code YoVariable}s.
 *
 * @author Sylvain Bertrand
 */
public class YoPlanarJoint extends Joint implements PlanarJointBasics
{
   /** Registry in which the {@code YoVariable}s created for this joint are attached to. */
   private final YoRegistry registry;
   /** The 3D pose of this joint restricted in the XZ-plane. */
   private final Pose3DBasics jointPose;
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

   private final String varName;

   /**
    * Creates a new planar joint.
    * <p>
    * This constructor is typically used for creating a root floating joint.
    * </p>
    *
    * @param name        the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param registry    the registry to register child variables to.
    */
   public YoPlanarJoint(String name, RigidBodyBasics predecessor, YoRegistry registry)
   {
      this(name, predecessor, null, registry);
   }

   /**
    * Creates a new planar joint.
    *
    * @param name              the name for the new joint.
    * @param predecessor       the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    * @param registry          the registry to register child variables to.
    */
   public YoPlanarJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent, YoRegistry registry)
   {
      super(name, predecessor, transformToParent);
      this.registry = registry;

      varName = !name.isEmpty() ? "_" + name + "_" : "_";

      YoDouble x = new YoDouble("q" + varName + "x", registry);
      YoDouble z = new YoDouble("q" + varName + "z", registry);
      YoDouble pitch = new YoDouble("q" + varName + "pitch", registry);
      jointPose = YoMecanoFactories.newPlanarYoPose3DBasics(pitch, x, z);
      YoDouble angularVelocityY = new YoDouble("qd" + varName + "wy", registry);
      YoDouble linearVelocityX = new YoDouble("qd" + varName + "x", registry);
      YoDouble linearVelocityZ = new YoDouble("qd" + varName + "z", registry);
      jointTwist = YoMecanoFactories.newPlanarYoFixedFrameTwistBasics(angularVelocityY,
                                                                      linearVelocityX,
                                                                      linearVelocityZ,
                                                                      afterJointFrame,
                                                                      beforeJointFrame,
                                                                      afterJointFrame);
      YoDouble angularAccelerationY = new YoDouble("qdd" + varName + "wy", registry);
      YoDouble linearAccelerationX = new YoDouble("qdd" + varName + "x", registry);
      YoDouble linearAccelerationZ = new YoDouble("qdd" + varName + "z", registry);
      jointAcceleration = YoMecanoFactories.newPlanarYoFixedFrameSpatialAccelerationVectorBasics(angularAccelerationY,
                                                                                                 linearAccelerationX,
                                                                                                 linearAccelerationZ,
                                                                                                 afterJointFrame,
                                                                                                 beforeJointFrame,
                                                                                                 afterJointFrame);
      unitTwists = MecanoTools.computePlanarJointMotionSubspace(beforeJointFrame, afterJointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      YoDouble torqueY = new YoDouble("tau" + varName + "wy", registry);
      YoDouble forceX = new YoDouble("tau" + varName + "x", registry);
      YoDouble forceZ = new YoDouble("tau" + varName + "z", registry);
      jointWrench = YoMecanoFactories.newPlanarYoFixedFrameWrenchBasics(torqueY, forceX, forceZ, successorFrame, afterJointFrame);
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
