package us.ihmc.mecano.yoVariables.multiBodySystem;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.yoVariables.spatial.YoSpatialInertia;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * {@code YoRigidBody} describes a link which used with {@code Joints}s describe a multi-body system
 * (see also {@link us.ihmc.mecano.multiBodySystem.RigidBody}).
 * <p>
 * This class gathers notably the physical properties -- backing them with YoVariables -- of the link and its position in the system, i.e. its parent and
 * children joints.
 * </p>
 */
public class YoRigidBody implements RigidBodyBasics
{
   /** This is where the physical properties of this rigid-body are stored. */
   private final YoSpatialInertia spatialInertia;
   /**
    * This is a reference rigidly attached this rigid-body. It's usually centered at the rigid-body's
    * center of mass.
    */
   private final MovingReferenceFrame bodyFixedFrame;
   /**
    * The parent joint is the joint directly connected to a rigid-body and located between the
    * rigid-body and the robot's root.
    * <p>
    * The parent joint is equal to {@code null} when this rigid-body represents the root of the robot.
    * </p>
    */
   private final JointBasics parentJoint;
   /**
    * As for the {@link #parentJoint}, the parent loop closure joints are assumed to be directly
    * connected to this rigid-body. Note that this list is most of the time empty as it is populated
    * only for rigid-bodies located at a kinematic loop closure.
    */
   private final List<JointBasics> parentLoopClosureJoints = new ArrayList<>();
   /**
    * The children joints are all the joints that are directly connected to a rigid-body and located
    * between the rigid-body and any end-effector of the robot.
    * <p>
    * There is no children joint when this rigid-body is an end-effector.
    * </p>
    */
   private final List<JointBasics> childrenJoints = new ArrayList<>();
   /**
    * The name of this rigid-body. It is important that this name is unique among all the rigid-bodies
    * of a robot.
    */
   private final String name;
   /**
    * The identification name for this rigid-body. It is composed of this rigid-body name and all of
    * its ancestors.
    */
   private final String nameId;

   /**
    * Creates a new root rigid-body to which the first joint of a robot kinematic chain can be added.
    * <p>
    * Note that there is only one root body per robot.
    * </p>
    *
    * @param bodyName              the name for this rigid-body.
    * @param transformToParent     provides the pose of this rigid-body's body-fixed-frame with respect
    *                              to the {@code parentStationaryFrame}. Not modified.
    * @param parentStationaryFrame the parent stationary, i.e. non-moving with respect to world frame,
    *                              frame to which this rigid-body will create and attach its body fixed
    *                              frame. Most of the time
    *                              {@code parentStationaryFrame == ReferenceFrame.getWorldFrame()}.
    */
   public YoRigidBody(String bodyName, RigidBodyTransformReadOnly transformToParent, ReferenceFrame parentStationaryFrame)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      name = bodyName;
      spatialInertia = null;
      bodyFixedFrame = MovingReferenceFrame.constructFrameFixedInParent(bodyName + "Frame", parentStationaryFrame, transformToParent);
      parentJoint = null;
      nameId = bodyName;
   }
   /**
    * Creates a new rigid-body that is setup as the successor of the given {@code parentJoint}.
    *
    * @param bodyName        the name for this rigid-body.
    * @param parentJoint     the joint directly attached to this rigid-body and located between this rigid-body and the root body of the robot.
    * @param momentOfInertia the 3D moment of inertia of this rigid-body. Not modified.
    * @param mass            the mass of this rigid-body.
    * @param inertiaPose     defines the transform from this rigid-body body-fixed-frame to the {@code parentJoint.getFrameAfterJointFrame()}. The given moment
    *                        of
    *                        inertia is assumed to be expressed in that body-fixed-frame. Also note that the translation part corresponds to the position of
    *                        this
    *                        rigid body center of mass position expressed in {@code parentJoint.getFrameAfterJointFrame()}. Not modified.
    * @param registry        the registry to add the YoVariables to.
    */
   public YoRigidBody(String bodyName, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransformReadOnly inertiaPose, YoRegistry registry)
   {
      this(bodyName, parentJoint, inertiaPose, registry);
      spatialInertia.getMomentOfInertia().set(momentOfInertia);
      spatialInertia.setMass(mass);
   }

   private YoRigidBody(String bodyName, JointBasics parentJoint, RigidBodyTransformReadOnly inertiaPose, YoRegistry registry)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      name = bodyName;
      this.parentJoint = parentJoint;

      ReferenceFrame frameAfterJoint = parentJoint.getFrameAfterJoint();
      bodyFixedFrame = MovingReferenceFrame.constructFrameFixedInParent(bodyName + "CoM", frameAfterJoint, inertiaPose);
      spatialInertia = new YoSpatialInertia(bodyFixedFrame, bodyFixedFrame, registry);
      // inertia should be expressed in body frame, otherwise it will change
      spatialInertia.getBodyFrame().checkReferenceFrameMatch(spatialInertia.getReferenceFrame());
      parentJoint.setSuccessor(this);
      nameId = parentJoint.getPredecessor().getNameId() + NAME_ID_SEPARATOR + bodyName;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialInertiaBasics getInertia()
   {
      return spatialInertia;
   }

   /** {@inheritDoc} */
   @Override
   public MovingReferenceFrame getBodyFixedFrame()
   {
      return bodyFixedFrame;
   }

   /** {@inheritDoc} */
   @Override
   public JointBasics getParentJoint()
   {
      return parentJoint;
   }

   @Override
   public List<JointBasics> getParentLoopClosureJoints()
   {
      return parentLoopClosureJoints;
   }

   /** {@inheritDoc} */
   @Override
   public void addChildJoint(JointBasics joint)
   {
      childrenJoints.add(joint);
   }

   /** {@inheritDoc} */
   @Override
   public List<JointBasics> getChildrenJoints()
   {
      return childrenJoints;
   }

   /** {@inheritDoc} */
   @Override
   public String getName()
   {
      return name;
   }

   /** {@inheritDoc} */
   @Override
   public String getNameId()
   {
      return nameId;
   }

   /** {@inheritDoc} */
   @Override
   public YoRigidBody[] subtreeArray()
   {
      return subtreeStream().toArray(YoRigidBody[]::new);
   }

   /**
    * Returns the name of this rigid-body.
    * <p>
    * This should probably include more information such as the mass.
    * </p>
    */
   @Override
   public String toString()
   {
      return name;
   }

   /**
    * The hash code of a rigid-body is based on its {@link #getNameId()}.
    *
    * @return the hash code of the {@link #getNameId()} of this rigid-body.
    */
   @Override
   public int hashCode()
   {
      return nameId.hashCode();
   }
}
