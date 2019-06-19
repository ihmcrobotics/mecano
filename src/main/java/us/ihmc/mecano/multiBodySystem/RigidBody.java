package us.ihmc.mecano.multiBodySystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

/**
 * {@code RigidBody} describes a link which used with {@code Joint}s describe a multi-body system.
 * <p>
 * This class gathers notably the physical properties of the link and its position in the system,
 * i.e. its parent and children joints.
 * </p>
 */
public class RigidBody implements RigidBodyBasics
{
   /** This is where the physical properties of this rigid-body are stored. */
   private final SpatialInertia inertia;
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
    * The children joints are all the joints that are directly connected to a rigid-body and located
    * between the rigid-body and any end-effector of the robot.
    * <p>
    * There is no children joint when this rigid-body is an end-effector.
    * </p>
    */
   private final List<JointBasics> childrenJoints = new ArrayList<>();
   private final List<JointBasics> childrenJointsReadOnly = Collections.unmodifiableList(childrenJoints);
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
    * @param parentStationaryFrame the parent stationary, i.e. non-moving with respect to world frame,
    *                              frame to which this rigid-body will create and attach its body fixed
    *                              frame. Most of the time
    *                              {@code parentStationaryFrame == ReferenceFrame.getWorldFrame()}.
    */
   public RigidBody(String bodyName, ReferenceFrame parentStationaryFrame)
   {
      this(bodyName, new RigidBodyTransform(), parentStationaryFrame);
   }

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
   public RigidBody(String bodyName, RigidBodyTransformReadOnly transformToParent, ReferenceFrame parentStationaryFrame)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      name = bodyName;
      inertia = null;
      bodyFixedFrame = MovingReferenceFrame.constructFrameFixedInParent(bodyName + "Frame", parentStationaryFrame, transformToParent);
      parentJoint = null;
      nameId = bodyName;
   }

   /**
    * Creates a new rigid-body that is setup as the successor of the given {@code parentJoint}.
    * 
    * @param bodyName           the name for this rigid-body.
    * @param parentJoint        the joint directly attached to this rigid-body and located between this
    *                           rigid-body and the root body of the robot.
    * @param Ixx                the moment of inertia around the x-axis.
    * @param Iyy                the moment of inertia around the y-axis.
    * @param Izz                the moment of inertia around the z-axis.
    * @param mass               the mass of this rigid-body.
    * @param centerOfMassOffset the translation offset of the center of the mass with respect to the
    *                           frame after the parent joint. Not modified.
    */
   public RigidBody(String bodyName, JointBasics parentJoint, double Ixx, double Iyy, double Izz, double mass, Tuple3DReadOnly centerOfMassOffset)
   {
      this(bodyName, parentJoint, new RigidBodyTransform(new Quaternion(), centerOfMassOffset));
      inertia.setMomentOfInertia(Ixx, Iyy, Izz);
      inertia.setMass(mass);
   }

   /**
    * Creates a new rigid-body that is setup as the successor of the given {@code parentJoint}.
    *
    * @param bodyName           the name for this rigid-body.
    * @param parentJoint        the joint directly attached to this rigid-body and located between this
    *                           rigid-body and the root body of the robot.
    * @param momentOfInertia    the 3D moment of inertia of this rigid-body. Not modified.
    * @param mass               the mass of this rigid-body.
    * @param centerOfMassOffset the translation offset of the center of the mass with respect to the
    *                           frame after the parent joint. Not modified.
    */
   public RigidBody(String bodyName, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, Tuple3DReadOnly centerOfMassOffset)
   {
      this(bodyName, parentJoint, new RigidBodyTransform(new Quaternion(), centerOfMassOffset));
      inertia.getMomentOfInertia().set(momentOfInertia);
      inertia.setMass(mass);
   }

   /**
    * Creates a new rigid-body that is setup as the successor of the given {@code parentJoint}.
    *
    * @param bodyName        the name for this rigid-body.
    * @param parentJoint     the joint directly attached to this rigid-body and located between this
    *                        rigid-body and the root body of the robot.
    * @param momentOfInertia the 3D moment of inertia of this rigid-body. Not modified.
    * @param mass            the mass of this rigid-body.
    * @param inertiaPose     defines the transform from this rigid-body body-fixed-frame to the
    *                        {@code parentJoint.getFrameAfterJointFrame()}. The given moment of inertia
    *                        is assumed to be expressed in that body-fixed-frame. Also note that the
    *                        translation part corresponds to the position of this rigid-body center of
    *                        mass position expressed in {@code parentJoint.getFrameAfterJointFrame()}.
    *                        Not modified.
    */
   public RigidBody(String bodyName, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransformReadOnly inertiaPose)
   {
      this(bodyName, parentJoint, inertiaPose);
      inertia.getMomentOfInertia().set(momentOfInertia);
      inertia.setMass(mass);
   }

   private RigidBody(String bodyName, JointBasics parentJoint, RigidBodyTransformReadOnly inertiaPose)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      name = bodyName;
      this.parentJoint = parentJoint;

      ReferenceFrame frameAfterJoint = parentJoint.getFrameAfterJoint();
      bodyFixedFrame = MovingReferenceFrame.constructFrameFixedInParent(bodyName + "CoM", frameAfterJoint, inertiaPose);
      inertia = new SpatialInertia(bodyFixedFrame, bodyFixedFrame);
      // inertia should be expressed in body frame, otherwise it will change
      inertia.getBodyFrame().checkReferenceFrameMatch(inertia.getReferenceFrame());
      parentJoint.setSuccessor(this);
      nameId = parentJoint.getPredecessor().getNameId() + NAME_ID_SEPARATOR + bodyName;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialInertiaBasics getInertia()
   {
      return inertia;
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
      return childrenJointsReadOnly;
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
