package us.ihmc.mecano.multiBodySystem.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;

/**
 * Write and read interface for 3D and 2D floating joints.
 * <p>
 * A floating joint is commonly is used to represent a virtual non-actuated joint between a floating
 * multi-body system and the world. This allows to position and estimate the motion of the
 * multi-body system with respect to the world without adding control authority over the system.
 * </p>
 * <p>
 * A 2D floating can be used to constrain a floating multi-body system to evolve on a 2D plane
 * restraining its possible motion to 2 degrees of freedom of translation and 1 degree of freedom of
 * rotation. In this framework, the {@code PlanarJoint} can be used to represent a 2D floating joint
 * in the XZ-plane.
 * </p>
 * <p>
 * Although a floating joint is non-actuated, this interface allows to set a force and/or torque to
 * a floating joint. It is up to the user to enforce the non-actuation, i.e. force and torque set to
 * zero, of a floating joint when simulating a floating multi-body system.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FloatingJointBasics extends FloatingJointReadOnly, JointBasics
{
   /**
    * Gets the reference to the pose of this floating joint, i.e. the pose of the multi-body system in
    * world.
    * <p>
    * Note that for 2D floating joints, some components of the 3D pose cannot be set as set the motion
    * is constrained to a 2D plane.
    * </p>
    *
    * @return this joint pose.
    */
   @Override
   Pose3DBasics getJointPose();

   /**
    * Gets the reference to this joint twist (3D angular and linear velocities). The reference frames
    * of the joint twist can not be changed and are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * <p>
    * Note that for 2D floating joints, some components of the twist cannot be set as set the motion is
    * constrained to a 2D plane.
    * </p>
    *
    * @return the reference to this joint twist.
    */
   @Override
   FixedFrameTwistBasics getJointTwist();

   /**
    * Gets the reference to this joint spatial acceleration (3D angular and linear accelerations). The
    * reference frames of the joint acceleration can not be changed and are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * <p>
    * Note that for 2D floating joints, some components of the spatial acceleration cannot be set as
    * set the motion is constrained to a 2D plane.
    * </p>
    *
    * @return the reference to this joint spatial acceleration.
    */
   @Override
   FixedFrameSpatialAccelerationBasics getJointAcceleration();

   /**
    * Gets the reference to this joint wrench (3D force and torque). The reference frames of the joint
    * wrench can not be changed and are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * <p>
    * Note that for 2D floating joints, some components of the wrench cannot be set as the motion is
    * constrained to a 2D plane.
    * </p>
    *
    * @return the reference to this joint wrench.
    */
   @Override
   FixedFrameWrenchBasics getJointWrench();

   /** {@inheritDoc} */
   @Override
   default void setJointConfigurationToZero()
   {
      getJointPose().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwistToZero()
   {
      getJointTwist().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAccelerationToZero()
   {
      getJointAcceleration().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTauToZero()
   {
      getJointWrench().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
      getJointPose().getOrientation().set(jointOrientation);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointPosition(Tuple3DReadOnly jointTranslation)
   {
      getJointPose().getPosition().set(jointTranslation);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
      getJointTwist().getAngularPart().set(jointAngularVelocity);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity)
   {
      getJointTwist().getLinearPart().set(jointLinearVelocity);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
      getJointAcceleration().getAngularPart().set(jointAngularAcceleration);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration)
   {
      getJointAcceleration().getLinearPart().set(jointLinearAcceleration);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTorque(Vector3DReadOnly jointTorque)
   {
      getJointWrench().getAngularPart().set(jointTorque);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointForce(Vector3DReadOnly jointForce)
   {
      getJointWrench().getLinearPart().set(jointForce);
   }
}
