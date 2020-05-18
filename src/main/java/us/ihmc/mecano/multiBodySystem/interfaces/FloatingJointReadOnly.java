package us.ihmc.mecano.multiBodySystem.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Read-only interface for 3D and 2D floating joints.
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
 *
 * @author Sylvain Bertrand
 */
public interface FloatingJointReadOnly extends JointReadOnly
{
   /**
    * Gets the read-only reference to the pose of this floating joint, i.e. the pose of the multi-body
    * system in world.
    *
    * @return this joint pose.
    */
   Pose3DReadOnly getJointPose();

   /** {@inheritDoc} */
   @Override
   default void getJointConfiguration(RigidBodyTransform jointTransform)
   {
      jointTransform.set(getJointPose().getOrientation(), getJointPose().getPosition());
   }
}
