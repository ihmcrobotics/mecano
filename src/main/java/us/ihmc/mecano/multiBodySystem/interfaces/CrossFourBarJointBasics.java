package us.ihmc.mecano.multiBodySystem.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for a cross four bar joint.
 * <p>
 * A cross four bar joint is a joint that has 1 degree of freedom of rotation and which has a
 * variable center of rotation which changes with its configuration. Example of a cross four bar
 * joint:
 * 
 * <pre>
 *    root
 *      |
 *      |
 * A O-----O B
 *    \   /
 *     \ /
 *      X
 *     / \
 *    /   \
 * C O-----O D
 *      |
 * end-effector
 * </pre>
 * 
 * where A, B, C, and D are all revolute joint around the same axis. It is assumed that only one
 * joint in this sub-system composed of {A, B, C, and D} is a torque source, this is the actuated
 * joint.
 * </p>
 */
public interface CrossFourBarJointBasics extends CrossFourBarJointReadOnly, OneDoFJointBasics
{
   /** {@inheritDoc} */
   @Override
   RevoluteJointBasics getActuatedJoint();

   /** {@inheritDoc} */
   @Override
   RevoluteJointBasics getJointA();

   /** {@inheritDoc} */
   @Override
   RevoluteJointBasics getJointB();

   /** {@inheritDoc} */
   @Override
   RevoluteJointBasics getJointC();

   /** {@inheritDoc} */
   @Override
   RevoluteJointBasics getJointD();

   /** {@inheritDoc} */
   @Override
   void updateFrame();

   /** {@inheritDoc} */
   @Override
   void updateMotionSubspace();

   /** {@inheritDoc} */
   @Override
   default void setQ(double q)
   {
      getActuatedJoint().setQ(computeActuatedJointQ(q));
   }

   /** {@inheritDoc} */
   @Override
   default void setQd(double qd)
   {
      getActuatedJoint().setQd(computeActuatedJointQd(qd));
   }

   /** {@inheritDoc} */
   @Override
   default void setQdd(double qdd)
   {
      getActuatedJoint().setQdd(computeActuatedJointQdd(qdd));
   }

   /** {@inheritDoc} */
   @Override
   default void setTau(double tau)
   {
      getJointA().setJointTauToZero();
      getJointB().setJointTauToZero();
      getJointC().setJointTauToZero();
      getJointD().setJointTauToZero();
      getActuatedJoint().setTau(computeActuatedJointTau(tau));
   }

   /**
    * This method is ineffective for cross four joints.
    */
   @Override
   default void setJointPosition(Tuple3DReadOnly jointPosition)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   default void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
      setQd(jointAngularVelocity.dot(getJointAxis()));
   }

   /**
    * This method is ineffective for cross four joints.
    */
   @Override
   default void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity)
   {
      // This joint type behaves more like a revolute joint.
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
      setQdd(jointAngularAcceleration.dot(getJointAxis()));
   }

   /**
    * This method is ineffective for cross four joints.
    */
   @Override
   default void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration)
   {
      // This joint type behaves more like a revolute joint.
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTorque(Vector3DReadOnly jointTorque)
   {
      setTau(jointTorque.dot(getJointAxis()));
   }

   /**
    * This method is ineffective for cross four joints.
    */
   @Override
   default void setJointForce(Vector3DReadOnly jointForce)
   {
      // This joint type behaves more like a revolute joint.
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLimitLower(double jointLimitLower)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * This feature is not supported.
    * <p>
    * Joint limits are evaluated at construction given the most restrictive limits between the internal
    * joints and the limits due to the four bar kinematics.
    * </p>
    */
   @Override
   default void setJointLimitUpper(double jointLimitUpper)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * This feature is not supported.
    * <p>
    * The joint limits are evaluated at construction given the most restrictive limits between the
    * internal joints and the limits due to the four bar kinematics.
    * </p>
    */
   @Override
   default void setVelocityLimitLower(double velocityLimitLower)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * This feature is not supported.
    */
   @Override
   default void setVelocityLimitUpper(double velocityLimitUpper)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * This feature is not supported.
    */
   @Override
   default void setEffortLimitLower(double effortLimitLower)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * This feature is not supported.
    */
   @Override
   default void setEffortLimitUpper(double effortLimitUpper)
   {
      throw new UnsupportedOperationException();
   }
}
