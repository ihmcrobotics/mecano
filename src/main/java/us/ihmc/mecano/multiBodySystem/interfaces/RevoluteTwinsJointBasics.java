package us.ihmc.mecano.multiBodySystem.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read and write interface for a revolute twins joint.
 * <p>
 * A revolute twins joint is a joint that has 1 degree of freedom of rotation and which has a variable center of rotation which changes with its configuration.
 * The joint is composed of two revolute joints that share the same axis of rotation, their angle is constrained such that overall the revolute twins has a
 * single degree of freedom. Example of a revolute twins joint:
 * <pre>
 *    root
 *      |
 *      |
 *      O A
 *      |
 *      O B
 *      |
 *      |
 * end-effector
 * </pre>
 * where A and B are both revolute joints around the same axis. It is assumed that only one joint in this sub-system composed of {A, B} is a torque source, this
 * is the actuated joint.
 * </p>
 */
public interface RevoluteTwinsJointBasics extends RevoluteTwinsJointReadOnly, OneDoFJointBasics
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
   default RigidBodyBasics getBodyAB()
   {
      return getJointA().getSuccessor();
   }

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
      getActuatedJoint().setTau(computeActuatedJointTau(tau));
   }

   /**
    * This method is ineffective for revolute twins joints.
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
    * This method is ineffective for revolute twins joints.
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
    * This method is ineffective for revolute twins joints.
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
    * This method is ineffective for revolute twins joints.
    */
   @Override
   default void setJointForce(Vector3DReadOnly jointForce)
   {
      // This joint type behaves more like a revolute joint.
   }
}
