package us.ihmc.mecano.multiBodySystem.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a revolute joint.
 * <p>
 * A revolute joint is a joint has 1 degree of freedom of rotation.
 * </p>
 * 
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface RevoluteJointBasics extends RevoluteJointReadOnly, OneDoFJointBasics
{
   /**
    * This method is ineffective for revolute joints.
    */
   @Override
   default void setJointPosition(Tuple3DReadOnly jointPosition)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
      setQd(getJointAxis().dot(jointAngularVelocity));
   }

   /**
    * This method is ineffective for revolute joints.
    */
   @Override
   default void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
      setQdd(getJointAxis().dot(jointAngularAcceleration));
   }

   /**
    * This method is ineffective for revolute joints.
    */
   @Override
   default void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTorque(Vector3DReadOnly jointTorque)
   {
      setTau(getJointAxis().dot(jointTorque));
   }

   /**
    * This method is ineffective for revolute joints.
    */
   @Override
   default void setJointForce(Vector3DReadOnly jointForce)
   {
   }
}
