package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Write and read interface for a spherical joint, i.e. 3 degrees of freedom of rotation.
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface SphericalJointBasics extends SphericalJointReadOnly, JointBasics
{
   /**
    * Gets the reference to this joint orientation.
    *
    * @return the reference to this joint orientation.
    */
   @Override
   QuaternionBasics getJointOrientation();

   /**
    * Gets the reference to this joint angular velocity.
    *
    * @return the reference to this joint angular velocity.
    */
   @Override
   FixedFrameVector3DBasics getJointAngularVelocity();

   /**
    * Gets the reference to this joint angular acceleration.
    *
    * @return the reference to this joint angular acceleration.
    */
   @Override
   FixedFrameVector3DBasics getJointAngularAcceleration();

   /**
    * Gets the reference to this joint torque.
    *
    * @return the reference to this joint torque.
    */
   @Override
   FixedFrameVector3DBasics getJointTorque();

   /** {@inheritDoc} */
   @Override
   default void setJointConfigurationToZero()
   {
      getJointOrientation().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwistToZero()
   {
      getJointAngularVelocity().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAccelerationToZero()
   {
      getJointAngularAcceleration().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTauToZero()
   {
      getJointTorque().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointConfiguration(JointReadOnly other)
   {
      setJointConfiguration(MecanoTools.checkTypeAndCast(other, SphericalJointReadOnly.class));
   }

   /**
    * Sets this joint configuration from the other joint.
    * 
    * @param other the other to get the configuration from. Not modified.
    */
   default void setJointConfiguration(SphericalJointReadOnly other)
   {
      getJointOrientation().set(other.getJointOrientation());
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwist(JointReadOnly other)
   {
      setJointTwist(MecanoTools.checkTypeAndCast(other, SphericalJointReadOnly.class));
   }

   /**
    * Sets this joint velocity from the other joint.
    * 
    * @param other the other to get the velocity from. Not modified.
    */
   default void setJointTwist(SphericalJointReadOnly other)
   {
      // Cast to frameless object so we don't perform frame checks which would automatically fail.
      Vector3DReadOnly otherAngularVelocity = other.getJointAngularVelocity();
      setJointAngularVelocity(otherAngularVelocity);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAcceleration(JointReadOnly other)
   {
      setJointAcceleration(MecanoTools.checkTypeAndCast(other, SphericalJointReadOnly.class));
   }

   /**
    * Sets this joint acceleration from the other joint.
    * 
    * @param other the other to get the acceleration from. Not modified.
    */
   default void setJointAcceleration(SphericalJointReadOnly other)
   {
      // Cast to frameless object so we don't perform frame checks which would automatically fail.
      Vector3DReadOnly otherAngularAcceleration = other.getJointAngularAcceleration();
      setJointAngularAcceleration(otherAngularAcceleration);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointWrench(JointReadOnly other)
   {
      setJointWrench(MecanoTools.checkTypeAndCast(other, SphericalJointReadOnly.class));
   }

   /**
    * Sets this joint force/torque from the other joint.
    * 
    * @param other the other to get the force/torque from. Not modified.
    */
   default void setJointWrench(SphericalJointReadOnly other)
   {
      // Cast to frameless object so we don't perform frame checks which would automatically fail.
      Vector3DReadOnly otherTorque = other.getJointTorque();
      setJointTorque(otherTorque);
   }

   /** {@inheritDoc} */
   @Override
   default int setJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      getJointOrientation().set(rowStart, matrix);
      return rowStart + getConfigurationMatrixSize();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      getJointAngularVelocity().set(rowStart, matrix);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      getJointAngularAcceleration().set(rowStart, matrix);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointTau(int rowStart, DenseMatrix64F matrix)
   {
      getJointTorque().set(rowStart, matrix);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
      getJointOrientation().set(jointOrientation);
   }

   /**
    * This method is ineffective for spherical joints.
    */
   @Override
   default void setJointPosition(Tuple3DReadOnly jointPosition)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
      getJointAngularVelocity().set(jointAngularVelocity);
   }

   /**
    * This method is ineffective for spherical joints.
    */
   @Override
   default void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
      getJointAngularAcceleration().set(jointAngularAcceleration);
   }

   /**
    * This method is ineffective for spherical joints.
    */
   @Override
   default void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTorque(Vector3DReadOnly jointTorque)
   {
      getJointTorque().set(jointTorque);
   }

   /**
    * This method is ineffective for spherical joints.
    */
   @Override
   default void setJointForce(Vector3DReadOnly jointForce)
   {
   }
}
