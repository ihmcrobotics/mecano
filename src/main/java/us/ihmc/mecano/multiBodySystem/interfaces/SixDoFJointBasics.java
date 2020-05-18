package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Write and read interface for 6 degree of freedom joint.
 * <p>
 * A 6-DoF joint can be used to represent a 3D floating joint.
 * </p>
 *
 * @author Robert Griffin
 * @author Sylvain Bertrand
 */
public interface SixDoFJointBasics extends SixDoFJointReadOnly, FloatingJointBasics
{
   /** {@inheritDoc} */
   @Override
   default void setJointConfiguration(JointReadOnly other)
   {
      setJointConfiguration(MecanoTools.checkTypeAndCast(other, SixDoFJointReadOnly.class));
   }

   /**
    * Sets this joint configuration from the other joint.
    *
    * @param other the other to get the configuration from. Not modified.
    */
   default void setJointConfiguration(SixDoFJointReadOnly other)
   {
      setJointConfiguration(other.getJointPose());
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwist(JointReadOnly other)
   {
      setJointTwist(MecanoTools.checkTypeAndCast(other, SixDoFJointReadOnly.class));
   }

   /**
    * Sets this joint velocity from the other joint.
    *
    * @param other the other to get the velocity from. Not modified.
    */
   default void setJointTwist(SixDoFJointReadOnly other)
   {
      // Cast to frameless object so we don't perform frame checks which would automatically fail.
      Vector3DReadOnly otherAngularVelocity = other.getJointTwist().getAngularPart();
      Vector3DReadOnly otherLinearVelocity = other.getJointTwist().getLinearPart();

      setJointAngularVelocity(otherAngularVelocity);
      setJointLinearVelocity(otherLinearVelocity);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAcceleration(JointReadOnly other)
   {
      setJointAcceleration(MecanoTools.checkTypeAndCast(other, SixDoFJointReadOnly.class));
   }

   /**
    * Sets this joint acceleration from the other joint.
    *
    * @param other the other to get the acceleration from. Not modified.
    */
   default void setJointAcceleration(SixDoFJointReadOnly other)
   {
      // Cast to frameless object so we don't perform frame checks which would automatically fail.
      Vector3DReadOnly otherAngularAcceleration = other.getJointAcceleration().getAngularPart();
      Vector3DReadOnly otherLinearAcceleration = other.getJointAcceleration().getLinearPart();

      setJointAngularAcceleration(otherAngularAcceleration);
      setJointLinearAcceleration(otherLinearAcceleration);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointWrench(JointReadOnly other)
   {
      setJointWrench(MecanoTools.checkTypeAndCast(other, SixDoFJointReadOnly.class));
   }

   /**
    * Sets this joint force/torque from the other joint.
    *
    * @param other the other to get the force/torque from. Not modified.
    */
   default void setJointWrench(SixDoFJointReadOnly other)
   {
      // Cast to frameless object so we don't perform frame checks which would automatically fail.
      Vector3DReadOnly otherTorque = other.getJointWrench().getAngularPart();
      Vector3DReadOnly otherForce = other.getJointWrench().getLinearPart();

      setJointTorque(otherTorque);
      setJointForce(otherForce);
   }

   /** {@inheritDoc} */
   @Override
   default int setJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      getJointPose().getOrientation().set(rowStart, matrix);
      getJointPose().getPosition().set(rowStart + 4, matrix);
      return rowStart + getConfigurationMatrixSize();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      getJointTwist().set(rowStart, matrix);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      getJointAcceleration().set(rowStart, matrix);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointTau(int rowStart, DenseMatrix64F matrix)
   {
      getJointWrench().set(rowStart, matrix);
      return rowStart + getDegreesOfFreedom();
   }
}
