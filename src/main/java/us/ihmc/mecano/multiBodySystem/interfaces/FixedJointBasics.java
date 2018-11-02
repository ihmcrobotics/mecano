package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Write and read interface for a fixed joint, it has 0 degrees of freedom, i.e. it does not move.
 *
 * @author Sylvain Bertrand
 */
public interface FixedJointBasics extends JointBasics, FixedJointReadOnly
{
   /** {@inheritDoc} */
   @Override
   default void setJointConfigurationToZero()
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwistToZero()
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAccelerationToZero()
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTauToZero()
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointConfiguration(JointReadOnly other)
   {
      MecanoTools.checkTypeAndCast(other, FixedJointReadOnly.class);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwist(JointReadOnly other)
   {
      MecanoTools.checkTypeAndCast(other, FixedJointReadOnly.class);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAcceleration(JointReadOnly other)
   {
      MecanoTools.checkTypeAndCast(other, FixedJointReadOnly.class);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointWrench(JointReadOnly other)
   {
      MecanoTools.checkTypeAndCast(other, FixedJointReadOnly.class);
   }

   /** {@inheritDoc} */
   @Override
   default int setJointConfiguration(int rowStart, DenseMatrix64F jointConfiguration)
   {
      return rowStart;
   }

   /** {@inheritDoc} */
   @Override
   default int setJointVelocity(int rowStart, DenseMatrix64F jointVelocity)
   {
      return rowStart;
   }

   /** {@inheritDoc} */
   @Override
   default int setJointAcceleration(int rowStart, DenseMatrix64F jointAcceleration)
   {
      return rowStart;
   }

   /** {@inheritDoc} */
   @Override
   default int setJointTau(int rowStart, DenseMatrix64F jointTau)
   {
      return rowStart;
   }

   /** {@inheritDoc} */
   @Override
   default void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointPosition(Tuple3DReadOnly jointPosition)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointForce(Vector3DReadOnly jointForce)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTorque(Vector3DReadOnly jointTorque)
   {
   }
}
