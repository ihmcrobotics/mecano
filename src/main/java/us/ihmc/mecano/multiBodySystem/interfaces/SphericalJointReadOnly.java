package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Read-only interface for a spherical joint, i.e. 3 degrees of freedom of rotation.
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface SphericalJointReadOnly extends JointReadOnly
{
   /** The number of DoFs, i.e. degrees of freedom, that a {@code SphericalJointReadOnly} has. */
   public static final int NUMBER_OF_DOFS = 3;

   /**
    * Gets the read-only reference to this joint orientation.
    *
    * @return the read-only reference to this joint orientation.
    */
   QuaternionReadOnly getJointOrientation();

   /**
    * Gets the read-only reference to this joint angular velocity.
    *
    * @return the read-only reference to this joint angular velocity.
    */
   FrameVector3DReadOnly getJointAngularVelocity();

   /**
    * Gets the read-only reference to this joint angular acceleration.
    *
    * @return the read-only reference to this joint angular acceleration.
    */
   FrameVector3DReadOnly getJointAngularAcceleration();

   /**
    * Gets the read-only reference to this joint torque.
    *
    * @return the read-only reference to this joint torque.
    */
   FrameVector3DReadOnly getJointTorque();

   /** {@inheritDoc} */
   @Override
   default void getJointConfiguration(RigidBodyTransform jointTransform)
   {
      jointTransform.setRotationAndZeroTranslation(getJointOrientation());
   }

   /** {@inheritDoc} */
   @Override
   default int getJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      getJointOrientation().get(rowStart, matrix);
      return rowStart + getConfigurationMatrixSize();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointVelocity(int rowStart, DenseMatrix64F matrixToPack)
   {
      getJointTwist().getAngularPart().get(rowStart, matrixToPack);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointAcceleration(int rowStart, DenseMatrix64F matrixToPack)
   {
      getJointAcceleration().getAngularPart().get(rowStart, matrixToPack);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointTau(int rowStart, DenseMatrix64F matrixToPack)
   {
      getJointWrench().getAngularPart().get(rowStart, matrixToPack);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getDegreesOfFreedom()
   {
      return NUMBER_OF_DOFS;
   }

   /** {@inheritDoc} */
   @Override
   default int getConfigurationMatrixSize()
   {
      /*
       * This is because of the orientation have 3 DoFs but being represented with a quaternion that has 4
       * elements.
       */
      return NUMBER_OF_DOFS + 1;
   }
}
