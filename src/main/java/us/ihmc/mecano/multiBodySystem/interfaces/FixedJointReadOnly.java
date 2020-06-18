package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Read-only interface for a fixed joint, it has 0 degrees of freedom, i.e. it does not move.
 *
 * @author Sylvain Bertrand
 */
public interface FixedJointReadOnly extends JointReadOnly
{
   /** The number of DoFs, i.e. degrees of freedom, that a {@code FixedJointReadOnly} has. */
   public static final int NUMBER_OF_DOFS = 0;

   /** {@inheritDoc} */
   @Override
   default void getJointConfiguration(RigidBodyTransform jointConfigurationToPack)
   {
      jointConfigurationToPack.setIdentity();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointConfiguration(int rowStart, DMatrix matrixToPack)
   {
      return rowStart;
   }

   /** {@inheritDoc} */
   @Override
   default int getJointVelocity(int rowStart, DMatrix matrixToPack)
   {
      return rowStart;
   }

   /** {@inheritDoc} */
   @Override
   default int getJointAcceleration(int rowStart, DMatrix matrixToPack)
   {
      return rowStart;
   }

   /** {@inheritDoc} */
   @Override
   default int getJointTau(int rowStart, DMatrix matrixToPack)
   {
      return rowStart;
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
      return NUMBER_OF_DOFS;
   }
}
