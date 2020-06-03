package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DMatrix;

/**
 * Read-only interface for 6 degree of freedom joint.
 * <p>
 * A 6-DoF joint can be used to represent a 3D floating joint.
 * </p>
 *
 * @author Robert Griffin
 * @author Sylvain Bertrand
 */
public interface SixDoFJointReadOnly extends FloatingJointReadOnly
{
   /** The number of DoFs, i.e. degrees of freedom, that a {@code SixDoFJointReadOnly} has. */
   public static final int NUMBER_OF_DOFS = 6;

   /** {@inheritDoc} */
   @Override
   default int getJointConfiguration(int rowStart, DMatrix matrixToPack)
   {
      getJointPose().getOrientation().get(rowStart, matrixToPack);
      getJointPose().getPosition().get(rowStart + 4, matrixToPack);
      return rowStart + getConfigurationMatrixSize();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointVelocity(int rowStart, DMatrix matrixToPack)
   {
      getJointTwist().get(rowStart, matrixToPack);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointAcceleration(int rowStart, DMatrix matrixToPack)
   {
      getJointAcceleration().get(rowStart, matrixToPack);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointTau(int rowStart, DMatrix matrixToPack)
   {
      getJointWrench().get(rowStart, matrixToPack);
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
