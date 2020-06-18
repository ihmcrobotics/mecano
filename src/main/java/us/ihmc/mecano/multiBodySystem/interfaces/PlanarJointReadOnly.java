package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DMatrix;

/**
 * Read-only interface for planar joint.
 * <p>
 * A planar joint can be used to represent a 2D floating joint in the XZ-plane.
 * </p>
 *
 * @author Robert Griffin
 * @author Sylvain Bertrand
 */
public interface PlanarJointReadOnly extends FloatingJointReadOnly
{
   /** The number of DoFs, i.e. degrees of freedom, that a {@code PlanarJointReadOnly} has. */
   public static final int NUMBER_OF_DOFS = 3;

   /** {@inheritDoc} */
   @Override
   default int getJointAcceleration(int rowStart, DMatrix matrixToPack)
   {
      matrixToPack.set(rowStart + 0, 0, getJointAcceleration().getAngularPartY());
      matrixToPack.set(rowStart + 1, 0, getJointAcceleration().getLinearPartX());
      matrixToPack.set(rowStart + 2, 0, getJointAcceleration().getLinearPartZ());
      return rowStart + getConfigurationMatrixSize();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointTau(int rowStart, DMatrix matrixToPack)
   {
      matrixToPack.set(rowStart + 0, 0, getJointWrench().getAngularPartY());
      matrixToPack.set(rowStart + 1, 0, getJointWrench().getLinearPartX());
      matrixToPack.set(rowStart + 2, 0, getJointWrench().getLinearPartZ());
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointConfiguration(int rowStart, DMatrix matrixToPack)
   {
      int index = rowStart;
      matrixToPack.set(index++, 0, getJointPose().getPitch());
      matrixToPack.set(index++, 0, getJointPose().getX());
      matrixToPack.set(index++, 0, getJointPose().getZ());
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointVelocity(int rowStart, DMatrix matrixToPack)
   {
      matrixToPack.set(rowStart + 0, 0, getJointTwist().getAngularPartY());
      matrixToPack.set(rowStart + 1, 0, getJointTwist().getLinearPartX());
      matrixToPack.set(rowStart + 2, 0, getJointTwist().getLinearPartZ());
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
      return getDegreesOfFreedom();
   }
}
