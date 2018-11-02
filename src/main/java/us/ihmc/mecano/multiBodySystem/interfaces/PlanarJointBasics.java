package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Write and read interface for planar joint.
 * <p>
 * A planar joint can be used to represent a 2D floating joint in the XZ-plane.
 * </p>
 *
 * @author Robert Griffin
 * @author Sylvain Bertrand
 */
public interface PlanarJointBasics extends PlanarJointReadOnly, FloatingJointBasics
{
   /** {@inheritDoc} */
   @Override
   default void setJointConfiguration(JointReadOnly other)
   {
      setJointConfiguration(MecanoTools.checkTypeAndCast(other, PlanarJointReadOnly.class));
   }

   /**
    * Sets this joint configuration from the other joint.
    * 
    * @param other the other to get the configuration from. Not modified.
    */
   default void setJointConfiguration(PlanarJointReadOnly other)
   {
      setJointConfiguration(other.getJointPose());
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwist(JointReadOnly other)
   {
      setJointTwist(MecanoTools.checkTypeAndCast(other, PlanarJointReadOnly.class));
   }

   /**
    * Sets this joint velocity from the other joint.
    * 
    * @param other the other to get the velocity from. Not modified.
    */
   default void setJointTwist(PlanarJointReadOnly other)
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
      setJointAcceleration(MecanoTools.checkTypeAndCast(other, PlanarJointReadOnly.class));
   }

   /**
    * Sets this joint acceleration from the other joint.
    * 
    * @param other the other to get the acceleration from. Not modified.
    */
   default void setJointAcceleration(PlanarJointReadOnly other)
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
      setJointWrench(MecanoTools.checkTypeAndCast(other, PlanarJointReadOnly.class));
   }

   /**
    * Sets this joint force/torque from the other joint.
    * 
    * @param other the other to get the force/torque from. Not modified.
    */
   default void setJointWrench(PlanarJointReadOnly other)
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
      int index = rowStart;
      double qRot = matrix.get(index++, 0);
      double x = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      getJointPose().getOrientation().setToPitchQuaternion(qRot);
      getJointPose().getPosition().set(x, 0.0, z);
      return rowStart + getConfigurationMatrixSize();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      int index = rowStart;
      double qdRot = matrix.get(index++, 0);
      double xd = matrix.get(index++, 0);
      double zd = matrix.get(index++, 0);
      getJointTwist().setToZero();
      getJointTwist().setAngularPartY(qdRot);
      getJointTwist().getLinearPart().set(xd, 0.0, zd);
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      getJointAcceleration().setToZero();
      getJointAcceleration().setAngularPartY(matrix.get(rowStart + 0, 0));
      getJointAcceleration().setLinearPartX(matrix.get(rowStart + 1, 0));
      getJointAcceleration().setLinearPartZ(matrix.get(rowStart + 2, 0));
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointTau(int rowStart, DenseMatrix64F matrix)
   {
      getJointWrench().setToZero();
      getJointWrench().setAngularPartY(matrix.get(rowStart + 0));
      getJointWrench().setLinearPartX(matrix.get(rowStart + 1));
      getJointWrench().setLinearPartZ(matrix.get(rowStart + 2));
      return rowStart + getDegreesOfFreedom();
   }
}
