package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Write and read interface for joints with a single degree of freedom (DoF).
 * <p>
 * The 1 DoF can either be a translation DoF, see {@link PrismaticJoint}, or a rotation DoF, see
 * {@link RevoluteJoint}.
 * </p>
 * <p>
 * A 1-DoF joint is usually actuated, has limits describing the range of motion and actuator
 * capabilities.
 * </p>
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface OneDoFJointBasics extends OneDoFJointReadOnly, JointBasics
{
   /**
    * Sets the current position/angle for this joint.
    *
    * @param q the new position/angle for this joint.
    */
   void setQ(double q);

   /**
    * Sets the current velocity for this joint.
    *
    * @param qd the new velocity for this joint.
    */
   void setQd(double qd);

   /**
    * Sets the current acceleration for this joint.
    *
    * @param qdd the new acceleration for this joint.
    */
   void setQdd(double qdd);

   /**
    * Sets the current force/torque for this joint.
    *
    * @param tau the new force/torque for this joint.
    */
   void setTau(double tau);

   /** {@inheritDoc} */
   @Override
   default void setJointConfigurationToZero()
   {
      setQ(0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwistToZero()
   {
      setQd(0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAccelerationToZero()
   {
      setQdd(0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTauToZero()
   {
      setTau(0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointConfiguration(JointReadOnly other)
   {
      setJointConfiguration(MecanoTools.checkTypeAndCast(other, OneDoFJointReadOnly.class));
   }

   /**
    * Sets this joint configuration from the other joint.
    * 
    * @param other the other to get the configuration from. Not modified.
    */
   default void setJointConfiguration(OneDoFJointReadOnly other)
   {
      setQ(other.getQ());
   }

   /** {@inheritDoc} */
   @Override
   default void setJointTwist(JointReadOnly other)
   {
      setJointTwist(MecanoTools.checkTypeAndCast(other, OneDoFJointReadOnly.class));
   }

   /**
    * Sets this joint velocity from the other joint.
    * 
    * @param other the other to get the velocity from. Not modified.
    */
   default void setJointTwist(OneDoFJointReadOnly other)
   {
      setQd(other.getQd());
   }

   /** {@inheritDoc} */
   @Override
   default void setJointAcceleration(JointReadOnly other)
   {
      setJointAcceleration(MecanoTools.checkTypeAndCast(other, OneDoFJointReadOnly.class));
   }

   /**
    * Sets this joint acceleration from the other joint.
    * 
    * @param other the other to get the acceleration from. Not modified.
    */
   default void setJointAcceleration(OneDoFJointReadOnly other)
   {
      setQdd(other.getQdd());
   }

   /** {@inheritDoc} */
   @Override
   default void setJointWrench(JointReadOnly other)
   {
      setJointWrench(MecanoTools.checkTypeAndCast(other, OneDoFJointReadOnly.class));
   }

   /**
    * Sets this joint force/torque from the other joint.
    * 
    * @param other the other to get the force/torque from. Not modified.
    */
   default void setJointWrench(OneDoFJointReadOnly other)
   {
      setTau(other.getTau());
   }

   /** {@inheritDoc} */
   @Override
   default int setJointConfiguration(int rowStart, DenseMatrix64F matrix)
   {
      setQ(matrix.get(rowStart, 0));
      return rowStart + getConfigurationMatrixSize();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointVelocity(int rowStart, DenseMatrix64F matrix)
   {
      setQd(matrix.get(rowStart, 0));
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointAcceleration(int rowStart, DenseMatrix64F matrix)
   {
      setQdd(matrix.get(rowStart + 0, 0));
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int setJointTau(int rowStart, DenseMatrix64F matrix)
   {
      setTau(matrix.get(rowStart, 0));
      return rowStart + getDegreesOfFreedom();
   }

   /**
    * Sets the lower limit of this joint range of motion:
    * 
    * <pre>
    * this.q &in; [jointLimitLower; jointLimitUpper]
    * </pre>
    * 
    * @param jointLimitLower the lower position/angle limit for this joint.
    */
   void setJointLimitLower(double jointLimitLower);

   /**
    * Sets the upper limit of this joint range of motion:
    * 
    * <pre>
    * this.q &in; [jointLimitLower; jointLimitUpper]
    * </pre>
    * 
    * @param jointLimitUpper the upper position/angle limit for this joint.
    */
   void setJointLimitUpper(double jointLimitUpper);

   /**
    * Sets the lower and upper limits of this joint range of motion:
    * 
    * <pre>
    * this.q &in; [jointLimitLower; jointLimitUpper]
    * </pre>
    * 
    * @param jointLimitLower the lower position/angle limit for this joint.
    * @param jointLimitUpper the upper position/angle limit for this joint.
    */
   default void setJointLimits(double jointLimitLower, double jointLimitUpper)
   {
      setJointLimitLower(jointLimitLower);
      setJointLimitUpper(jointLimitUpper);
   }

   /**
    * Sets the lower limit of this joint velocity:
    * 
    * <pre>
    * this.q &in; [jointLimitLower; jointLimitUpper]
    * </pre>
    * 
    * @param velocityLimitLower the lower velocity limit for this joint.
    */
   void setVelocityLimitLower(double velocityLimitLower);

   /**
    * Sets the upper limit of this joint velocity:
    * 
    * <pre>
    * this.q &in; [jointLimitLower; jointLimitUpper]
    * </pre>
    * 
    * @param velocityLimitUpper the upper velocity limit for this joint.
    */
   void setVelocityLimitUpper(double velocityLimitUpper);

   /**
    * Sets the lower and upper limits of this joint velocity:
    * 
    * <pre>
    * this.q &in; [jointLimitLower; jointLimitUpper]
    * </pre>
    * 
    * @param velocityLimitLower the lower velocity limit for this joint.
    * @param velocityLimitUpper the upper velocity limit for this joint.
    */
   default void setVelocityLimits(double velocityLimitLower, double velocityLimitUpper)
   {
      setVelocityLimitLower(velocityLimitLower);
      setVelocityLimitUpper(velocityLimitUpper);
   }

   /**
    * Sets the absolute limit of this joint velocity:
    * 
    * <pre>
    * this.qd &in; [-velocityLimit; velocityLimit]
    * </pre>
    * 
    * @param velocityLimit the maximum absolute velocity this joint can have.
    */
   default void setVelocityLimit(double velocityLimit)
   {
      setVelocityLimits(-velocityLimit, velocityLimit);
   }

   /**
    * Sets the lower limit of this joint force/torque:
    * 
    * <pre>
    * this.tau &in; [effortLimitLower; effortLimitUpper]
    * </pre>
    * 
    * @param effortLimitLower the lower force/torque limit for this joint.
    */
   void setEffortLimitLower(double effortLimitLower);

   /**
    * Sets the upper limit of this joint force/torque:
    * 
    * <pre>
    * this.tau &in; [effortLimitLower; effortLimitUpper]
    * </pre>
    * 
    * @param effortLimitUpper the upper force/torque limit for this joint.
    */
   void setEffortLimitUpper(double effortLimitUpper);

   /**
    * Sets the lower and upper limits of this joint force/torque:
    * 
    * <pre>
    * this.tau &in; [effortLimitLower; effortLimitUpper]
    * </pre>
    * 
    * @param effortLimitLower the lower force/torque limit for this joint.
    * @param effortLimitUpper the upper force/torque limit for this joint.
    */
   default void setEffortLimits(double effortLimitLower, double effortLimitUpper)
   {
      setEffortLimitLower(effortLimitLower);
      setEffortLimitUpper(effortLimitUpper);
   }

   /**
    * Sets the absolute limit of this joint force/torque:
    * 
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * 
    * @param effortLimit the maximum absolute force/torque this joint can have.
    */
   default void setEffortLimit(double effortLimit)
   {
      setEffortLimits(-effortLimit, effortLimit);
   }
}
