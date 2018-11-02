package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

/**
 * Read-only interface for joints with a single degree of freedom (DoF).
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
public interface OneDoFJointReadOnly extends JointReadOnly
{
   /** The number of DoFs, i.e. degrees of freedom, that a {@code OneDoFJointReadOnly} has. */
   public static final int NUMBER_OF_DOFS = 1;

   /**
    * Gets the read-only reference to the axis of this joint.
    * <p>
    * This joint either translates along or rotates around its joint axis.
    * </p>
    * 
    * @return the read-only reference to this joint axis.
    */
   FrameVector3DReadOnly getJointAxis();

   /**
    * Gets the current position/angle of this joint.
    * 
    * @return this joint current position/angle.
    */
   double getQ();

   /**
    * Gets the current velocity of this joint.
    * 
    * @return this joint current velocity.
    */
   double getQd();

   /**
    * Gets the current acceleration of this joint.
    * 
    * @return this joint current acceleration.
    */
   double getQdd();

   /**
    * Gets the current force/torque of this joint.
    * 
    * @return this joint force/torque.
    */
   double getTau();

   /**
    * Gets the lower limit of this joint range of motion:
    * 
    * <pre>
    * this.q &in; [this.jointLimitLower; this.jointLimitUpper]
    * </pre>
    * 
    * @return the lower position/angle limit for this joint.
    */
   double getJointLimitLower();

   /**
    * Gets the upper limit of this joint range of motion:
    * 
    * <pre>
    * this.q &in; [this.jointLimitLower; this.jointLimitUpper]
    * </pre>
    * 
    * @return the upper position/angle limit for this joint.
    */
   double getJointLimitUpper();

   /**
    * Gets the lower limit of this joint velocity:
    * 
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * 
    * @return the lower velocity limit for this joint.
    */
   double getVelocityLimitLower();

   /**
    * Gets the upper limit of this joint velocity:
    * 
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * 
    * @return the upper velocity limit for this joint.
    */
   double getVelocityLimitUpper();

   /**
    * Gets the lower limit of this joint force/torque:
    * 
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * 
    * @return the lower force/torque limit for this joint.
    */
   double getEffortLimitLower();

   /**
    * Gets the upper limit of this joint force/torque:
    * 
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * 
    * @return the upper force/torque limit for this joint.
    */
   double getEffortLimitUpper();

   /**
    * Gets the read-only reference to this joint unitary twist.
    * <p>
    * The unit twist can be interpreted as the 6D joint axis:
    * <ul>
    * <li>for a {@code RevoluteJoint}:
    * <tt>[this.jointAxis<sup>T</sup> 0<sub>3x1</sub><sup>T</sup>]<sup>T</sup></tt>
    * <li>for a {@code PrismaticJoint}:
    * <tt>[0<sub>3x1</sub><sup>T</sup> this.jointAxis<sup>T</sup>]<sup>T</sup></tt>
    * </ul>
    * The unit-twist of a common 1-DoF joint is invariant and is most often for building Geometric
    * Jacobians or getting the joint velocity as a twist.
    * </p>
    * <p>
    * The unit-twist has the following frames:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * </p>
    * 
    * @return the read-only reference to this joint unit-twist.
    */
   TwistReadOnly getUnitJointTwist();

   /**
    * Gets the read-only reference to this joint's unitary twist.
    * <p>
    * This is the same unit-twist as {@link #getUnitJointTwist()} but expressed in different frames:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    * </p>
    * 
    * @return the read-only reference to this joint unit-twist.
    */
   TwistReadOnly getUnitSuccessorTwist();

   /**
    * Gets the read-only reference to this joint's unitary twist.
    * <p>
    * This is the same unit-twist as {@link #getUnitJointTwist()} but expressed in different frames:
    * <ul>
    * <li>{@code bodyFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code predecessorFrame}.
    * </ul>
    * </p>
    * 
    * @return the read-only reference to this joint unit-twist.
    */
   TwistReadOnly getUnitPredecessorTwist();

   /**
    * Gets the read-only reference to this joint unitary spatial acceleration.
    * <p>
    * The unit spatial acceleration can be interpreted as the 6D joint axis:
    * <ul>
    * <li>for a {@code RevoluteJoint}:
    * <tt>[this.jointAxis<sup>T</sup> 0<sub>3x1</sub><sup>T</sup>]<sup>T</sup></tt>
    * <li>for a {@code PrismaticJoint}:
    * <tt>[0<sub>3x1</sub><sup>T</sup> this.jointAxis<sup>T</sup>]<sup>T</sup></tt>
    * </ul>
    * The unit-twist of a common 1-DoF joint is invariant and is most often for building Geometric
    * Jacobians or getting the joint velocity as a twist.
    * </p>
    * <p>
    * The unit spatial acceleration has the following frames:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * </p>
    * 
    * @return the read-only reference to this joint unit spatial acceleration.
    */
   SpatialAccelerationReadOnly getUnitJointAcceleration();

   /**
    * Gets the read-only reference to this joint's unitary spatial acceleration.
    * <p>
    * This is the same unit spatial acceleration as {@link #getUnitJointAcceleration()} but
    * expressed in different frames:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    * </p>
    * 
    * @return the read-only reference to this joint unit spatial acceleration.
    */
   SpatialAccelerationReadOnly getUnitSuccessorAcceleration();

   /**
    * Gets the read-only reference to this joint's unitary spatial acceleration.
    * <p>
    * This is the same unit spatial acceleration as {@link #getUnitJointAcceleration()} but
    * expressed in different frames:
    * <ul>
    * <li>{@code bodyFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code predecessorFrame}.
    * </ul>
    * </p>
    * 
    * @return the read-only reference to this joint unit spatial acceleration.
    */
   SpatialAccelerationReadOnly getUnitPredecessorAcceleration();

   /** {@inheritDoc} */
   @Override
   default void getSuccessorTwist(TwistBasics twistToPack)
   {
      twistToPack.setIncludingFrame(getUnitSuccessorTwist());
      twistToPack.scale(getQd());
   }

   /** {@inheritDoc} */
   @Override
   default void getPredecessorTwist(TwistBasics twistToPack)
   {
      twistToPack.setIncludingFrame(getUnitPredecessorTwist());
      twistToPack.scale(getQd());
   }

   /** {@inheritDoc} */
   @Override
   default void getSuccessorAcceleration(SpatialAccelerationBasics accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(getUnitSuccessorAcceleration());
      accelerationToPack.scale(getQdd());
   }

   /** {@inheritDoc} */
   @Override
   default void getPredecessorAcceleration(SpatialAccelerationBasics accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(getUnitPredecessorAcceleration());
      accelerationToPack.scale(getQdd());
   }

   /** {@inheritDoc} */
   @Override
   default int getJointConfiguration(int rowStart, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(rowStart, getQ());
      return rowStart + getConfigurationMatrixSize();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointVelocity(int rowStart, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(rowStart, 0, getQd());
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointAcceleration(int rowStart, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(rowStart, 0, getQdd());
      return rowStart + getDegreesOfFreedom();
   }

   /** {@inheritDoc} */
   @Override
   default int getJointTau(int rowStart, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(rowStart, 0, getTau());
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
