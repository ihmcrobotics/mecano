package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DMatrix;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

/**
 * Read-only interface for a revolute twins joint.
 * <p>
 * A revolute twins joint is a joint that has 1 degree of freedom of rotation and which has a variable center of rotation which changes with its configuration.
 * The joint is composed of two revolute joints that share the same axis of rotation, their angle is constrained such that overall the revolute twins has a
 * single degree of freedom. Example of a revolute twins joint:
 * <pre>
 *    root
 *      |
 *      |
 *      O A
 *      |
 *      O B
 *      |
 *      |
 * end-effector
 * </pre>
 * where A and B are both revolute joints around the same axis. It is assumed that only one joint in this sub-system composed of {A, B} is a torque source, this
 * is the actuated joint.
 * </p>
 */
public interface RevoluteTwinsJointReadOnly extends OneDoFJointReadOnly
{
   /**
    * Returns the reference to the actuated joint of this revolute twins. It is assumed to be the sole
    * torque source and is used to define the state of this revolute twins joint.
    *
    * @return the reference to the actuated joint.
    */
   RevoluteJointReadOnly getActuatedJoint();

   /**
    * Returns one of the two joints starting the linkage:
    * <pre>
    *    root
    *      |
    *      |
    *      O A
    *      |
    *      O B
    *      |
    *      |
    * end-effector
    * </pre>
    *
    * @return the reference to the first joint of this revolute twins.
    */
   RevoluteJointReadOnly getJointA();

   /**
    * Returns one of the two joints starting the linkage:
    * <pre>
    *    root
    *      |
    *      |
    *      O A
    *      |
    *      O B
    *      |
    *      |
    * end-effector
    * </pre>
    *
    * @return the reference to the second joint of this revolute twins.
    */
   RevoluteJointReadOnly getJointB();

   /**
    * Returns the rigid body that connects the two joints of this revolute twins.
    *
    * <pre>
    *    root
    *      |
    *      |
    *      O A
    *      |
    *      O B
    *      |
    *      |
    * end-effector
    * </pre>
    *
    * @return the reference to the rigid body connecting the two joints of this revolute twins.
    */
   default RigidBodyReadOnly getBodyAB()
   {
      return getJointA().getSuccessor();
   }

   /**
    * Given the revolute twins joint angle {@code q}, computes the corresponding angle for the actuated joint.
    *
    * @param q the revolute twins joint angle to compute the actuated joint angle for.
    * @return the corresponding actuated joint angle.
    */
   double computeActuatedJointQ(double q);

   /**
    * Given the revolute twins joint velocity {@code qDot}, computes the corresponding velocity for the actuated joint.
    *
    * @param qDot the revolute twins joint velocity to compute the actuated joint velocity for.
    * @return the corresponding actuated joint velocity.
    */
   double computeActuatedJointQd(double qDot);

   /**
    * Given the revolute twins joint acceleration {@code qDDot}, computes the corresponding acceleration for the actuated joint.
    *
    * @param qDDot the revolute twins joint acceleration to compute the actuated joint acceleration for.
    * @return the corresponding actuated joint acceleration.
    */
   double computeActuatedJointQdd(double qDDot);

   /**
    * Given the revolute twins joint torque {@code tau}, computes the corresponding torque for the actuated joint.
    *
    * @param tau the revolute twins joint torque to compute the actuated joint torque for.
    * @return the corresponding actuated joint torque.
    */
   double computeActuatedJointTau(double tau);

   /**
    * Gets the index in [0, 3] corresponding to the actuated joint.
    * <p>
    * The index can be used with the matrices {@link #getLoopJacobian()} and
    * {@link CrossFourBarJointReadOnly#getLoopConvectiveTerm()} to retrieve the row that correspond to
    * the mast joint.
    * </p>
    *
    * @return the index in [0, 3] corresponding to the actuated joint.
    */
   int getActuatedJointIndex();

   /**
    * Gets the Jacobian {@code G} which maps from the actuated joint velocity/acceleration to all internal joint velocities/accelerations:
    *
    * <pre>
    * qDot  = G * yDot
    * qDDot = G * yDDot + g
    * </pre>
    * <p>
    * The {@code G} matrix is a 2-by-1 matrix which row indexing follows the alphabetical ordering of
    * the joints, i.e. A then B, {@code g} is the convective term vector see
    * {@link #getConstraintConvectiveTerm()}, {@code qDot} and {@code qDDot} are the 2 element vectors of the
    * 2 constrained joint velocities and accelerations, and {@code yDot} and {@code yDDot} are the actuated
    * joint velocity and acceleration.
    * </p>
    *
    * @return the Jacobian of the revolute twins internal joints.
    */
   DMatrix getConstraintJacobian();

   /**
    * Gets the convective term {@code g} used in the mapping from the actuated joint acceleration to
    * all internal joint accelerations:
    *
    * <pre>
    * qDDot = G * yDDot + g
    * </pre>
    * <p>
    * The {@code g} is a 2 element vector which indexing follows the alphabetical ordering of the
    * joints, i.e. A then B, {@code G} the constrained Jacobian, see {@link #getConstraintJacobian()},
    * {@code qDDot} is the 2 element vector of the 2 loop joint accelerations, and {@code yDDot} is the
    * actuated joint acceleration.
    * </p>
    *
    * @return the convective term of the revolute twins internal joints.
    */
   DMatrix getConstraintConvectiveTerm();

   /**
    * Gets the ratio between the actuated joint and the constrained joint, such that:
    * <pre>
    *    q_constrained     = constraintRatio * q_actuated    + constraintOffset
    *    qDot_constrained  = constraintRatio * qDot_actuated
    *    qDDot_constrained = constraintRatio * qDDot_actuated
    * </pre>
    * <p>
    * The constrained joint is the joint that is not actuated, i.e. the joint that is not a torque source.
    * </p>
    *
    * @return the ratio between the actuated joint and the constrained joint.
    */
   double getConstraintRatio();

   /**
    * Gets the offset between the actuated joint and the constrained joint, such that:
    * <pre>
    *    q_constrained     = constraintRatio * q_actuated    + constraintOffset
    *    qDot_constrained  = constraintRatio * qDot_actuated
    *    qDDot_constrained = constraintRatio * qDDot_actuated
    * </pre>
    * <p>
    * The constrained joint is the joint that is not actuated, i.e. the joint that is not a torque source.
    * </p>
    *
    * @return the offset between the actuated joint and the constrained joint.
    */
   double getConstraintOffset();

   /**
    * {@inheritDoc}
    */
   @Override
   default boolean isMotionSubspaceVariable()
   {
      return true;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default FrameVector3DReadOnly getJointAxis()
   {
      return getActuatedJoint().getJointAxis();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getQ()
   {
      return getJointA().getQ() + getJointB().getQ();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getQd()
   {
      return getJointA().getQd() + getJointB().getQd();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getQdd()
   {
      return getJointA().getQdd() + getJointB().getQdd();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getJointLimitLower()
   {
      return getJointA().getJointLimitLower() + getJointB().getJointLimitLower();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getJointLimitUpper()
   {
      return getJointA().getJointLimitUpper() + getJointB().getJointLimitUpper();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getVelocityLimitLower()
   {
      return getJointA().getVelocityLimitLower() + getJointB().getVelocityLimitLower();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getVelocityLimitUpper()
   {
      return getJointA().getVelocityLimitUpper() + getJointB().getVelocityLimitUpper();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getEffortLimitLower()
   {
      return getActuatedJoint().getEffortLimitLower();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   default double getEffortLimitUpper()
   {
      return getActuatedJoint().getEffortLimitUpper();
   }
}
