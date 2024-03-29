package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

/**
 * Read-only interface for a cross four bar joint.
 * <p>
 * A cross four bar joint is a joint that has 1 degree of freedom of rotation and which has a
 * variable center of rotation which changes with its configuration. Example of a cross four bar
 * joint:
 * 
 * <pre>
 *    root
 *      |
 *      |
 * A O-----O B
 *    \   /
 *     \ /
 *      X
 *     / \
 *    /   \
 * C O-----O D
 *      |
 * end-effector
 * </pre>
 * 
 * where A, B, C, and D are all revolute joint around the same axis. It is assumed that only one
 * joint in this sub-system composed of {A, B, C, and D} is a torque source, this is the actuated
 * joint.
 * </p>
 */
public interface CrossFourBarJointReadOnly extends OneDoFJointReadOnly
{
   /**
    * Returns the reference to the actuated joint of this cross four bar. It is assumed to be the sole
    * torque source and is used to define the state of this cross four bar joint.
    * 
    * @return the reference to the actuated joint.
    */
   RevoluteJointReadOnly getActuatedJoint();

   /**
    * Returns one of the two joints starting the linkage:
    * 
    * <pre>
    *    root
    *      |
    *      |
    * A O-----O B
    *    \   /
    *     \ /
    *      X
    *     / \
    *    /   \
    * C O-----O D
    *      |
    * end-effector
    * </pre>
    * 
    * @return the reference to the joint A.
    */
   RevoluteJointReadOnly getJointA();

   /**
    * Returns one of the two joints starting the linkage:
    * 
    * <pre>
    *    root
    *      |
    *      |
    * A O-----O B
    *    \   /
    *     \ /
    *      X
    *     / \
    *    /   \
    * C O-----O D
    *      |
    * end-effector
    * </pre>
    * 
    * @return the reference to the joint B.
    */
   RevoluteJointReadOnly getJointB();

   /**
    * Returns one of the two joints ending the linkage:
    * 
    * <pre>
    *    root
    *      |
    *      |
    * A O-----O B
    *    \   /
    *     \ /
    *      X
    *     / \
    *    /   \
    * C O-----O D
    *      |
    * end-effector
    * </pre>
    * 
    * @return the reference to the joint C.
    */
   RevoluteJointReadOnly getJointC();

   /**
    * Returns one of the two joints ending the linkage:
    * 
    * <pre>
    *    root
    *      |
    *      |
    * A O-----O B
    *    \   /
    *     \ /
    *      X
    *     / \
    *    /   \
    * C O-----O D
    *      |
    * end-effector
    * </pre>
    * 
    * @return the reference to the joint D.
    */
   RevoluteJointReadOnly getJointD();

   /**
    * Returns the rigid body that represents one of the two cross bars:
    * 
    * <pre>
    *    root
    *      |
    *      |
    * A O-----O B
    *    \   /
    *     \ /
    *      X
    *     / \
    *    /   \
    * C O-----O D
    *      |
    * end-effector
    * </pre>
    * 
    * @return the reference to the rigid-body DA, i.e. successor of joint A and predecessor of joint D.
    */
   default RigidBodyReadOnly getBodyDA()
   {
      return getJointA().getSuccessor();
   }

   /**
    * Returns the rigid body that represents one of the two cross bars:
    * 
    * <pre>
    *    root
    *      |
    *      |
    * A O-----O B
    *    \   /
    *     \ /
    *      X
    *     / \
    *    /   \
    * C O-----O D
    *      |
    * end-effector
    * </pre>
    * 
    * @return the reference to the rigid-body BC, i.e. successor of joint B and predecessor of joint C.
    */
   default RigidBodyReadOnly getBodyBC()
   {
      return getJointB().getSuccessor();
   }

   /**
    * Given the cross four bar joint angle {@code q}, computes the corresponding angle for the actuated
    * joint.
    * 
    * @param q the cross four bar joint angle to compute the actuated joint angle for.
    * @return the corresponding actuated joint angle.
    */
   double computeActuatedJointQ(double q);

   /**
    * Given the cross four bar joint velocity {@code qd}, computes the corresponding velocity for the
    * actuated joint.
    * 
    * @param qd the cross four bar joint velocity to compute the actuated joint velocity for.
    * @return the corresponding actuated joint velocity.
    */
   double computeActuatedJointQd(double qd);

   /**
    * Given the cross four bar joint acceleration {@code qdd}, computes the corresponding acceleration
    * for the actuated joint.
    * 
    * @param qdd the cross four bar joint acceleration to compute the actuated joint acceleration for.
    * @return the corresponding actuated joint acceleration.
    */
   double computeActuatedJointQdd(double qdd);

   /**
    * Given the cross four bar joint torque {@code tau}, computes the corresponding torque for the
    * actuated joint.
    * 
    * @param tau the cross four bar joint torque to compute the actuated joint torque for.
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
    * Gets the Jacobian {@code G} which maps from the actuated joint velocity/acceleration to all loop
    * joint velocities/accelerations:
    * 
    * <pre>
    * qDot  = G * yDot
    * qDDot = G * yDDot + g
    * </pre>
    * <p>
    * The {@code G} matrix is a 4-by-1 matrix which row indexing follows the alphabetical ordering of
    * the joints, i.e. A, B, C, and D, {@code g} is the convective term vector see
    * {@link #getLoopConvectiveTerm()}, {@code qDot} and {@code qDDot} are the 4 element vectors of the
    * 4 loop joint velocities and accelerations, and {@code yDot} and {@code yDDot} are the actuated
    * joint velocity and acceleration.
    * </p>
    * 
    * @return the Jacobian of the four bar internal joints.
    * @see KinematicLoopFunction#getLoopJacobian()
    */
   DMatrix getLoopJacobian();

   /**
    * Gets the convective term {@code g} used in the mapping from the actuated joint acceleration to
    * all loop joint accelerations:
    * 
    * <pre>
    * qDDot = G * yDDot + g
    * </pre>
    * <p>
    * The {@code g} is a 4 element vector which indexing follows the alphabetical ordering of the
    * joints, i.e. A, B, C, and D, {@code G} the loop Jacobian, see {@link #getLoopJacobian()},
    * {@code qDDot} is the 4 element vector of the 4 loop joint accelerations, and {@code yDDot} is the
    * actuated joint acceleration.
    * </p>
    * 
    * @return the convective term of the four bar internal joints.
    * @see KinematicLoopFunction#getLoopConvectiveTerm()
    */
   DMatrix getLoopConvectiveTerm();

   /** {@inheritDoc} */
   @Override
   default boolean isMotionSubspaceVariable()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default FrameVector3DReadOnly getJointAxis()
   {
      return getActuatedJoint().getJointAxis();
   }

   /** {@inheritDoc} */
   @Override
   default double getQ()
   {
      return getJointA().getQ() + getJointD().getQ();
   }

   /** {@inheritDoc} */
   @Override
   default double getQd()
   {
      return getJointA().getQd() + getJointD().getQd();
   }

   /** {@inheritDoc} */
   @Override
   default double getQdd()
   {
      return getJointA().getQdd() + getJointD().getQdd();
   }

   /** {@inheritDoc} */
   @Override
   default double getJointLimitLower()
   {
      return getJointA().getJointLimitLower() + getJointD().getJointLimitLower();
   }

   /** {@inheritDoc} */
   @Override
   default double getJointLimitUpper()
   {
      return getJointA().getJointLimitUpper() + getJointD().getJointLimitUpper();
   }

   /** {@inheritDoc} */
   @Override
   default double getVelocityLimitLower()
   {
      return getJointA().getVelocityLimitLower() + getJointD().getVelocityLimitLower();
   }

   /** {@inheritDoc} */
   @Override
   default double getVelocityLimitUpper()
   {
      return getJointA().getVelocityLimitUpper() + getJointD().getVelocityLimitUpper();
   }

   /** {@inheritDoc} */
   @Override
   default double getEffortLimitLower()
   {
      return getActuatedJoint().getEffortLimitLower();
   }

   /** {@inheritDoc} */
   @Override
   default double getEffortLimitUpper()
   {
      return getActuatedJoint().getEffortLimitUpper();
   }
}
