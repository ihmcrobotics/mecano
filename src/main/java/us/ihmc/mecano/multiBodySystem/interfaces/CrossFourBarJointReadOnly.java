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
 * joint in this sub-system composed of {A, B, C, and D} is a torque source, this is the master
 * joint.
 * </p>
 */
public interface CrossFourBarJointReadOnly extends OneDoFJointReadOnly
{
   /**
    * Returns the reference to the master joint of this cross four bar. It is assumed to be the sole
    * torque source and is used to define the state of the this cross four bar joint.
    * 
    * @return the reference to the master joint.
    */
   RevoluteJointReadOnly getMasterJoint();

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
    * Given the cross four bar joint angle {@code q}, computes the corresponding angle for the master
    * joint.
    * 
    * @param q the cross four bar joint angle to compute the master joint angle for.
    * @return the corresponding master joint angle.
    */
   double computeMasterJointQ(double q);

   /**
    * Given the cross four bar joint velocity {@code qd}, computes the corresponding velocity for the
    * master joint.
    * 
    * @param qd the cross four bar joint velocity to compute the master joint velocity for.
    * @return the corresponding master joint velocity.
    */
   double computeMasterJointQd(double qd);

   /**
    * Given the cross four bar joint acceleration {@code qdd}, computes the corresponding acceleration
    * for the master joint.
    * 
    * @param qdd the cross four bar joint acceleration to compute the master joint acceleration for.
    * @return the corresponding master joint acceleration.
    */
   double computeMasterJointQdd(double qdd);

   /**
    * Given the cross four bar joint torque {@code tau}, computes the corresponding torque for the
    * master joint.
    * 
    * @param tau the cross four bar joint torque to compute the master joint torque for.
    * @return the corresponding master joint torque.
    */
   double computeMasterJointTau(double tau);

   /**
    * Gets the index in [0, 3] corresponding to the master joint.
    * <p>
    * The index can be used with the matrices {@link #getLoopJacobian()} and
    * {@link CrossFourBarJointReadOnly#getLoopConvectiveTerm()} to retrieve the row that correspond to
    * the mast joint.
    * </p>
    * 
    * @return the index in [0, 3] corresponding to the master joint.
    */
   int getMasterJointIndex();

   DMatrix getLoopJacobian();

   DMatrix getLoopConvectiveTerm();

   @Override
   default boolean isMotionSubspaceVariable()
   {
      return true;
   }

   @Override
   default FrameVector3DReadOnly getJointAxis()
   {
      return getMasterJoint().getJointAxis();
   }

   @Override
   default double getQ()
   {
      return getJointA().getQ() + getJointD().getQ();
   }

   @Override
   default double getQd()
   {
      return getJointA().getQd() + getJointD().getQd();
   }

   @Override
   default double getQdd()
   {
      return getJointA().getQdd() + getJointD().getQdd();
   }

   @Override
   default double getJointLimitLower()
   {
      return getJointA().getJointLimitLower() + getJointD().getJointLimitLower();
   }

   @Override
   default double getJointLimitUpper()
   {
      return getJointA().getJointLimitUpper() + getJointD().getJointLimitUpper();
   }

   @Override
   default double getVelocityLimitLower()
   {
      return getJointA().getVelocityLimitLower() + getJointD().getVelocityLimitLower();
   }

   @Override
   default double getVelocityLimitUpper()
   {
      return getJointA().getVelocityLimitUpper() + getJointD().getVelocityLimitUpper();
   }

   @Override
   default double getEffortLimitLower()
   {
      return getMasterJoint().getEffortLimitLower();
   }

   @Override
   default double getEffortLimitUpper()
   {
      return getMasterJoint().getEffortLimitUpper();
   }
}
