package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

/**
 * Write and read interface for any joint that gathers all the common information that a joint can
 * provide and be provided with.
 * <p>
 * A joint represents a part of the robot system that allows a rotation and/or translation of a
 * rigid-body, i.e. the successor, with respect to another rigid-body, i.e. the predecessor.
 * </p>
 * <p>
 * Each joint has a configuration, velocity, acceleration, and force and/or torque. Each of these
 * can be accessed for reading and writing purposes using this interface.
 * </p>
 * <p>
 * Each joint is assigned to reference frames: {@code frameBeforeJoint} and {@code frameAfterJoint}.
 * The transform from {@code frameAfterJoint} to {@code frameBeforeJoint} represents the
 * configuration of this joint. When the joint is at a "zero" configuration, these two reference
 * frames coincide.
 * </p>
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface JointBasics extends JointReadOnly
{
   @Override
   RigidBodyBasics getPredecessor();

   @Override
   RigidBodyBasics getSuccessor();

   /**
    * Attach the {@code RigidBody} located after this joint, i.e. the {@code successor}. "After" means
    * that the {@code RigidBody} sits between the joint and the end-effector of the kinematics chain,
    * or that its is the end-effector. This method is to be called <b>only once</b>, at creation of a
    * kinematics chain. It is also in this method, that the final steps of initialization of this joint
    * are made.
    *
    * @param successor the {@code RigidBody} located after this joint.
    */
   void setSuccessor(RigidBodyBasics successor);

   /**
    * Configures this joint as closing a kinematic loop in the multi-body system.
    * <p>
    * Requires the successor to be set beforehand.
    * </p>
    * 
    * @param successor                         the {@code RigidBody} located after this joint.
    * @param transformFromSuccessorParentJoint specifies the transform from
    *                                          {@code this.getFrameAfterJoint()} to
    *                                          {@code successor.getParentJoint().getFrameAfterJoint()}.
    *                                          The transform represents the pose of this joint with
    *                                          respect to the {@code successor}'s parent joint.
    */
   void setupLoopClosure(RigidBodyBasics successor, RigidBodyTransformReadOnly transformFromSuccessorParentJoint);

   /**
    * Resets this joint's configuration to zero.
    * <p>
    * For instance, for a {@code SphericalJoint}, this will reset the orientation as follows:
    * {@code joint.getJointOrientation().setToZero()}.
    * </p>
    */
   void setJointConfigurationToZero();

   /**
    * Resets this joint's twist to zero.
    */
   void setJointTwistToZero();

   /**
    * Resets this joint's acceleration to zero.
    */
   void setJointAccelerationToZero();

   /**
    * Resets this joint's effort to zero.
    */
   void setJointTauToZero();

   /**
    * Sets this joint configuration from the other joint.
    * <p>
    * The given joint has to be of same type as this joint.
    * </p>
    *
    * @param other the other to get the configuration from. Not modified.
    * @throws RuntimeException if the other joint type is incompatible.
    */
   void setJointConfiguration(JointReadOnly other);

   /**
    * Sets this joint velocity from the other joint.
    * <p>
    * The given joint has to be of same type as this joint.
    * </p>
    *
    * @param other the other to get the velocity from. Not modified.
    * @throws RuntimeException if the other joint type is incompatible.
    */
   void setJointTwist(JointReadOnly other);

   /**
    * Sets this joint acceleration from the other joint.
    * <p>
    * The given joint has to be of same type as this joint.
    * </p>
    *
    * @param other the other to get the acceleration from. Not modified.
    * @throws RuntimeException if the other joint type is incompatible.
    */
   void setJointAcceleration(JointReadOnly other);

   /**
    * Sets this joint force/torque from the other joint.
    * <p>
    * The given joint has to be of same type as this joint.
    * </p>
    *
    * @param other the other to get the force/torque from. Not modified.
    * @throws RuntimeException if the other joint type is incompatible.
    */
   void setJointWrench(JointReadOnly other);

   /**
    * Sets the joint current configuration from the given column vector {@code DMatrix}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column vector
    * is used to set the current joint angle {@code q}.
    * <li>For a {@code SixDoFJoint}, the 4 rows starting from {@code rowStart} are used to set the
    * current 3D orientation as a quaternion, and the 3 rows starting from ({@code rowStart + 4}) are
    * used to set the 3D position.
    * </ul>
    * <p>
    * When updating the configuration the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot configuration.
    * </p>
    *
    * @param rowStart           row index of the first component of this joint configuration.
    * @param jointConfiguration the column vector from which the configuration of this joint is to be
    *                           extracted. Not modified.
    * @return {@code rowStart + this.getConfigurationMatrixSize()}.
    */
   int setJointConfiguration(int rowStart, DMatrix jointConfiguration);

   /**
    * Sets this joint current velocity from the given column vector {@code DenseMAtrix64F}. Here are a
    * few examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column vector
    * is used to set the joint current angular velocity {@code qd}.
    * <li>For a {@code SixDoFJoint}, the 6 rows starting from {@code rowStart} are use to set the
    * current twist of this joint starting with the angular velocity. Note: the joint twist is the
    * twist of the {@code afterJointFrame} with respect to the {@code beforeJointFrame} expressed in
    * the {@code afterJointFrame}.
    * </ul>
    *
    * @param rowStart      row index of the first component of this joint velocity.
    * @param jointVelocity the column vector from which the current velocity of this joint is to be
    *                      extracted. Not modified.
    * @return {@code rowStart + this.getDegreesOfFreedom()}.
    */
   int setJointVelocity(int rowStart, DMatrix jointVelocity);

   /**
    * Sets this joint acceleration from the given column vector {@code DenseMAtrix64F}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column vector
    * is used to set the joint angular acceleration {@code qddDesired}.
    * <li>For a {@code SixDoFJoint}, the 6 rows starting from {@code rowStart} are use to set the
    * spatial acceleration of this joint starting with the angular acceleration. Note: the joint
    * spatial acceleration is the acceleration of the {@code afterJointFrame} with respect to the
    * {@code beforeJointFrame} expressed in the {@code afterJointFrame}.
    * </ul>
    *
    * @param rowStart          row index of the first component of this joint acceleration.
    * @param jointAcceleration the column vector from which the acceleration of this joint is to be
    *                          extracted. Not modified.
    * @return {@code rowStart + this.getDegreesOfFreedom()}.
    */
   int setJointAcceleration(int rowStart, DMatrix jointAcceleration);

   /**
    * Sets the joint current wrench from the given column vector {@code DMatrix}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the {@code rowStart}<sup>th</sup> row of the given column vector
    * is used to set the joint torque {@code tau}.
    * <li>For a {@code SixDoFJoint}, the 6 rows starting from {@code rowStart} are use to set the
    * current spatial wrench of this joint starting with the torque. Note: the joint wrench is the
    * wrench of the {@code afterJointFrame} with respect to the {@code beforeJointFrame} expressed in
    * the {@code afterJointFrame}.
    * </ul>
    *
    * @param rowStart row index of the first component of this joint configuration.
    * @param jointTau the column vector from which the configuration of this joint is to be extracted.
    *                 Not modified.
    * @return {@code rowStart + this.getDegreesOfFreedom()}.
    */
   int setJointTau(int rowStart, DMatrix jointTau);

   /**
    * Sets the joint current configuration from the given transform.
    * <p>
    * The common implementation of this method is to project the transform onto the motion subspace of
    * this joint such not all the components of the transform are used to update this joint
    * configuration. For instance, a {@code SphericalJoint} only uses the orientation part and a
    * {@code RevoluteJoint} projects the orientation onto its joint axis to compute its new joint
    * angle.
    * </p>
    * <p>
    * When updating the configuration the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot configuration.
    * </p>
    *
    * @param jointConfiguration the transform containing the new configuration for this joint. Not
    *                           modified.
    */
   default void setJointConfiguration(RigidBodyTransformReadOnly jointConfiguration)
   {
      setJointOrientation(jointConfiguration.getRotation());
      setJointPosition(jointConfiguration.getTranslation());
   }

   /**
    * Sets the joint current configuration from the given pose 3D.
    * <p>
    * The common implementation of this method is to project the pose onto the motion subspace of this
    * joint such not all the components of the pose are used to update this joint configuration. For
    * instance, a {@code SphericalJoint} only uses the orientation part and a {@code RevoluteJoint}
    * projects the orientation onto its joint axis to compute its new joint angle.
    * </p>
    * <p>
    * When updating the configuration the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot configuration.
    * </p>
    *
    * @param jointConfiguration the pose 3D containing the new configuration for this joint. Not
    *                           modified.
    */
   default void setJointConfiguration(Pose3DReadOnly jointConfiguration)
   {
      setJointOrientation(jointConfiguration.getOrientation());
      setJointPosition(jointConfiguration.getPosition());
   }

   /**
    * Sets the joint current configuration from the given orientation and position.
    * <p>
    * The common implementation of this method is to project the pose onto the motion subspace of this
    * joint such not all the components of the pose are used to update this joint configuration. For
    * instance, a {@code SphericalJoint} only uses the orientation part and a {@code RevoluteJoint}
    * projects the orientation onto its joint axis to compute its new joint angle.
    * </p>
    * <p>
    * When updating the configuration the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot configuration.
    * </p>
    *
    * @param jointOrientation the new orientation for this joint. Unused for a joint that cannot
    *                         rotate. Not modified.
    * @param jointPosition    the new position for this joint. Unused for a joint that cannot
    *                         translate. Not modified.
    */
   default void setJointConfiguration(Orientation3DReadOnly jointOrientation, Tuple3DReadOnly jointPosition)
   {
      setJointOrientation(jointOrientation);
      setJointPosition(jointPosition);
   }

   /**
    * Sets, if possible, the joint current orientation.
    * <p>
    * This method is ineffective for joints that cannot rotate.
    * </p>
    * <p>
    * The common implementation of this method is to project the orientation onto the motion subspace
    * of this joint such not all the components of the given orientation are used to update this joint
    * configuration. For instance, a {@code RevoluteJoint} projects the orientation onto its joint axis
    * to compute its new joint angle.
    * </p>
    * <p>
    * When updating the configuration the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot configuration.
    * </p>
    *
    * @param jointOrientation the new orientation for this joint. Unused for a joint that cannot
    *                         rotate. Not modified.
    */
   void setJointOrientation(Orientation3DReadOnly jointOrientation);

   /**
    * Sets, if possible, the joint current position.
    * <p>
    * This method is ineffective for joints that cannot translate.
    * </p>
    * <p>
    * The common implementation of this method is to project the position onto the motion subspace of
    * this joint such not all the components of the given position are used to update this joint
    * configuration. For instance, a {@code PrismaticJoint} projects the orientation onto its joint
    * axis to compute its new joint position.
    * </p>
    * <p>
    * When updating the configuration the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot configuration.
    * </p>
    *
    * @param jointPosition the new position for this joint. Unused for a joint that cannot translate.
    *                      Not modified.
    */
   void setJointPosition(Tuple3DReadOnly jointPosition);

   /**
    * Sets this joint current velocity to the given twist.
    * <p>
    * As for configuration setters, the common implementation here is to project the given twist onto
    * the motion subspace of this joint such that not all the components of the given twist may
    * necessarily be used.
    * </p>
    * <p>
    * When updating the velocity the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot motion.
    * </p>
    *
    * @param jointTwist the new twist for this joint. Not modified.
    * @throws ReferenceFrameMismatchException if the given twist does not have the following frames:
    *                                         <ul>
    *                                         <li>{@code bodyFrame} is {@code afterJointFrame}.
    *                                         <li>{@code baseFrame} is {@code beforeJointFrame}.
    *                                         <li>{@code expressedInFrame} is {@code afterJointFrame}.
    *                                         </ul>
    */
   default void setJointTwist(TwistReadOnly jointTwist)
   {
      jointTwist.checkBodyFrameMatch(getFrameAfterJoint());
      jointTwist.checkBaseFrameMatch(getFrameBeforeJoint());
      jointTwist.checkExpressedInFrameMatch(getFrameAfterJoint());
      setJointAngularVelocity((Vector3DReadOnly) jointTwist.getAngularPart());
      setJointLinearVelocity((Vector3DReadOnly) jointTwist.getLinearPart());
   }

   /**
    * Sets the angular velocity of this joint.
    * <p>
    * This method is ineffective for joints that cannot rotate.
    * </p>
    * <p>
    * As for configuration setters, the common implementation here is to project the given angular
    * velocity onto the motion subspace of this joint such that not all the components of the given
    * velocity may necessarily be used.
    * </p>
    * <p>
    * When updating the velocity the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot motion.
    * </p>
    *
    * @param jointAngularVelocity the new angular velocity for this joint. Unused if this joint cannot
    *                             rotate. Not modified.
    * @throws ReferenceFrameMismatchException if the given vector is not expressed in
    *                                         {@code frameAfterJoint}.
    */
   default void setJointAngularVelocity(FrameVector3DReadOnly jointAngularVelocity)
   {
      jointAngularVelocity.checkReferenceFrameMatch(getFrameAfterJoint());
      setJointAngularVelocity((Vector3DReadOnly) jointAngularVelocity);
   }

   /**
    * Sets the linear velocity of this joint.
    * <p>
    * This method assumes that the given velocity describes the linear velocity of the
    * {@code frameAfterJoint}'s origin.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot translate.
    * </p>
    * <p>
    * As for configuration setters, the common implementation here is to project the given linear
    * velocity onto the motion subspace of this joint such that not all the components of the given
    * velocity may necessarily be used.
    * </p>
    * <p>
    * When updating the velocity the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot motion.
    * </p>
    *
    * @param jointLinearVelocity the new linear velocity for this joint. Unused if this joint cannot
    *                            translate. Not modified.
    * @throws ReferenceFrameMismatchException if the given vector is not expressed in
    *                                         {@code frameAfterJoint}.
    */
   default void setJointLinearVelocity(FrameVector3DReadOnly jointLinearVelocity)
   {
      jointLinearVelocity.checkReferenceFrameMatch(getFrameAfterJoint());
      setJointLinearVelocity((Vector3DReadOnly) jointLinearVelocity);
   }

   /**
    * Sets the angular velocity of this joint.
    * <p>
    * This method assumes that the given velocity is expressed in local joint coordinates, i.e. in
    * {@code frameAfterJoint}.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot rotate.
    * </p>
    * <p>
    * As for configuration setters, the common implementation here is to project the given angular
    * velocity onto the motion subspace of this joint such that not all the components of the given
    * velocity may necessarily be used.
    * </p>
    * <p>
    * When updating the velocity the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot motion.
    * </p>
    *
    * @param jointAngularVelocity the new angular velocity for this joint. Unused if this joint cannot
    *                             rotate. Not modified.
    */
   void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity);

   /**
    * Sets the linear velocity of this joint.
    * <p>
    * This method assumes that the given velocity describes the linear velocity of the
    * {@code frameAfterJoint}'s origin and that the vector is expressed in local joint coordinates,
    * i.e. in {@code frameAfterJoint}.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot translate.
    * </p>
    * <p>
    * As for configuration setters, the common implementation here is to project the given linear
    * velocity onto the motion subspace of this joint such that not all the components of the given
    * velocity may necessarily be used.
    * </p>
    * <p>
    * When updating the velocity the joints of a robot, it is important to then call
    * {@link #updateFramesRecursively()} on the root joint to update all the reference frames to the
    * new robot motion.
    * </p>
    *
    * @param jointLinearVelocity the new linear velocity for this joint. Unused if this joint cannot
    *                            translate. Not modified.
    */
   void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity);

   /**
    * Sets this joint current acceleration to the given spatial acceleration.
    * <p>
    * As for configuration setters, the common implementation here is to project the given acceleration
    * onto the motion subspace of this joint such that not all the components of the given spatial
    * acceleration may necessarily be used.
    * </p>
    *
    * @param jointAcceleration the new acceleration for this joint. Not modified.
    * @throws ReferenceFrameMismatchException if the given spatial acceleration does not have the
    *                                         following frames:
    *                                         <ul>
    *                                         <li>{@code bodyFrame} is {@code afterJointFrame}.
    *                                         <li>{@code baseFrame} is {@code beforeJointFrame}.
    *                                         <li>{@code expressedInFrame} is {@code afterJointFrame}.
    *                                         </ul>
    */
   default void setJointAcceleration(SpatialAccelerationReadOnly jointAcceleration)
   {
      jointAcceleration.checkBodyFrameMatch(getFrameAfterJoint());
      jointAcceleration.checkBaseFrameMatch(getFrameBeforeJoint());
      jointAcceleration.checkExpressedInFrameMatch(getFrameAfterJoint());
      setJointAngularAcceleration((Vector3DReadOnly) jointAcceleration.getAngularPart());
      setJointLinearAcceleration((Vector3DReadOnly) jointAcceleration.getLinearPart());
   }

   /**
    * Sets the angular acceleration of this joint.
    * <p>
    * This method is ineffective for joints that cannot rotate.
    * </p>
    * <p>
    * As for configuration setters, the common implementation here is to project the given angular
    * acceleration onto the motion subspace of this joint such that not all the components of the given
    * acceleration may necessarily be used.
    * </p>
    *
    * @param jointAngularAcceleration the new angular acceleration for this joint. Unused if this joint
    *                                 cannot rotate. Not modified.
    * @throws ReferenceFrameMismatchException if the given vector is not expressed in
    *                                         {@code frameAfterJoint}.
    */
   default void setJointAngularAcceleration(FrameVector3DReadOnly jointAngularAcceleration)
   {
      jointAngularAcceleration.checkReferenceFrameMatch(getFrameAfterJoint());
      setJointAngularAcceleration((Vector3DReadOnly) jointAngularAcceleration);
   }

   /**
    * Sets the linear acceleration of this joint.
    * <p>
    * This method assumes that the given acceleration describes the linear acceleration of the
    * {@code frameAfterJoint}'s origin.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot translate.
    * </p>
    * <p>
    * As for configuration setters, the common implementation here is to project the given linear
    * acceleration onto the motion subspace of this joint such that not all the components of the given
    * acceleration may necessarily be used.
    * </p>
    *
    * @param jointLinearAcceleration the new linear acceleration for this joint. Unused if this joint
    *                                cannot translate. Not modified.
    * @throws ReferenceFrameMismatchException if the given vector is not expressed in
    *                                         {@code frameAfterJoint}.
    */
   default void setJointLinearAcceleration(FrameVector3DReadOnly jointLinearAcceleration)
   {
      jointLinearAcceleration.checkReferenceFrameMatch(getFrameAfterJoint());
      setJointLinearAcceleration((Vector3DReadOnly) jointLinearAcceleration);
   }

   /**
    * Sets the angular acceleration of this joint.
    * <p>
    * This method assumes that the given acceleration is expressed in local joint coordinates, i.e. in
    * {@code frameAfterJoint}.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot rotate.
    * </p>
    * <p>
    * As for configuration setters, the common implementation here is to project the given angular
    * acceleration onto the motion subspace of this joint such that not all the components of the given
    * acceleration may necessarily be used.
    * </p>
    *
    * @param jointAngularAcceleration the new angular velocity for this joint. Unused if this joint
    *                                 cannot rotate. Not modified.
    */
   void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration);

   /**
    * Sets the linear acceleration of this joint.
    * <p>
    * This method assumes that the given acceleration describes the linear acceleration of the
    * {@code frameAfterJoint}'s origin and that the vector is expressed in local joint coordinates,
    * i.e. in {@code frameAfterJoint}.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot translate.
    * </p>
    * <p>
    * As for configuration setters, the common implementation here is to project the given linear
    * acceleration onto the motion subspace of this joint such that not all the components of the given
    * acceleration may necessarily be used.
    * </p>
    *
    * @param jointLinearAcceleration the new linear acceleration for this joint. Unused if this joint
    *                                cannot translate. Not modified.
    */
   void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration);

   /**
    * Sets this joint current force/torque to the given wrench.
    * <p>
    * As for the other spatial setters, this method's common implementation is to project the given
    * wrench to this joint's motion subspace. For instance, a {@code RevoluteJoint} that can rotate
    * around the y-axis will extract the torque around the y-axis from the given wrench to update its
    * torque.
    * </p>
    *
    * @param jointWrench the new wrench for this joint. Not modified.
    * @throws ReferenceFrameMismatchException if the given wrench does not have the following frames:
    *                                         <ul>
    *                                         <li>{@code bodyFrame} is
    *                                         {@code successor.getBodyFixedFrame()}.
    *                                         <li>{@code expressedInFrame} is {@code frameAfterJoint}.
    *                                         </ul>
    */
   default void setJointWrench(WrenchReadOnly jointWrench)
   {
      jointWrench.checkBodyFrameMatch(getSuccessor().getBodyFixedFrame());
      jointWrench.checkExpressedInFrameMatch(getFrameAfterJoint());
      setJointTorque((Vector3DReadOnly) jointWrench.getAngularPart());
      setJointForce((Vector3DReadOnly) jointWrench.getLinearPart());
   }

   /**
    * Sets this joint current torque.
    * <p>
    * This method assumes that the given torque is the torque at the {@code frameAfterJoint}'s origin.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot rotate.
    * </p>
    * <p>
    * As for the other spatial setters, this method's common implementation is to project the given 3D
    * torque to this joint's motion subspace. For instance, a {@code RevoluteJoint} that can rotate
    * around the y-axis will extract the torque around the y-axis from the given 3D torque to update
    * its torque.
    * </p>
    *
    * @param jointTorque the new torque for this joint. Unused if this joint cannot rotate. Not
    *                    modified.
    * @throws ReferenceFrameMismatchException if the given vector is not expressed in
    *                                         {@code frameAfterJoint}.
    */
   default void setJointTorque(FrameVector3DReadOnly jointTorque)
   {
      jointTorque.checkReferenceFrameMatch(getFrameAfterJoint());
      setJointTorque((Vector3DReadOnly) jointTorque);
   }

   /**
    * Sets this joint current force.
    * <p>
    * This method is ineffective for joints that cannot translate.
    * </p>
    * <p>
    * As for the other spatial setters, this method's common implementation is to project the given 3D
    * force to this joint's motion subspace. For instance, a {@code PrismaticJoint} that can translate
    * along the z-axis will extract the force along the z-axis from the given 3D force to update its
    * force.
    * </p>
    *
    * @param jointForce the new force for this joint. Unused if this joint cannot translate. Not
    *                   modified.
    * @throws ReferenceFrameMismatchException if the given vector is not expressed in
    *                                         {@code frameAfterJoint}.
    */
   default void setJointForce(FrameVector3DReadOnly jointForce)
   {
      jointForce.checkReferenceFrameMatch(getFrameAfterJoint());
      setJointForce((Vector3DReadOnly) jointForce);
   }

   /**
    * Sets this joint current torque.
    * <p>
    * This method assumes that the given torque is the torque at the {@code frameAfterJoint}'s origin
    * and the vector is expressed in the same frame.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot rotate.
    * </p>
    * <p>
    * As for the other spatial setters, this method's common implementation is to project the given 3D
    * torque to this joint's motion subspace. For instance, a {@code RevoluteJoint} that can rotate
    * around the y-axis will extract the torque around the y-axis from the given 3D torque to update
    * its torque.
    * </p>
    *
    * @param jointTorque the new torque for this joint. Unused if this joint cannot rotate. Not
    *                    modified.
    */
   void setJointTorque(Vector3DReadOnly jointTorque);

   /**
    * Sets this joint current force.
    * <p>
    * This method assumes that the given force is expressed in {@code frameAfterJoint}.
    * </p>
    * <p>
    * This method is ineffective for joints that cannot translate.
    * </p>
    * <p>
    * As for the other spatial setters, this method's common implementation is to project the given 3D
    * force to this joint's motion subspace. For instance, a {@code PrismaticJoint} that can translate
    * along the z-axis will extract the force along the z-axis from the given 3D force to update its
    * force.
    * </p>
    *
    * @param jointForce the new force for this joint. Unused if this joint cannot translate. Not
    *                   modified.
    */
   void setJointForce(Vector3DReadOnly jointForce);

   /**
    * Update the motion subspace of this joint. It is only necessary for when the motion subspace
    * depends on this joint configuration. A good example is a four bar linkage, for which the motion
    * subspace depends on the linkage configuration.
    */
   default void updateMotionSubspace()
   {
      // The common scenario is that the motion subspace is invariant.
   }

   @Override
   default Iterable<? extends JointBasics> subtreeIterable()
   {
      return new JointIterable<>(JointBasics.class, null, this);
   }

   @Override
   default Stream<? extends JointBasics> subtreeStream()
   {
      return SubtreeStreams.from(this);
   }

   @Override
   default List<? extends JointBasics> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   @Override
   default JointBasics[] subtreeArray()
   {
      return subtreeStream().toArray(JointBasics[]::new);
   }

   /**
    * Updates {@code afterJointFrame} of this joint to take into consideration the new joint
    * configuration and velocity.
    * <p>
    * This method only updates the frame for this joint, use {@link #updateFramesRecursively()} to
    * update all reference frames in the subtree starting at this joint.
    * </p>
    */
   default void updateFrame()
   {
      getFrameBeforeJoint().update();
      getFrameAfterJoint().update();
   }

   /**
    * Updates {@code afterJointFrame} of this joint to take into consideration the new joint
    * configuration. Then calls {@link RigidBody#updateFramesRecursively()} which in its turn updates
    * its {@code bodyFixedFrame} and then {@link #updateFramesRecursively()} for all of its
    * {@link JointBasics} child.
    * <p>
    * As a result, this method will update all the reference frames of the subtree starting from this
    * joint.
    * </p>
    * <p>
    * In addition to updating their respective poses, the reference frame also updates their velocity
    * based on the joint velocities.
    * </p>
    */
   default void updateFramesRecursively()
   {
      updateFrame();

      if (getSuccessor() != null)
      {
         getSuccessor().updateFramesRecursively();
      }
   }
}
