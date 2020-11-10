package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrix1Row;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SphericalJoint;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

/**
 * Read-only interface for any joint that gathers all the common information that a joint can
 * provide.
 * <p>
 * A joint represents a part of the robot system that allows a rotation and/or translation of a
 * rigid-body, i.e. the successor, with respect to another rigid-body, i.e. the predecessor.
 * </p>
 * <p>
 * Each joint has a configuration, velocity, acceleration, and force and/or torque. Each of these
 * can be accessed for reading purposes using this interface.
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
public interface JointReadOnly
{
   /** Represents the max number of degrees of freedom that a single joint can have. */
   static int MAX_NUMBER_OF_DOFS = SixDoFJointReadOnly.NUMBER_OF_DOFS;
   /** A string used to separate joint names in the {@link #getNameId()}. */
   static final String NAME_ID_SEPARATOR = ":";

   /**
    * Returns the {@code RigidBody} that precedes this joint. In other words, the {@code RigidBody}
    * directly connected to this joint that sits between this joint and the root or that is the root of
    * this kinematics chain.
    *
    * @return the {@code predecessor} of this joint.
    */
   RigidBodyReadOnly getPredecessor();

   /**
    * Returns the {@code RigidBody} that succeeds this joint. In other words, the {@code RigidBody}
    * directly connected to this joint that sits between this joint and the end-effector or that is the
    * end-effector of this kinematics chain.
    *
    * @return the {@code successor} of this joint.
    */
   RigidBodyReadOnly getSuccessor();

   /**
    * Returns the the {@code MovingReferenceFrame} that is attached to the predecessor of this joint,
    * namely the {@code RigidBody} before this joint, and has its origin centered at the joint origin.
    * The pose of the {@code frameBeforeJoint} is independent from this joint motion.
    *
    * @return the {@code MovingReferenceFrame} located right before this joint.
    */
   MovingReferenceFrame getFrameBeforeJoint();

   /**
    * Returns the the {@code MovingReferenceFrame} that is attached to the successor of this joint,
    * namely the {@code RigidBody} after this joint, and has its origin centered at the joint origin.
    * The pose of the {@code frameAfterJoint} will change as this joint moves.
    *
    * @return the {@code MovingReferenceFrame} located right after this joint.
    */
   MovingReferenceFrame getFrameAfterJoint();

   /**
    * Packs the configuration of this joint as a transform.
    *
    * @param jointConfigurationToPack transform in which the pose of the frame after joint expressed in
    *                                 the frame before joint is stored. Modified.
    */
   void getJointConfiguration(RigidBodyTransform jointConfigurationToPack);

   /**
    * Gets the read-only reference to this joint twist (the 3D angular and linear velocities). The
    * reference frames of the joint twist are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    *
    * @return the read-only reference of this joint twist.
    */
   TwistReadOnly getJointTwist();

   /**
    * Retrieves the unit-twist corresponding to each degree of freedom for this joint.
    * <p>
    * Unit-twist are mostly used to compute a Jacobian.
    * </p>
    * <p>
    * For instance, the unit-twist for a {@link RevoluteJoint} about the y-axis is equal to [0, 1, 0,
    * 0, 0, 0]<sup>T</sup> with {@code bodyFrame = frameAfterJoint},
    * {@code baseFrame = frameBeforeJoint}, and expressed in the predecessor body-fixed frame.
    * </p>
    *
    * @return the list of this joint unit-twists, one per degree of freedom.
    */
   List<TwistReadOnly> getUnitTwists();

   /**
    * Returns the {@code MovingReferenceFrame} which parent is {@link #getFrameAfterJoint()} and should
    * match {@code getSuccessor().getParentJoint().getFrameAfterJoint()} when the kinematic loop is
    * properly closed.
    * 
    * @return the reference frame used for algorithms to correct the loop closure.
    */
   MovingReferenceFrame getLoopClosureFrame();

   /**
    * Indicates whether this joint closes a kinematic loop in the multi-body system.
    * 
    * @return {@code true} if this joint closes a kinematic loop, {@code false} otherwise.
    */
   default boolean isLoopClosure()
   {
      return getLoopClosureFrame() != null;
   }

   /**
    * Indicates whether the motion subspace of this joint is constant with respect to the joint's
    * configuration.
    * <p>
    * Simple joints such as {@link RevoluteJoint}s and {@link SphericalJoint}s have an invariant motion
    * subspace. Only complex joints may have a motion subspace that is a function of their
    * configuration, such joint are not implemented in this library.
    * </p>
    * <p>
    * If this returns {@code false}, then the derivative of this joint's motion subspace is always
    * zero.
    * </p>
    * 
    * @return {@code true} to indicate that the motion subspace for this joint depends on its current
    *         configuration, {@code false} (default) otherwise.
    */
   default boolean isMotionSubspaceVariable()
   {
      return false;
   }

   /**
    * Packs the motion subspace of this joint into the given matrix.
    * <p>
    * The resulting matrix is a 6-by-N matrix, where N is equal to the number of DoFs this joint has,
    * see {@link #getDegreesOfFreedom()}. The motion subspace is equivalent to the geometric Jacobian
    * of this joint only expressed in {@code frameAfterJoint}.
    * </p>
    *
    * @param matrixToPack the matrix used to store the motion subspace. It is reshaped to the proper
    *                     size. Modified.
    */
   default void getMotionSubspace(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(SpatialVectorReadOnly.SIZE, getDegreesOfFreedom());

      for (int dofIndex = 0; dofIndex < getDegreesOfFreedom(); dofIndex++)
         getUnitTwists().get(dofIndex).get(0, dofIndex, matrixToPack);
   }

   /**
    * Packs the first derivative of the motion subspace of this joint into the given matrix.
    * <p>
    * Note that for most joints, the motion subspace is constant with respect to the joint
    * configuration and thus its derivative is always zero.
    * </p>
    * <p>
    * Note that the bias acceleration can be used as a way to compute <tt>dS/dt * qDot</tt>, where
    * <tt>dS/dt</tt> is the motion subspace derivative and <tt>qDot</tt> the joint velocity, and is
    * usually cheaper computation-wise.
    * </p>
    * 
    * @param matrixToPack the matrix used to store the first derivative of the motion subspace. It is
    *                     reshaped to the proper size. Modified.
    * @return {@code true} if the motion subspace derivative was computed and packed into the given
    *         matrix. This method returns {@code false} (default) if the given matrix was left
    *         unchanged, this is usually because the motion subspace is invariant.
    */
   default boolean getMotionSubspaceDot(DMatrix1Row matrixToPack)
   {
      return false;
   }

   /**
    * Packs the twist (the 3D angular and linear velocities) of this joint's {@code successor} with
    * respect to this joint's {@code predecessor}. The reference frames of the resulting twist are as
    * follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    *
    * @param successorTwistToPack the object in which the velocity of this joint's {@code successor} is
    *                             stored. Modified.
    */
   default void getSuccessorTwist(TwistBasics successorTwistToPack)
   {
      successorTwistToPack.setIncludingFrame(getJointTwist());

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      successorTwistToPack.setBaseFrame(predecessorFrame);
      successorTwistToPack.setBodyFrame(successorFrame);
      successorTwistToPack.changeFrame(successorFrame);
   }

   /**
    * Packs the twist (the 3D angular and linear velocities) of this joint's {@code predecessor} with
    * respect to this joint's {@code successor}. The reference frames of the resulting twist are as
    * follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code predecessorFrame}.
    * </ul>
    *
    * @param predecessorTwistToPack the object in which the velocity of this joint's
    *                               {@code predecessor} is stored. Modified.
    */
   default void getPredecessorTwist(TwistBasics predecessorTwistToPack)
   {
      predecessorTwistToPack.setIncludingFrame(getJointTwist());

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      predecessorTwistToPack.setBaseFrame(predecessorFrame);
      predecessorTwistToPack.setBodyFrame(successorFrame);
      predecessorTwistToPack.invert();
      predecessorTwistToPack.changeFrame(predecessorFrame);
   }

   /**
    * Gets the read-only reference to this joint spatial acceleration (the 3D angular and linear
    * accelerations). The reference frames of the joint spatial acceleration are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    *
    * @return the read-only reference of this joint spatial acceleration.
    */
   SpatialAccelerationReadOnly getJointAcceleration();

   /**
    * Gets the read-only reference to this joint's bias acceleration, i.e. acceleration solely due to
    * velocities.
    * <p>
    * Note that only complex joints have a bias acceleration, common joints such as
    * {@link RevoluteJoint}s or {@link PrismaticJoint}s will return {@code null}.
    * </p>
    * <p>
    * The reference frames of the joint bias spatial acceleration are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code afterJointFrame}.
    * <li>{@code baseFrame} is {@code beforeJointFrame}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    * </p>
    * 
    * @return the bias acceleration of this joint, or {@code null} if it is zero.
    */
   default SpatialAccelerationReadOnly getJointBiasAcceleration()
   {
      return null;
   }

   /**
    * Packs the spatial acceleration (the 3D angular and linear accelerations) of this joint's
    * {@code successor} with respect to this joint's {@code predecessor}. The reference frames of the
    * resulting spatial acceleration are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    *
    * @param successorAccelerationToPack the object in which the acceleration of this joint's
    *                                    {@code successor} is stored. Modified.
    */
   default void getSuccessorAcceleration(SpatialAccelerationBasics successorAccelerationToPack)
   {
      successorAccelerationToPack.setIncludingFrame(getJointAcceleration());

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      successorAccelerationToPack.setBaseFrame(predecessorFrame);
      successorAccelerationToPack.setBodyFrame(successorFrame);
      successorAccelerationToPack.changeFrame(successorFrame);

      if (isMotionSubspaceVariable())
      {
         SpatialAccelerationReadOnly successorBiasAcceleration = getSuccessorBiasAcceleration();
         successorAccelerationToPack.checkReferenceFrameMatch(successorBiasAcceleration);
         successorAccelerationToPack.add((SpatialVectorReadOnly) successorBiasAcceleration);
      }
   }

   /**
    * Gets the read-only reference to the bias acceleration, i.e. acceleration solely due to
    * velocities, of this joint's {@code successor} with respect to this joint's {@code predecessor}.
    * <p>
    * Note that only complex joints have a bias acceleration, common joints such as
    * {@link RevoluteJoint}s or {@link PrismaticJoint}s will return {@code null}.
    * </p>
    * <p>
    * The reference frames of the joint bias spatial acceleration are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code successorFrame}.
    * </ul>
    * </p>
    * 
    * @return the the bias acceleration of this joint's {@code successor}, or {@code null} if it is
    *         zero.
    */
   default SpatialAccelerationReadOnly getSuccessorBiasAcceleration()
   {
      return null;
   }

   /**
    * Packs the spatial acceleration (the 3D angular and linear accelerations) of this joint's
    * {@code predecessor} with respect to this joint's {@code successor} resulting from the desired
    * acceleration of this joint. The reference frames of the resulting spatial acceleration are as
    * follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code predecessorFrame}.
    * </ul>
    *
    * @param predecessorAccelerationToPack the object in which the acceleration of this joint's
    *                                      {@code predecessor} resulting from this joint desired
    *                                      acceleration is stored. Modified.
    */
   default void getPredecessorAcceleration(SpatialAccelerationBasics predecessorAccelerationToPack)
   {
      predecessorAccelerationToPack.setIncludingFrame(getJointAcceleration());

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      predecessorAccelerationToPack.setBaseFrame(predecessorFrame);
      predecessorAccelerationToPack.setBodyFrame(successorFrame);
      predecessorAccelerationToPack.invert();
      predecessorAccelerationToPack.changeFrame(predecessorFrame);

      if (isMotionSubspaceVariable())
      {
         SpatialAccelerationReadOnly predecessorBiasAcceleration = getPredecessorBiasAcceleration();
         predecessorAccelerationToPack.checkReferenceFrameMatch(predecessorBiasAcceleration);
         predecessorAccelerationToPack.add((SpatialVectorReadOnly) predecessorBiasAcceleration);
      }
   }

   /**
    * Gets the read-only reference to the bias acceleration, i.e. acceleration solely due to
    * velocities, of this joint's {@code predecessor} with respect to this joint's {@code successor}.
    * <p>
    * Note that only complex joints have a bias acceleration, common joints such as
    * {@link RevoluteJoint}s or {@link PrismaticJoint}s will return {@code null}.
    * </p>
    * <p>
    * The reference frames of the joint bias spatial acceleration are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code predecessorFrame = predecessor.getBodyFixedFrame()}.
    * <li>{@code baseFrame} is {@code successorFrame = successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code predecessorFrame}.
    * </ul>
    * </p>
    * 
    * @return the the bias acceleration of this joint's {@code predecessor}, or {@code null} if it is
    *         zero.
    */
   default SpatialAccelerationReadOnly getPredecessorBiasAcceleration()
   {
      return null;
   }

   /**
    * Gets the read-only reference to this joint wrench (the 3D torque and force). The reference frames
    * of the joint wrench are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code successor.getBodyFixedFrame()}.
    * <li>{@code expressedInFrame} is {@code afterJointFrame}.
    * </ul>
    *
    * @return the read-only reference of this joint wrench.
    */
   WrenchReadOnly getJointWrench();

   /**
    * Packs this joint's configuration into a column vector {@code DMatrix}. Here are a few examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the actual joint angle is stored at the
    * {@code rowStart}<sup>th</sup> row.
    * <li>For a {@code SphericalJoint}, the actual joint configuration is a quaternion and is stored
    * from the {@code rowStart}<sup>th</sup> row to the ({@code rowStart + 3})<sup>th</sup> row.
    * </ul>
    *
    * @param rowStart     row index for the first component of the configuration.
    * @param matrixToPack the column vector in which this joint actual configuration is stored.
    *                     Modified.
    * @return {@code rowStart + this.getConfigurationMatrixSize()}.
    */
   int getJointConfiguration(int rowStart, DMatrix matrixToPack);

   /**
    * Packs this joint actual velocity into a column vector {@code DMatrix}. Here are a few examples:
    * <ul>
    * <li>For a {@code OneDoFJoint}, the scalar velocity {@code qd} is stored at the
    * {@code rowStart}<sup>th</sup> row.
    * <li>For a {@code SixDoFJoint}, the joint twist is stored from the {@code rowStart}<sup>th</sup>
    * row to the ({@code rowStart + 5})<sup>th</sup> row, starting with the three components of the
    * angular velocity. Note: the joint twist is the twist of the {@code afterJointFrame} with respect
    * to the {@code beforeJointFrame} expressed in the {@code afterJointFrame}.
    * </ul>
    *
    * @param rowStart     row index for the first component of the velocity.
    * @param matrixToPack the column vector in which the velocity of this joint is stored. Modified.
    * @return {@code rowStart + this.getDegreesOfFreedom()}.
    */
   int getJointVelocity(int rowStart, DMatrix matrixToPack);

   /**
    * Packs this joint desired acceleration into a column vector {@code DMatrix}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code OneDoFJoint}, the scalar acceleration {@code qddDesired} is stored at the
    * {@code rowStart}<sup>th</sup> row.
    * <li>For a {@code SixDoFJoint}, the joint desired spatial acceleration is stored from the
    * {@code rowStart}<sup>th</sup> row to the ({@code rowStart + 5})<sup>th</sup> row, starting with
    * the three components of the angular acceleration.
    * </ul>
    *
    * @param rowStart     row index for the first component of the acceleration.
    * @param matrixToPack the column vector in which the acceleration of this joint is stored.
    *                     Modified.
    * @return {@code rowStart + this.getDegreesOfFreedom()}.
    */
   int getJointAcceleration(int rowStart, DMatrix matrixToPack);

   /**
    * Packs this joint desired force/torque into a column vector {@code DMatrix}. Here are a few
    * examples:
    * <ul>
    * <li>For a {@code RevoluteJoint}, the desired joint torque is stored at the 1<sup>st</sup> row.
    * <li>For a {@code PrismaticJoint}, the desired joint force is stored at the 1<sup>st</sup> row.
    * <li>For a {@code SixDoFJoint}, the desired wrench (the 3D torque and 3D force) of this joint is
    * stored from the {@code rowStart}<sup>th</sup> row to the ({@code rowStart + 5})<sup>th</sup> row,
    * starting with the three components of the torque. Note: the joint wrench is the wrench of
    * {@code successorFrame} expressed in {@code afterJointFrame}.
    * </ul>
    *
    * @param rowStart     row index for the first component of the force/torque.
    * @param matrixToPack the column vector in which the desired force/torque of this joint is stored.
    *                     Modified.
    * @return {@code rowStart + this.getDegreesOfFreedom()}.
    */
   int getJointTau(int rowStart, DMatrix matrixToPack);

   /**
    * Packs the offset from the frame before this joint to the frame after this parent joint.
    *
    * @param jointOffsetTransformToPack the transform in which this joint's offset is stored. Modified.
    */
   default void getJointOffset(RigidBodyTransform jointOffsetTransformToPack)
   {
      getFrameBeforeJoint().getTransformToParent(jointOffsetTransformToPack);
   }

   /**
    * Returns the number of degrees of freedom that this joint has.
    *
    * @return the number of degrees of freedom for this joint.
    */
   int getDegreesOfFreedom();

   /**
    * In most cases, this is the same as {@link #getDegreesOfFreedom()}. However, for
    * {@code SixDoFJoint} and {@code SphericalJoint} this method will return
    * {@code getDegreesOfFreedom() + 1}. This is due to the orientation, which has 3 degrees of
    * freedom, being represented as a quaternion which has 4 components.
    *
    * @return the size needed to pack this joint configuration into a matrix.
    */
   int getConfigurationMatrixSize();

   /**
    * Gets a new iterable to iterate through all the joints, including {@code this}, of the subtree
    * that starts at this joint.
    * <p>
    * A subtree is defined by a start joint and is the set of all the joint for which the start joint
    * is an ancestor.
    * </p>
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new subtree iterable.
    * @see JointIterable
    */
   default Iterable<? extends JointReadOnly> subtreeIterable()
   {
      return new JointIterable<>(JointReadOnly.class, null, this);
   }

   /**
    * Gets a new stream to go through all the joints, including {@code this}, of the subtree that
    * starts at this joint.
    * <p>
    * A subtree is defined by a start joint and is the set of all the joint for which the start joint
    * is an ancestor.
    * </p>
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new subtree stream.
    * @see SubtreeStreams
    */
   default Stream<? extends JointReadOnly> subtreeStream()
   {
      return SubtreeStreams.from(this);
   }

   /**
    * Gets a list that contains all the joints, including {@code this}, of the subtree that starts at
    * this joint.
    * <p>
    * A subtree is defined by a start joint and is the set of all the joint for which the start joint
    * is an ancestor.
    * </p>
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new subtree list.
    */
   default List<? extends JointReadOnly> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   /**
    * Gets an array that contains all the joints, including {@code this}, of the subtree that starts at
    * this joint.
    * <p>
    * A subtree is defined by a start joint and is the set of all the joint for which the start joint
    * is an ancestor.
    * </p>
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new subtree array.
    */
   default JointReadOnly[] subtreeArray()
   {
      return subtreeStream().toArray(JointReadOnly[]::new);
   }

   /**
    * Returns the reference to the name of this joint.
    *
    * @return the name of this joint.
    */
   String getName();

   /**
    * Gets the string that is used to identify this joint.
    * <p>
    * It contains the name of this joint and all ancestors up to the root joint.
    * </p>
    * <p>
    * This name ID is used to compute this joint {@link #hashCode()} and can be used for the
    * {@link #equals(Object)}.
    * </p>
    *
    * @return the identification name for this joint.
    */
   String getNameId();
}
