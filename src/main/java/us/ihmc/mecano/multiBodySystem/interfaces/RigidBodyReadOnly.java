package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.RigidBodyIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

/**
 * Read-only interface for a rigid-body.
 * <p>
 * A rigid-body describes a link which used with {@code Joint}s to describe a multi-body system.
 * <p>
 * This class gathers notably the physical properties of the link and its position in the system,
 * i.e. its parent and children joints.
 * </p>
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface RigidBodyReadOnly
{
   /** A string used to separate joint names in the {@link #getNameId()}. */
   static final String NAME_ID_SEPARATOR = JointReadOnly.NAME_ID_SEPARATOR;

   /**
    * Gets the read-only reference to the spatial inertia of this rigid-body.
    *
    * @return the reference to this rigid-body's inertia.
    */
   SpatialInertiaReadOnly getInertia();

   /**
    * Gets the reference to this rigid-body's body-fixed frame.
    * <p>
    * The main property of the body-fixed-frame is that it is rigidly attached to this rigid-body.
    * Unless it has been changed at runtime, the body-fixed frame is also centered at this rigid-body's
    * center of mass position.
    * </p>
    *
    * @return the reference frame attached to this rigid-body.
    */
   MovingReferenceFrame getBodyFixedFrame();

   /**
    * Gets the reference to the parent joint of this rigid-body.
    * <p>
    * The parent joint is the joint directly connected to this rigid-body and located between this and
    * the root body of the robot.
    * </p>
    *
    * @return the reference to the parent joint or {@code null} if this rigid-body is a root body.
    */
   JointReadOnly getParentJoint();

   /**
    * Gets the view list of all the children joints that have been registered to this rigid-body.
    * <p>
    * A child joint is a joint directly connected to this rigid-body and located between this and an
    * end-effector of the robot. This list is empty when this rigid-body is an end-effector.
    * </p>
    *
    * @return all the children joints of this rigid-body.
    */
   List<? extends JointReadOnly> getChildrenJoints();

   /**
    * Verifies whether this rigid-body has at least one child joint or not.
    * <p>
    * A rigid-body without any children joint is usually referred to as an end-effector.
    * </p>
    *
    * @return {@code true} if this rigid-body has at least one child joint, {@code false} otherwise.
    */
   default boolean hasChildrenJoints()
   {
      return !getChildrenJoints().isEmpty();
   }

   /**
    * Verifies whether this rigid-body is the root body of a robot.
    * <p>
    * Internally, this verifies if this rigid-body has a parent joint. By definition, the root body of
    * a robot does not have any parent joint.
    * </p>
    *
    * @return {@code true} if this is the root body, {@code false} otherwise.
    */
   default boolean isRootBody()
   {
      return getParentJoint() == null;
   }

   /**
    * Gets the name of this rigid-body.
    * <p>
    * Each rigid-body of a robot should have a unique name.
    * </p>
    *
    * @return this rigid-body's name.
    */
   String getName();

   /**
    * Packs this rigid-body's center of mass coordinates in the given argument.
    * <p>
    * Note that unless modified in runtime, this method outputs {@code (0, 0, 0)} in the
    * body-fixed-frame. The body-fixed-frame is centered at this rigid-body's center of mass position
    * by default.
    * </p>
    *
    * @param centerOfMassToPack the {@code FramePoint} in which the center of mass position is stored.
    *                           Modified.
    */
   default void getCenterOfMass(FramePoint3DBasics centerOfMassToPack)
   {
      centerOfMassToPack.setIncludingFrame(getInertia().getCenterOfMassOffset());
   }

   /**
    * Gets a new iterable to iterate through all the rigid-bodies, including {@code this}, of the
    * subtree that starts at this rigid-body.
    * <p>
    * A subtree is defined by a start rigid-body and is the set of all the rigid-body for which the
    * start rigid-body is an ancestor.
    * </p>
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new subtree iterable.
    * @see RigidBodyIterable
    */
   default Iterable<? extends RigidBodyReadOnly> subtreeIterable()
   {
      return new RigidBodyIterable<>(RigidBodyReadOnly.class, null, this);
   }

   /**
    * Gets a new iterable to iterate over the joints that compose the subtree starting from the
    * children of this body.
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new joint subtree iterable.
    * @see JointIterable
    */
   default Iterable<? extends JointReadOnly> childrenSubtreeIterable()
   {
      return new JointIterable<>(JointReadOnly.class, null, getChildrenJoints());
   }

   /**
    * Gets a new stream to go through all the rigid-bodies, including {@code this}, of the subtree that
    * starts at this rigid-body.
    * <p>
    * A subtree is defined by a start rigid-body and is the set of all the rigid-body for which the
    * start rigid-body is an ancestor.
    * </p>
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new subtree stream.
    * @see SubtreeStreams
    */
   default Stream<? extends RigidBodyReadOnly> subtreeStream()
   {
      return SubtreeStreams.from(this);
   }

   /**
    * Gets a list that contains all the rigid-bodies, including {@code this}, of the subtree that
    * starts at this rigid-body.
    * <p>
    * A subtree is defined by a start rigid-body and is the set of all the rigid-body for which the
    * start rigid-body is an ancestor.
    * </p>
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new subtree list.
    */
   default List<? extends RigidBodyReadOnly> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   /**
    * Gets an array that contains all the rigid-bodies, including {@code this}, of the subtree that
    * starts at this rigid-body.
    * <p>
    * A subtree is defined by a start rigid-body and is the set of all the rigid-body for which the
    * start rigid-body is an ancestor.
    * </p>
    * <p>
    * This method generates garbage.
    * </p>
    *
    * @return the new subtree array.
    */
   default RigidBodyReadOnly[] subtreeArray()
   {
      return subtreeStream().toArray(RigidBodyReadOnly[]::new);
   }

   /**
    * Gets the string that is used to identify this rigid-body.
    * <p>
    * It contains the name of this rigid-body and all ancestors up to the root body.
    * </p>
    * <p>
    * This name ID is used to compute this rigid-body {@link #hashCode()} and can be used for the
    * {@link #equals(Object)}.
    * </p>
    *
    * @return the identification name for this rigid-body.
    */
   String getNameId();
}
