package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.JointIterable;
import us.ihmc.mecano.multiBodySystem.iterators.RigidBodyIterable;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

/**
 * Write and read interface for a rigid-body.
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
public interface RigidBodyBasics extends RigidBodyReadOnly
{
   /**
    * Gets the internal reference to the object to contains the information about this rigid-body
    * physical properties.
    *
    * @return the reference to this rigid-body's inertia.
    */
   @Override
   SpatialInertiaBasics getInertia();

   @Override
   JointBasics getParentJoint();

   /**
    * Registers a new parent joint that closes a kinematic loop to this rigid-body.
    * <p>
    * This method should only be called when building the robot.
    * </p>
    * 
    * @param parentLoopClosureJoint the new parent joint to register to this body.
    */
   default void addParentLoopClosureJoint(JointBasics parentLoopClosureJoint)
   {
      getParentLoopClosureJoints().add(parentLoopClosureJoint);
   }

   @Override
   default List<JointBasics> getParentLoopClosureJoints()
   {
      return Collections.emptyList();
   }

   /**
    * Registers a new child joint to this rigid-body.
    * <p>
    * This method should only be called when building the robot.
    * </p>
    * <p>
    * A child joint is a joint directly connected to this rigid-body and located between this and an
    * end-effector of the robot.
    * </p>
    *
    * @param joint the new child joint to register to this rigid-body.
    */
   default void addChildJoint(JointBasics joint)
   {
      getChildrenJoints().add(joint);
   }

   @Override
   List<JointBasics> getChildrenJoints();

   /**
    * Changes the position of this rigid-body's center of mass.
    * <p>
    * Note that the center of mass position should only be set at construction time and that changing
    * it at runtime might result in undesirable effects as some modules may not anticipate such a
    * change or may not consider the case where the body-fixed-frame is not centered at the center of
    * mass position.
    * </p>
    *
    * @param centerOfMass the new coordinates of this rigid-body's center of mass position. Not
    *                     modified.
    */
   default void setCenterOfMass(FramePoint3DReadOnly centerOfMass)
   {
      getInertia().getCenterOfMassOffset().setMatchingFrame(centerOfMass);
   }

   /**
    * Request all the children joints of this rigid-body to update their reference frame and request
    * their own successor to call this same method.
    * <p>
    * By calling this method on the root body of a robot, it will update the reference frames of all
    * the robot's joints.
    * </p>
    */
   default void updateFramesRecursively()
   {
      getBodyFixedFrame().update();

      for (int childIndex = 0; childIndex < getChildrenJoints().size(); childIndex++)
      {
         getChildrenJoints().get(childIndex).updateFramesRecursively();
      }
   }

   @Override
   default Iterable<? extends RigidBodyBasics> subtreeIterable()
   {
      return new RigidBodyIterable<>(RigidBodyBasics.class, null, null, this);
   }

   @Override
   default Iterable<? extends JointBasics> childrenSubtreeIterable()
   {
      return new JointIterable<>(JointBasics.class, null, null, getChildrenJoints());
   }

   @Override
   default Stream<? extends RigidBodyBasics> subtreeStream()
   {
      return SubtreeStreams.from(this);
   }

   @Override
   default List<? extends RigidBodyBasics> subtreeList()
   {
      return subtreeStream().collect(Collectors.toList());
   }

   @Override
   default RigidBodyBasics[] subtreeArray()
   {
      return subtreeStream().toArray(RigidBodyBasics[]::new);
   }
}
