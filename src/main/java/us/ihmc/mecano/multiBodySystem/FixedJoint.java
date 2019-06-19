package us.ihmc.mecano.multiBodySystem;

import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FixedJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;

/**
 * A {@code FixedJoint} has no degrees of freedom, it does not move.
 *
 * @author Sylvain Bertrand
 */
public class FixedJoint implements FixedJointBasics
{
   /**
    * The name of this joint. Each joint of a robot should have a unique name, but it is not enforced
    * here.
    */
   protected final String name;
   /**
    * The identification name for this joint. It is composed of this joint name and all of its
    * ancestors.
    */
   private final String nameId;
   /**
    * The {@code RigidBody} directly connected to this joint and located between this joint and the
    * root body of the robot. The predecessor cannot be {@code null}.
    */
   protected final RigidBodyBasics predecessor;
   /**
    * The {@code RigidBody} directly connected to this joint and located between this joint and an
    * end-effector of the robot. The successor should not be {@code null}, but it is not enforced here.
    */
   protected RigidBodyBasics successor;
   /**
    * Reference frame which origin is located at this joint's origin.
    */
   protected final MovingReferenceFrame jointFrame;
   /** The twist of this joint, it is always set to zero. */
   private final TwistReadOnly jointTwist;
   /** The spatial acceleration of this joint, it is always set to zero. */
   private final SpatialAccelerationReadOnly jointAcceleration;
   /** The wrench of this joint, it is always set to zero. */
   private WrenchReadOnly jointWrench;

   /**
    * Creates a new fixed joint.
    * <p>
    * This constructor is typically used for creating a root floating joint, i.e. the first joint of
    * the multi-body system.
    * </p>
    * 
    * @param name        the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    */
   public FixedJoint(String name, RigidBodyBasics predecessor)
   {
      this(name, predecessor, null);
   }

   /**
    * Creates a new fixed joint.
    * 
    * @param name              the name for the new joint.
    * @param predecessor       the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    */
   public FixedJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
   {
      if (name.contains(NAME_ID_SEPARATOR))
         throw new IllegalArgumentException("A joint name can not contain '" + NAME_ID_SEPARATOR + "'. Tried to construct a jonit with name " + name + ".");

      this.name = name;
      this.predecessor = predecessor;
      jointFrame = MecanoFactories.newJointFrame(this, transformToParent, getName() + "Frame");

      if (predecessor.isRootBody())
         nameId = name;
      else
         nameId = predecessor.getParentJoint().getNameId() + NAME_ID_SEPARATOR + name;
      predecessor.addChildJoint(this);

      jointTwist = new Twist(jointFrame, jointFrame, jointFrame);
      jointAcceleration = new SpatialAcceleration(jointFrame, jointFrame, jointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public final MovingReferenceFrame getFrameBeforeJoint()
   {
      return jointFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final MovingReferenceFrame getFrameAfterJoint()
   {
      return jointFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final RigidBodyBasics getPredecessor()
   {
      return predecessor;
   }

   /** {@inheritDoc} */
   @Override
   public final RigidBodyBasics getSuccessor()
   {
      return successor;
   }

   /** {@inheritDoc} */
   @Override
   public final String getName()
   {
      return name;
   }

   /** {@inheritDoc} */
   @Override
   public String getNameId()
   {
      return nameId;
   }

   /** {@inheritDoc} */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      jointWrench = new Wrench(successorFrame, jointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public TwistReadOnly getJointTwist()
   {
      return jointTwist;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getJointAcceleration()
   {
      return jointAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public WrenchReadOnly getJointWrench()
   {
      return jointWrench;
   }

   /** {@inheritDoc} */
   @Override
   public List<TwistReadOnly> getUnitTwists()
   {
      return Collections.emptyList();
   }

   /**
    * Returns the implementation name of this joint and the joint name.
    */
   @Override
   public String toString()
   {
      return getClass().getSimpleName() + " " + getName();
   }

   /**
    * The hash code of a joint is based on its {@link #getNameId()}.
    *
    * @return the hash code of the {@link #getNameId()} of this joint.
    */
   @Override
   public int hashCode()
   {
      return nameId.hashCode();
   }
}
