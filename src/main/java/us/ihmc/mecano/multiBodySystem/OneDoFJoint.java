package us.ihmc.mecano.multiBodySystem;

import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;

/**
 * Base implementation for joints with a single degree of freedom (DoF).
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
public abstract class OneDoFJoint extends Joint implements OneDoFJointBasics
{
   /**
    * This joint twist. Note that this field cannot be used to actually changed this joint velocity.
    * Instead, this field is automatically updated when {@link #qd} changes.
    */
   private final TwistReadOnly jointTwist;
   /**
    * This joint unit-twist. In the current framework, the unit-twist is calculated once at
    * construction and remains constant.
    */
   private final TwistReadOnly unitJointTwist;
   /** A 1-element list containing this joint unit-twist. */
   private final List<TwistReadOnly> unitTwists;
   /**
    * This joint unit-twist in different frames. In the current framework, the unit-twist is
    * calculated once at construction and remains constant.
    */
   private TwistReadOnly unitSuccessorTwist, unitPredecessorTwist;

   /**
    * This joint spatial acceleration. Note that this field cannot be used to actually changed this
    * joint acceleration. Instead, this field is automatically updated when {@link #qdd} changes.
    */
   private final SpatialAccelerationReadOnly jointAcceleration;
   /**
    * This joint unit spatial acceleration. In the current framework, the unit spatial acceleration
    * is calculated once at construction and remains constant.
    */
   private final SpatialAccelerationReadOnly unitJointAcceleration;
   /**
    * This joint unit spatial acceleration in different frames. In the current framework, the unit
    * spatial acceleration is calculated once at construction and remains constant.
    */
   private SpatialAccelerationReadOnly unitSuccessorAcceleration, unitPredecessorAcceleration;

   /**
    * This joint resulting wrench on its successor. Note that this field cannot be used to actually
    * changed this joint force/torque. Instead, this field is automatically updated when
    * {@link #tau} changes.
    */
   private WrenchReadOnly jointWrench;
   /**
    * This joint unit-wrench. In the current framework, the unit-wrench is calculated once at
    * construction and remains constant.
    */
   private WrenchReadOnly unitJointWrench;

   /** This joint current position/angle. */
   private double q;
   /** This joint current velocity. */
   private double qd;
   /** This joint current acceleration. */
   private double qdd;
   /** This joint current force/torque. */
   private double tau;

   /**
    * The minimum value {@link #q} can have:
    * 
    * <pre>
    * this.q &in; [this.jointLimitLower; this.jointLimitUpper]
    * </pre>
    * 
    * It is initialized to -&infin;.
    */
   private double jointLimitLower = Double.NEGATIVE_INFINITY;
   /**
    * The maximum value {@link #q} can have:
    * 
    * <pre>
    * this.q &in; [this.jointLimitLower; this.jointLimitUpper]
    * </pre>
    * 
    * It is initialized to +&infin;.
    */
   private double jointLimitUpper = Double.POSITIVE_INFINITY;
   /**
    * The minimum value {@link #qd} can have:
    * 
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * 
    * It is initialized to -&infin;.
    */
   private double velocityLimitLower = Double.NEGATIVE_INFINITY;
   /**
    * The maximum value {@link #qd} can have:
    * 
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * 
    * It is initialized to +&infin;.
    */
   private double velocityLimitUpper = Double.POSITIVE_INFINITY;
   /**
    * The minimum value {@link #tau} can have:
    * 
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * 
    * It is initialized to -&infin;.
    */
   private double effortLimitLower = Double.NEGATIVE_INFINITY;
   /**
    * The maximum value {@link #tau} can have:
    * 
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * 
    * It is initialized to +&infin;.
    */
   private double effortLimitUpper = Double.POSITIVE_INFINITY;

   /**
    * Creates the abstract layer for a new 1-DoF joint.
    * 
    * @param name the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param jointAxisAngularPart the unit-vector if this joint is a revolute joint. A zero vector
    *           otherwise. Not modified.
    * @param jointAxisLinearPart the unit-vector if this joint is a prismatic joint. A zero vector
    *           otherwise. Not modified.
    * @param transformToParent the transform from this joint to the {@code frameAfterJoint} of its
    *           parent joint. Not modified.
    */
   public OneDoFJoint(String name, RigidBodyBasics predecessor, Vector3DReadOnly jointAxisAngularPart, Vector3DReadOnly jointAxisLinearPart,
                      RigidBodyTransform transformToParent)
   {
      super(name, predecessor, transformToParent);

      unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, jointAxisAngularPart, jointAxisLinearPart);
      unitTwists = Collections.singletonList(unitJointTwist);
      jointTwist = MecanoFactories.newTwistReadOnly(this::getQd, unitJointTwist);
      unitJointAcceleration = new SpatialAcceleration(unitJointTwist);
      jointAcceleration = MecanoFactories.newSpatialAccelerationVectorReadOnly(this::getQdd, unitJointAcceleration);
   }

   /** {@inheritDoc} */
   @Override
   public final void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;

      ReferenceFrame predecessorFrame = getPredecessor().getBodyFixedFrame();
      ReferenceFrame successorFrame = getSuccessor().getBodyFixedFrame();

      {
         Twist twist = new Twist(unitJointTwist);
         twist.setBaseFrame(predecessorFrame);
         twist.setBodyFrame(successorFrame);
         twist.changeFrame(successorFrame);
         unitSuccessorTwist = twist;
      }

      {
         Twist twist = new Twist(unitSuccessorTwist);
         twist.invert();
         twist.changeFrame(predecessorFrame);
         unitPredecessorTwist = twist;
      }

      {
         SpatialAcceleration acceleration = new SpatialAcceleration(unitJointAcceleration);
         acceleration.setBaseFrame(predecessorFrame);
         acceleration.setBodyFrame(successorFrame);
         acceleration.changeFrame(successorFrame);
         unitSuccessorAcceleration = acceleration;
      }

      {
         SpatialAcceleration acceleration = new SpatialAcceleration(unitSuccessorAcceleration);
         acceleration.invert();
         acceleration.changeFrame(predecessorFrame); // actually, there is relative motion, but not in the directions that matter
         unitPredecessorAcceleration = acceleration;

      }

      {
         Wrench wrench = new Wrench();
         wrench.setIncludingFrame(unitJointTwist);
         wrench.setBodyFrame(successorFrame);
         wrench.changeFrame(afterJointFrame);
         unitJointWrench = wrench;
      }

      jointWrench = MecanoFactories.newWrenchReadOnly(this::getTau, unitJointWrench);
   }

   /** {@inheritDoc} */
   @Override
   public void setQ(double q)
   {
      this.q = q;
   }

   /** {@inheritDoc} */
   @Override
   public void setQd(double qd)
   {
      this.qd = qd;
   }

   /** {@inheritDoc} */
   @Override
   public void setQdd(double qdd)
   {
      this.qdd = qdd;
   }

   /** {@inheritDoc} */
   @Override
   public void setTau(double tau)
   {
      this.tau = tau;
   }

   /** {@inheritDoc} */
   @Override
   public void setJointLimits(double jointLimitLower, double jointLimitUpper)
   {
      this.jointLimitLower = jointLimitLower;
      this.jointLimitUpper = jointLimitUpper;
   }

   /** {@inheritDoc} */
   @Override
   public void setVelocityLimits(double velocityLimitLower, double velocityLimitUpper)
   {
      this.velocityLimitLower = velocityLimitLower;
      this.velocityLimitUpper = velocityLimitUpper;
   }

   /** {@inheritDoc} */
   @Override
   public void setEffortLimits(double effortLimitLower, double effortLimitUpper)
   {
      this.effortLimitLower = effortLimitLower;
      this.effortLimitUpper = effortLimitUpper;
   }

   /** {@inheritDoc} */
   @Override
   public TwistReadOnly getJointTwist()
   {
      return jointTwist;
   }

   /** {@inheritDoc} */
   @Override
   public TwistReadOnly getUnitJointTwist()
   {
      return unitJointTwist;
   }

   /** {@inheritDoc} */
   @Override
   public List<TwistReadOnly> getUnitTwists()
   {
      return unitTwists;
   }

   /** {@inheritDoc} */
   @Override
   public TwistReadOnly getUnitSuccessorTwist()
   {
      return unitSuccessorTwist;
   }

   /** {@inheritDoc} */
   @Override
   public TwistReadOnly getUnitPredecessorTwist()
   {
      return unitPredecessorTwist;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getJointAcceleration()
   {
      return jointAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getUnitJointAcceleration()
   {
      return unitJointAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getUnitSuccessorAcceleration()
   {
      return unitSuccessorAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getUnitPredecessorAcceleration()
   {
      return unitPredecessorAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public WrenchReadOnly getJointWrench()
   {
      return jointWrench;
   }

   /** {@inheritDoc} */
   @Override
   public double getQ()
   {
      return q;
   }

   /** {@inheritDoc} */
   @Override
   public double getQd()
   {
      return qd;
   }

   /** {@inheritDoc} */
   @Override
   public double getQdd()
   {
      return qdd;
   }

   /** {@inheritDoc} */
   @Override
   public double getTau()
   {
      return tau;
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitLower()
   {
      return jointLimitLower;
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitUpper()
   {
      return jointLimitUpper;
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitLower()
   {
      return velocityLimitLower;
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitUpper()
   {
      return velocityLimitUpper;
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitLower()
   {
      return effortLimitLower;
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitUpper()
   {
      return effortLimitUpper;
   }

   @Override
   public String toString()
   {
      String qAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQ());
      String qdAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQd());
      String qddAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQdd());
      String tauAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getTau());
      return super.toString() + ", q: " + qAsString + ", qd: " + qdAsString + ", qdd: " + qddAsString + ", tau: " + tauAsString;
   }
}