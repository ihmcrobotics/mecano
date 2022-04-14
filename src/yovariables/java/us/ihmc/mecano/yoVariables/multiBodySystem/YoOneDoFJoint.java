package us.ihmc.mecano.yoVariables.multiBodySystem;

import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.interfaces.YoOneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Extension of {@code OneDoFJoint} for 1-DoF joints which state is backed by {@code YoVariable}s.
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public abstract class YoOneDoFJoint extends Joint implements YoOneDoFJointBasics
{
   private final YoDouble q, qd, qdd, tau;
   private final YoDouble jointLimitLower, jointLimitUpper;
   private final YoDouble velocityLimitLower, velocityLimitUpper;
   private final YoDouble effortLimitLower, effortLimitUpper;

   // See OneDoFJoint for additional documentation
   private final TwistReadOnly jointTwist;
   private final TwistReadOnly unitJointTwist;
   private final List<TwistReadOnly> unitTwists;
   private TwistReadOnly unitSuccessorTwist, unitPredecessorTwist;
   private final SpatialAccelerationReadOnly jointAcceleration;
   private final SpatialAccelerationReadOnly unitJointAcceleration;
   private SpatialAccelerationReadOnly unitSuccessorAcceleration, unitPredecessorAcceleration;
   private WrenchReadOnly jointWrench;
   private WrenchReadOnly unitJointWrench;

   /**
    * Creates the abstract layer for a new 1-DoF joint backed by {@code YoVariable}s.
    *
    * @param name                 the name for the new joint.
    * @param predecessor          the rigid-body connected to and preceding this joint.
    * @param jointAxisAngularPart the unit-vector if this joint is a revolute joint. A zero vector
    *                             otherwise. Not modified.
    * @param jointAxisLinearPart  the unit-vector if this joint is a prismatic joint. A zero vector
    *                             otherwise. Not modified.
    * @param transformToParent    the transform from this joint to the {@code frameAfterJoint} of its
    *                             parent joint. Not modified.
    * @param registry             the registry to register child variables to.
    */
   public YoOneDoFJoint(String name,
                        RigidBodyBasics predecessor,
                        Vector3DReadOnly jointAxisAngularPart,
                        Vector3DReadOnly jointAxisLinearPart,
                        RigidBodyTransformReadOnly transformToParent,
                        YoRegistry registry)
   {
      super(name, predecessor, transformToParent);

      q = new YoDouble("q_" + getName(), registry);
      qd = new YoDouble("qd_" + getName(), registry);
      qdd = new YoDouble("qdd_" + getName(), registry);
      tau = new YoDouble("tau_" + getName(), registry);
      jointLimitLower = new YoDouble("q_min_" + getName(), registry);
      jointLimitUpper = new YoDouble("q_max_" + getName(), registry);
      velocityLimitLower = new YoDouble("qd_min_" + getName(), registry);
      velocityLimitUpper = new YoDouble("qd_max_" + getName(), registry);
      effortLimitLower = new YoDouble("tau_min_" + getName(), registry);
      effortLimitUpper = new YoDouble("tau_max_" + getName(), registry);
      jointLimitLower.set(Double.NEGATIVE_INFINITY);
      jointLimitUpper.set(Double.POSITIVE_INFINITY);
      velocityLimitLower.set(Double.NEGATIVE_INFINITY);
      velocityLimitUpper.set(Double.POSITIVE_INFINITY);
      effortLimitLower.set(Double.NEGATIVE_INFINITY);
      effortLimitUpper.set(Double.POSITIVE_INFINITY);

      unitJointTwist = new Twist(afterJointFrame, beforeJointFrame, afterJointFrame, jointAxisAngularPart, jointAxisLinearPart);
      unitTwists = Collections.singletonList(unitJointTwist);
      jointTwist = MecanoFactories.newTwistReadOnly(this::getQd, unitJointTwist);
      unitJointAcceleration = new SpatialAcceleration(unitJointTwist);
      jointAcceleration = MecanoFactories.newSpatialAccelerationVectorReadOnly(this::getQdd, unitJointAcceleration);
   }

   /** {@inheritDoc} */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      unitSuccessorTwist = MecanoFactories.newOneDoFJointUnitSuccessorTwist(this);
      unitPredecessorTwist = MecanoFactories.newOneDoFJointUnitPredecessorTwist(this);
      unitSuccessorAcceleration = MecanoFactories.newOneDoFJointUnitSuccessorAcceleration(this);
      unitPredecessorAcceleration = MecanoFactories.newOneDoFJointUnitPredecessorAcceleration(this);
      unitJointWrench = MecanoFactories.newOneDoFJointUnitJointWrench(this);
      jointWrench = MecanoFactories.newWrenchReadOnly(this::getTau, unitJointWrench);
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

   /**
    * Gets the yoVariable used to store this joint position.
    *
    * @return the position's yoVariable.
    */
   @Override
   public YoDouble getYoQ()
   {
      return q;
   }

   /**
    * Gets the yoVariable used to store this joint velocity.
    *
    * @return the velocity's yoVariable.
    */
   @Override
   public YoDouble getYoQd()
   {
      return qd;
   }

   /**
    * Gets the yoVariable used to store this joint acceleration.
    *
    * @return the acceleration's yoVariable.
    */
   @Override
   public YoDouble getYoQdd()
   {
      return qdd;
   }

   /**
    * Gets the yoVariable used to store this joint force/torque.
    *
    * @return the force/torque's yoVariable.
    */
   @Override
   public YoDouble getYoTau()
   {
      return tau;
   }

   /**
    * Gets the yoVariable used to store this joint lower limit.
    *
    * @return the lower limit's yoVariable.
    */
   @Override
   public YoDouble getYoJointLimitLower()
   {
      return jointLimitLower;
   }

   /**
    * Gets the yoVariable used to store this joint upper limit.
    *
    * @return the upper limit's yoVariable.
    */
   @Override
   public YoDouble getYoJointLimitUpper()
   {
      return jointLimitUpper;
   }

   /**
    * Gets the yoVariable used to store this joint's velocity lower limit.
    *
    * @return the velocity lower limit's yoVariable.
    */
   @Override
   public YoDouble getYoVelocityLimitLower()
   {
      return velocityLimitLower;
   }

   /**
    * Gets the yoVariable used to store this joint's velocity upper limit.
    *
    * @return the velocity upper limit's yoVariable.
    */
   @Override
   public YoDouble getYoVelocityLimitUpper()
   {
      return velocityLimitUpper;
   }

   /**
    * Gets the yoVariable used to store this joint's force/torque lower limit.
    *
    * @return the force/torque lower limit's yoVariable.
    */
   @Override
   public YoDouble getYoEffortLimitLower()
   {
      return effortLimitLower;
   }

   /**
    * Gets the yoVariable used to store this joint's force/torque upper limit.
    *
    * @return the force/torque upper limit's yoVariable.
    */
   @Override
   public YoDouble getYoEffortLimitUpper()
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
