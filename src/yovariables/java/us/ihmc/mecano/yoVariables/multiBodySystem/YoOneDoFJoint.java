package us.ihmc.mecano.yoVariables.multiBodySystem;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Extension of {@code OneDoFJoint} for 1-DoF joints which state is backed by {@code YoVariable}s.
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public abstract class YoOneDoFJoint extends OneDoFJoint
{
   private final YoDouble q, qd, qdd, tau;
   private final YoDouble jointLimitLower, jointLimitUpper;
   private final YoDouble velocityLimitLower, velocityLimitUpper;
   private final YoDouble effortLimitLower, effortLimitUpper;

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
   public YoOneDoFJoint(String name, RigidBodyBasics predecessor, Vector3DReadOnly jointAxisAngularPart, Vector3DReadOnly jointAxisLinearPart,
                        RigidBodyTransform transformToParent, YoVariableRegistry registry)
   {
      super(name, predecessor, jointAxisAngularPart, jointAxisLinearPart, transformToParent);

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
   }

   /** {@inheritDoc} */
   @Override
   public void setQ(double q)
   {
      this.q.set(q);
   }

   /** {@inheritDoc} */
   @Override
   public void setQd(double qd)
   {
      this.qd.set(qd);
   }

   /** {@inheritDoc} */
   @Override
   public void setQdd(double qdd)
   {
      this.qdd.set(qdd);
   }

   /** {@inheritDoc} */
   @Override
   public void setTau(double tau)
   {
      this.tau.set(tau);
   }

   /** {@inheritDoc} */
   @Override
   public void setJointLimits(double jointLimitLower, double jointLimitUpper)
   {
      this.jointLimitLower.set(jointLimitLower);
      this.jointLimitUpper.set(jointLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public void setVelocityLimits(double velocityLimitLower, double velocityLimitUpper)
   {
      this.velocityLimitLower.set(velocityLimitLower);
      this.velocityLimitUpper.set(velocityLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public void setEffortLimits(double effortLimitLower, double effortLimitUpper)
   {
      this.effortLimitLower.set(effortLimitLower);
      this.effortLimitUpper.set(effortLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public double getQ()
   {
      return q.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getQd()
   {
      return qd.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getQdd()
   {
      return qdd.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getTau()
   {
      return tau.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitLower()
   {
      return jointLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitUpper()
   {
      return jointLimitUpper.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitLower()
   {
      return velocityLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitUpper()
   {
      return velocityLimitUpper.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitLower()
   {
      return effortLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitUpper()
   {
      return effortLimitUpper.getValue();
   }
}
