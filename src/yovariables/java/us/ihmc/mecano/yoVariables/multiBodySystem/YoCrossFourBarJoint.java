package us.ihmc.mecano.yoVariables.multiBodySystem;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoCrossFourBarJoint extends CrossFourBarJoint
{
   private final YoDouble q, qd, qdd, tau;
   private final YoDouble jointLimitLower, jointLimitUpper;
   private final YoDouble velocityLimitLower, velocityLimitUpper;
   private final YoDouble effortLimitLower, effortLimitUpper;

   public YoCrossFourBarJoint(String name,
                              RigidBodyBasics predecessor,
                              String jointNameA,
                              String jointNameB,
                              String jointNameC,
                              String jointNameD,
                              String bodyNameDA,
                              String bodyNameBC,
                              RigidBodyTransformReadOnly transformAToPredecessor,
                              RigidBodyTransformReadOnly transformBToPredecessor,
                              RigidBodyTransformReadOnly transformCToB,
                              RigidBodyTransformReadOnly transformDToA,
                              Matrix3DReadOnly bodyInertiaDA,
                              Matrix3DReadOnly bodyInertiaBC,
                              double bodyMassDA,
                              double bodyMassBC,
                              RigidBodyTransformReadOnly bodyInertiaPoseDA,
                              RigidBodyTransformReadOnly bodyInertiaPoseBC,
                              int actuatedJointIndex,
                              int loopClosureJointIndex,
                              Vector3DReadOnly jointAxis,
                              YoRegistry registry)
   {
      super(name,
            predecessor,
            jointNameA,
            jointNameB,
            jointNameC,
            jointNameD,
            bodyNameDA,
            bodyNameBC,
            transformAToPredecessor,
            transformBToPredecessor,
            transformCToB,
            transformDToA,
            bodyInertiaDA,
            bodyInertiaBC,
            bodyMassDA,
            bodyMassBC,
            bodyInertiaPoseDA,
            bodyInertiaPoseBC,
            actuatedJointIndex,
            loopClosureJointIndex,
            jointAxis);

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

      q.addListener(v -> super.setQ(q.getValue()));
      qd.addListener(v -> super.setQd(qd.getValue()));
      qdd.addListener(v -> super.setQdd(qdd.getValue()));
      tau.addListener(v -> super.setTau(tau.getValue()));
      jointLimitLower.addListener(v -> super.setJointLimitLower(jointLimitLower.getValue()));
      jointLimitUpper.addListener(v -> super.setJointLimitUpper(jointLimitUpper.getValue()));
      velocityLimitLower.addListener(v -> super.setVelocityLimitLower(velocityLimitLower.getValue()));
      velocityLimitUpper.addListener(v -> super.setVelocityLimitUpper(velocityLimitUpper.getValue()));
      effortLimitLower.addListener(v -> super.setEffortLimitLower(effortLimitLower.getValue()));
      effortLimitUpper.addListener(v -> super.setEffortLimitUpper(effortLimitUpper.getValue()));
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

   @Override
   public void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
      super.setJointOrientation(jointOrientation);
      getQ(); // Update YoVariable
   }

   @Override
   public void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
      super.setJointAngularVelocity(jointAngularVelocity);
      getQd(); // Update YoVariable
   }

   @Override
   public void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
      super.setJointAngularAcceleration(jointAngularAcceleration);
      getQdd(); // Update YoVariable
   }

   @Override
   public void setJointTorque(Vector3DReadOnly jointTorque)
   {
      super.setJointTorque(jointTorque);
      getTau(); // Update YoVariable
   }

   @Override
   public void setJointLimitLower(double jointLimitLower)
   {
      this.jointLimitLower.set(jointLimitLower);
   }

   @Override
   public void setJointLimitUpper(double jointLimitUpper)
   {
      this.jointLimitUpper.set(jointLimitUpper);
   }

   @Override
   public void setVelocityLimitLower(double velocityLimitLower)
   {
      this.velocityLimitLower.set(velocityLimitLower);
   }

   @Override
   public void setVelocityLimitUpper(double velocityLimitUpper)
   {
      this.velocityLimitUpper.set(velocityLimitUpper);
   }

   @Override
   public void setEffortLimitLower(double effortLimitLower)
   {
      this.effortLimitLower.set(effortLimitLower);
   }

   @Override
   public void setEffortLimitUpper(double effortLimitUpper)
   {
      this.effortLimitUpper.set(effortLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public double getQ()
   {
      q.set(super.getQ(), false);
      return q.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getQd()
   {
      qd.set(super.getQd(), false);
      return qd.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getQdd()
   {
      qdd.set(super.getQdd(), false);
      return qdd.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getTau()
   {
      tau.set(super.getTau(), false);
      return tau.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitLower()
   {
      jointLimitLower.set(super.getJointLimitLower(), false);
      return jointLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitUpper()
   {
      jointLimitUpper.set(super.getJointLimitUpper(), false);
      return jointLimitUpper.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitLower()
   {
      velocityLimitLower.set(super.getVelocityLimitLower(), false);
      return velocityLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitUpper()
   {
      velocityLimitUpper.set(super.getVelocityLimitUpper(), false);
      return velocityLimitUpper.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitLower()
   {
      effortLimitLower.set(super.getEffortLimitLower(), false);
      return effortLimitLower.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitUpper()
   {
      effortLimitUpper.set(super.getEffortLimitUpper(), false);
      return effortLimitUpper.getValue();
   }
}
