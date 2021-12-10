package us.ihmc.mecano.yoVariables.multiBodySystem;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Implementation of a {@code CrossFourBarJoint} with internal state variables backed with
 * {@code YoVariable}s.
 * 
 * @see CrossFourBarJoint
 */
public class YoCrossFourBarJoint extends CrossFourBarJoint
{
   private final YoDouble q, qd, qdd, tau;
   private final YoDouble jointLimitLower, jointLimitUpper;
   private final YoDouble velocityLimitLower, velocityLimitUpper;
   private final YoDouble effortLimitLower, effortLimitUpper;

   /**
    * Creates a new cross four bar joint with the following structure:
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
    * <p>
    * Internally, this joint creates a kinematics that represents the above diagram using revolute
    * joints and rigid-bodies. The kinematics is isolated from the kinematics that this cross four bar
    * joint is connected to. This internal kinematics is only used to facilitates computation of this
    * joint properties.
    * </p>
    * <p>
    * Note that the mass properties of the two cross bars, i.e. rigid bodies DA and BC, are not used in
    * the rigid-body algorithms such as inverse and forward dynamics calculators.
    * </p>
    * 
    * @param name                    the name of this joint.
    * @param predecessor             the rigid-body connected to and preceding this joint.
    * @param jointNameA              the name of the joint A, see diagram above. Can be {@code null}.
    * @param jointNameB              the name of the internal joint B, see diagram above. Can be
    *                                {@code null}.
    * @param jointNameC              the name of the internal joint C, see diagram above. Can be
    *                                {@code null}.
    * @param jointNameD              the name of the internal joint D, see diagram above. Can be
    *                                {@code null}.
    * @param bodyNameDA              the name of the internal rigid body DA, i.e. cross bar, see
    *                                diagram above. Can be {@code null}.
    * @param bodyNameBC              the name of the internal rigid body BC, i.e. cross bar, see
    *                                diagram above. Can be {@code null}.
    * @param transformAToPredecessor the transform from the frame after the parent joint to the joint
    *                                A. Not modified.
    * @param transformBToPredecessor the transform from the frame after the parent joint to the joint
    *                                B. Not modified.
    * @param transformDToA           the transform from the frame after the joint A to the joint D. Not
    *                                modified.
    * @param transformCToB           the transform from the frame after the joint B to the joint C. Not
    *                                modified.
    * @param bodyInertiaDA           the 3D momentum inertia of the rigid-body DA. Can be {@code null}.
    * @param bodyInertiaBC           the 3D momentum inertia of the rigid-body BC. Can be {@code null}.
    * @param bodyMassDA              the mass of the rigid-body DA. Can be {@code 0.0}.
    * @param bodyMassBC              the mass of the rigid-body BC. Can be {@code 0.0}.
    * @param bodyInertiaPoseDA       the transform of the rigid-body DA's body-fixed frame. Can be
    *                                {@code null}.
    * @param bodyInertiaPoseBC       the transform of the rigid-body BC's body-fixed frame. Can be
    *                                {@code null}.
    * @param actuatedJointIndex      the index of the joint that is actuated, i.e. torque source. 0 is
    *                                for joint A, 1 for B, 2 for C, and 3 for D.
    * @param loopClosureJointIndex   the index of the joint is constructed as the loop closure of the
    *                                internal mechanism. Can be 2 for joint C or 3 for joint D. It has
    *                                to be different from {@code actuatedJointIndex}. The frame after
    *                                the other joint is used as the frame after this cross four bar
    *                                joint.
    * @param jointAxis               the axis of this joint. It is also used to define the axis of the
    *                                internal revolute joints.
    * @param registry                the registry to register child variables to.
    */
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
                              RigidBodyTransformReadOnly transformDToA,
                              RigidBodyTransformReadOnly transformCToB,
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
      this(name,
           predecessor,
           jointNameA,
           jointNameB,
           jointNameC,
           jointNameD,
           bodyNameDA,
           bodyNameBC,
           transformAToPredecessor,
           transformBToPredecessor,
           transformDToA,
           transformCToB,
           bodyInertiaDA,
           bodyInertiaBC,
           bodyMassDA,
           bodyMassBC,
           bodyInertiaPoseDA,
           bodyInertiaPoseBC,
           MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER,
           actuatedJointIndex,
           loopClosureJointIndex,
           jointAxis,
           registry);
   }

   /**
    * Creates a new cross four bar joint with the following structure:
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
    * <p>
    * Internally, this joint creates a kinematics that represents the above diagram using revolute
    * joints and rigid-bodies. The kinematics is isolated from the kinematics that this cross four bar
    * joint is connected to. This internal kinematics is only used to facilitates computation of this
    * joint properties.
    * </p>
    * <p>
    * Note that the mass properties of the two cross bars, i.e. rigid bodies DA and BC, are not used in
    * the rigid-body algorithms such as inverse and forward dynamics calculators.
    * </p>
    * 
    * @param name                    the name of this joint.
    * @param predecessor             the rigid-body connected to and preceding this joint.
    * @param jointNameA              the name of the joint A, see diagram above. Can be {@code null}.
    * @param jointNameB              the name of the internal joint B, see diagram above. Can be
    *                                {@code null}.
    * @param jointNameC              the name of the internal joint C, see diagram above. Can be
    *                                {@code null}.
    * @param jointNameD              the name of the internal joint D, see diagram above. Can be
    *                                {@code null}.
    * @param bodyNameDA              the name of the internal rigid body DA, i.e. cross bar, see
    *                                diagram above. Can be {@code null}.
    * @param bodyNameBC              the name of the internal rigid body BC, i.e. cross bar, see
    *                                diagram above. Can be {@code null}.
    * @param transformAToPredecessor the transform from the frame after the parent joint to the joint
    *                                A. Not modified.
    * @param transformBToPredecessor the transform from the frame after the parent joint to the joint
    *                                B. Not modified.
    * @param transformDToA           the transform from the frame after the joint A to the joint D. Not
    *                                modified.
    * @param transformCToB           the transform from the frame after the joint B to the joint C. Not
    *                                modified.
    * @param bodyInertiaDA           the 3D momentum inertia of the rigid-body DA. Can be {@code null}.
    * @param bodyInertiaBC           the 3D momentum inertia of the rigid-body BC. Can be {@code null}.
    * @param bodyMassDA              the mass of the rigid-body DA. Can be {@code 0.0}.
    * @param bodyMassBC              the mass of the rigid-body BC. Can be {@code 0.0}.
    * @param bodyInertiaPoseDA       the transform of the rigid-body DA's body-fixed frame. Can be
    *                                {@code null}.
    * @param bodyInertiaPoseBC       the transform of the rigid-body BC's body-fixed frame. Can be
    *                                {@code null}.
    * @param rigidBodyBuilder        the factory used to construct the two cross bars, i.e.
    *                                rigid-bodies DA and BC.
    * @param actuatedJointIndex      the index of the joint that is actuated, i.e. torque source. 0 is
    *                                for joint A, 1 for B, 2 for C, and 3 for D.
    * @param loopClosureJointIndex   the index of the joint is constructed as the loop closure of the
    *                                internal mechanism. Can be 2 for joint C or 3 for joint D. It has
    *                                to be different from {@code actuatedJointIndex}. The frame after
    *                                the other joint is used as the frame after this cross four bar
    *                                joint.
    * @param jointAxis               the axis of this joint. It is also used to define the axis of the
    *                                internal revolute joints.
    * @param registry                the registry to register child variables to.
    */
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
                              RigidBodyTransformReadOnly transformDToA,
                              RigidBodyTransformReadOnly transformCToB,
                              Matrix3DReadOnly bodyInertiaDA,
                              Matrix3DReadOnly bodyInertiaBC,
                              double bodyMassDA,
                              double bodyMassBC,
                              RigidBodyTransformReadOnly bodyInertiaPoseDA,
                              RigidBodyTransformReadOnly bodyInertiaPoseBC,
                              RigidBodyBuilder rigidBodyBuilder,
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
            transformDToA,
            transformCToB,
            bodyInertiaDA,
            bodyInertiaBC,
            bodyMassDA,
            bodyMassBC,
            bodyInertiaPoseDA,
            bodyInertiaPoseBC,
            rigidBodyBuilder,
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

   /** {@inheritDoc} */
   @Override
   public void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
      super.setJointOrientation(jointOrientation);
      getQ(); // Update YoVariable
   }

   /** {@inheritDoc} */
   @Override
   public void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
      super.setJointAngularVelocity(jointAngularVelocity);
      getQd(); // Update YoVariable
   }

   /** {@inheritDoc} */
   @Override
   public void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
      super.setJointAngularAcceleration(jointAngularAcceleration);
      getQdd(); // Update YoVariable
   }

   /** {@inheritDoc} */
   @Override
   public void setJointTorque(Vector3DReadOnly jointTorque)
   {
      super.setJointTorque(jointTorque);
      getTau(); // Update YoVariable
   }

   /** {@inheritDoc} */
   @Override
   public void setJointLimitLower(double jointLimitLower)
   {
      this.jointLimitLower.set(jointLimitLower);
   }

   /** {@inheritDoc} */
   @Override
   public void setJointLimitUpper(double jointLimitUpper)
   {
      this.jointLimitUpper.set(jointLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public void setVelocityLimitLower(double velocityLimitLower)
   {
      this.velocityLimitLower.set(velocityLimitLower);
   }

   /** {@inheritDoc} */
   @Override
   public void setVelocityLimitUpper(double velocityLimitUpper)
   {
      this.velocityLimitUpper.set(velocityLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   public void setEffortLimitLower(double effortLimitLower)
   {
      this.effortLimitLower.set(effortLimitLower);
   }

   /** {@inheritDoc} */
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
