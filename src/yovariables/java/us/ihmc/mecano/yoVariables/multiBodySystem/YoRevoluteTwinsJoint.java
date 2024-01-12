package us.ihmc.mecano.yoVariables.multiBodySystem;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteTwinsJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteTwinsJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.interfaces.YoOneDoFJointBasics;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Collections;
import java.util.List;

/**
 * Implementation of a {@code RevoluteTwinsJoint} backed by {@code YoVariable}s.
 *
 * @see RevoluteTwinsJoint
 */
public class YoRevoluteTwinsJoint implements RevoluteTwinsJointBasics, YoOneDoFJointBasics
{
   private final YoDouble q, qd, qdd, tau;

   private final String name;
   private final String nameId;
   private final RigidBodyBasics predecessor;
   private RigidBodyBasics successor;
   private final MovingReferenceFrame beforeJointFrame;
   private final MovingReferenceFrame afterJointFrame;
   private final YoRevoluteJoint jointA, jointB;
   private final YoRevoluteJoint actuatedJoint;
   private final YoRevoluteJoint constrainedJoint;
   private final TwistReadOnly jointTwist;
   private final Twist unitJointTwist = new Twist();
   private final Twist unitSuccessorTwist = new Twist();
   private final Twist unitPredecessorTwist = new Twist();
   private final List<TwistReadOnly> unitTwists;

   private final SpatialAccelerationReadOnly jointAcceleration;
   private final SpatialAcceleration jointBiasAcceleration = new SpatialAcceleration();
   private final SpatialAcceleration successorBiasAcceleration = new SpatialAcceleration();
   private final SpatialAcceleration unitJointAcceleration = new SpatialAcceleration();
   private final SpatialAcceleration unitSuccessorAcceleration = new SpatialAcceleration();
   private final SpatialAcceleration unitPredecessorAcceleration = new SpatialAcceleration();

   private final Wrench unitJointWrench = new Wrench();
   private WrenchReadOnly jointWrench;

   private final int actuatedJointIndex;
   private final DMatrixRMaj constraintJacobian;
   /** Because the constraint ratio is constant, there is no convective term for this joint. */
   private final DMatrixRMaj constraintConvectiveTerm = new DMatrixRMaj(2, 1);
   /** The constraint ratio of the constrained joint with respect to the actuated joint. */
   private final YoDouble constraintRatio;
   /** The constraint offset of the constrained joint with respect to the actuated joint. */
   private final YoDouble constraintOffset;

   /** Variable to store intermediate results for garbage-free operations. */
   private final Vector3D rotationVector = new Vector3D();

   /**
    * The minimum value {@link #getQ()} can have:
    *
    * <pre>
    * this.q &in; [this.jointLimitLower; this.jointLimitUpper]
    * </pre>
    * <p>
    * It is initialized to -&infin;.
    */
   private final YoDouble jointLimitLower;
   /**
    * The maximum value {@link #getQ()} can have:
    *
    * <pre>
    * this.q &in; [this.jointLimitLower; this.jointLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private final YoDouble jointLimitUpper;
   /**
    * The minimum value {@link #getQd()} can have:
    *
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * <p>
    * It is initialized to -&infin;.
    */
   private final YoDouble velocityLimitLower;
   /**
    * The maximum value {@link #getQd()} can have:
    *
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private final YoDouble velocityLimitUpper;
   /**
    * The minimum value {@link #getTau()} can have:
    *
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * <p>
    * It is initialized to -&infin;.
    */
   private final YoDouble effortLimitLower;
   /**
    * The maximum value {@link #getTau()} can have:
    *
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private final YoDouble effortLimitUpper;

   /** These limits result from the limits of the internal joints. */
   private final YoDouble jointInternalLimitLower, jointInternalLimitUpper;
   /** These limits result from the limits of the internal joints. */
   private final YoDouble internalVelocityLimitLower, internalVelocityLimitUpper;

   /**
    * Creates a new revolute twins joint with the following structure:
    * <pre>
    *    root
    *      |
    *      |
    *      O A
    *      |
    *      O B
    *      |
    *      |
    * end-effector
    * </pre>
    * <p>
    * Internally, this joint creates a kinematics that represents the above diagram using revolute joints and rigid-bodies.
    * The kinematics is isolated from the rest of the robot it is attached to. This internal kinematics is only used to facilitate computation of this joint
    * properties.
    * </p>
    * <p>
    * Note that the mass properties of the connecting rigid-body are not used in the rigid-body algorithms such as inverse and forward dynamics calculators.
    * </p>
    *
    * @param name                    the name of this joint.
    * @param predecessor             the rigid-body connected to and preceding this joint.
    * @param jointNameA              the name of the joint A, see diagram above. Can be {@code null}.
    * @param jointNameB              the name of the internal joint B, see diagram above. Can be {@code null}.
    * @param bodyNameAB              the name of the internal rigid-body AB, see diagram above. Can be {@code null}.
    * @param transformAToPredecessor the transform from the frame after the parent joint to the joint A. Not modified.
    * @param transformBToA           the transform from the frame after the joint A to the joint B. Not modified.
    * @param bodyInertiaAB           the inertia of the internal rigid-body AB. Not modified. Can be {@code null}.
    * @param bodyMassAB              the mass of the internal rigid-body AB. Can be {@code 0.0}.
    * @param bodyInertiaPoseAB       the transform of the rigid-body AB's body-fixed frame. Not modified. Can be {@code null}.
    * @param actuatedJointIndex      the index of the joint that is actuated, i.e. torque source. 0 is for joint A, 1 is for joint B.
    * @param constraintRatio         the constraint ratio of the constrained joint with respect to the actuated joint. See {@link #getConstraintRatio()}.
    * @param constraintOffset        the constraint offset of the constrained joint with respect to the actuated joint. See {@link #getConstraintOffset()}.
    * @param jointAxis               the axis of this joint. It is also used to define the axis of the internal revolute joints.
    */
   public YoRevoluteTwinsJoint(String name,
                               RigidBodyBasics predecessor,
                               String jointNameA,
                               String jointNameB,
                               String bodyNameAB,
                               RigidBodyTransformReadOnly transformAToPredecessor,
                               RigidBodyTransformReadOnly transformBToA,
                               Matrix3DReadOnly bodyInertiaAB,
                               double bodyMassAB,
                               RigidBodyTransformReadOnly bodyInertiaPoseAB,
                               int actuatedJointIndex,
                               double constraintRatio,
                               double constraintOffset,
                               Vector3DReadOnly jointAxis,
                               YoRegistry registry)
   {
      this(name,
           predecessor,
           jointNameA,
           jointNameB,
           bodyNameAB,
           transformAToPredecessor,
           transformBToA,
           bodyInertiaAB,
           bodyMassAB,
           bodyInertiaPoseAB,
           MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER,
           actuatedJointIndex,
           constraintRatio,
           constraintOffset,
           jointAxis,
           registry);
   }

   /**
    * Creates a new revolute twins joint with the following structure:
    * <pre>
    *    root
    *      |
    *      |
    *      O A
    *      |
    *      O B
    *      |
    *      |
    * end-effector
    * </pre>
    * <p>
    * Internally, this joint creates a kinematics that represents the above diagram using revolute joints and rigid-bodies.
    * The kinematics is isolated from the rest of the robot it is attached to. This internal kinematics is only used to facilitate computation of this joint
    * properties.
    * </p>
    * <p>
    * Note that the mass properties of the connecting rigid-body are not used in the rigid-body algorithms such as inverse and forward dynamics calculators.
    * </p>
    *
    * @param name                    the name of this joint.
    * @param predecessor             the rigid-body connected to and preceding this joint.
    * @param jointNameA              the name of the joint A, see diagram above. Can be {@code null}.
    * @param jointNameB              the name of the internal joint B, see diagram above. Can be {@code null}.
    * @param bodyNameAB              the name of the internal rigid-body AB, see diagram above. Can be {@code null}.
    * @param transformAToPredecessor the transform from the frame after the parent joint to the joint A. Not modified.
    * @param transformBToA           the transform from the frame after the joint A to the joint B. Not modified.
    * @param bodyInertiaAB           the inertia of the internal rigid-body AB. Not modified. Can be {@code null}.
    * @param bodyMassAB              the mass of the internal rigid-body AB. Can be {@code 0.0}.
    * @param bodyInertiaPoseAB       the transform of the rigid-body AB's body-fixed frame. Not modified. Can be {@code null}.
    * @param rigidBodyBuilder        the factory used to construct the connecting rigid-body AB.
    * @param actuatedJointIndex      the index of the joint that is actuated, i.e. torque source. 0 is for joint A, 1 is for joint B.
    * @param constraintRatio         the constraint ratio of the constrained joint with respect to the actuated joint. See {@link #getConstraintRatio()}.
    * @param constraintOffset        the constraint offset of the constrained joint with respect to the actuated joint. See {@link #getConstraintOffset()}.
    * @param jointAxis               the axis of this joint. It is also used to define the axis of the internal revolute joints.
    */
   public YoRevoluteTwinsJoint(String name,
                               RigidBodyBasics predecessor,
                               String jointNameA,
                               String jointNameB,
                               String bodyNameAB,
                               RigidBodyTransformReadOnly transformAToPredecessor,
                               RigidBodyTransformReadOnly transformBToA,
                               Matrix3DReadOnly bodyInertiaAB,
                               double bodyMassAB,
                               RigidBodyTransformReadOnly bodyInertiaPoseAB,
                               MultiBodySystemFactories.RigidBodyBuilder rigidBodyBuilder,
                               int actuatedJointIndex,
                               double constraintRatio,
                               double constraintOffset,
                               Vector3DReadOnly jointAxis,
                               YoRegistry registry)
   {
      if (actuatedJointIndex < 0 || actuatedJointIndex > 1)
         throw new IllegalArgumentException("The actuated joint index has to be either 0 or 1, was: " + actuatedJointIndex);

      this.actuatedJointIndex = actuatedJointIndex;

      JointReadOnly.checkJointNameSanity(name);

      jointNameA = CrossFourBarJoint.getInternalName(name, jointNameA, "A");
      jointNameB = CrossFourBarJoint.getInternalName(name, jointNameB, "B");

      bodyNameAB = CrossFourBarJoint.getInternalName(name, bodyNameAB, "AB");

      if (bodyInertiaAB == null)
         bodyInertiaAB = new Matrix3D();
      if (bodyInertiaPoseAB == null)
         bodyInertiaPoseAB = new RigidBodyTransform();

      MovingReferenceFrame parentFrame;
      if (predecessor.isRootBody())
         parentFrame = predecessor.getBodyFixedFrame();
      else
         parentFrame = predecessor.getParentJoint().getFrameAfterJoint();

      RigidBody base = new RigidBody(name + "InternalBase", parentFrame);
      jointA = new YoRevoluteJoint(jointNameA, base, transformAToPredecessor, jointAxis, registry);
      RigidBodyBasics bodyAB = rigidBodyBuilder.build(bodyNameAB, jointA, bodyInertiaAB, bodyMassAB, bodyInertiaPoseAB);
      jointB = new YoRevoluteJoint(jointNameB, bodyAB, transformBToA, jointAxis, registry);
      actuatedJoint = actuatedJointIndex == 0 ? jointA : jointB;
      constrainedJoint = actuatedJointIndex == 0 ? jointB : jointA;

      this.name = name;
      this.predecessor = predecessor;
      predecessor.addChildJoint(this);
      nameId = JointReadOnly.computeNameId(this);

      beforeJointFrame = jointA.getFrameBeforeJoint();
      afterJointFrame = jointB.getFrameAfterJoint();

      unitTwists = Collections.singletonList(unitJointTwist);
      jointTwist = MecanoFactories.newTwistReadOnly(this::getQd, unitJointTwist);
      jointAcceleration = MecanoFactories.newSpatialAccelerationVectorReadOnly(this::getQdd, unitJointAcceleration, jointBiasAcceleration);

      this.constraintRatio = new YoDouble("constraintRatio_" + name, registry);
      this.constraintRatio.set(constraintRatio);
      this.constraintOffset = new YoDouble("constraintOffset_" + name, registry);
      this.constraintOffset.set(constraintOffset);
      constraintJacobian = new DMatrixRMaj(2, 1);
      constraintJacobian.set(actuatedJointIndex, 0, 1.0);
      constraintJacobian.set(1 - actuatedJointIndex, 0, constraintRatio);
      // The convection term is zero because the constraint ratio is constant.

      // Creating the YoVariables:
      q = new YoDouble("q_" + name, registry);
      qd = new YoDouble("qd_" + name, registry);
      qdd = new YoDouble("qdd_" + name, registry);
      tau = new YoDouble("tau_" + name, registry);
      jointLimitLower = new YoDouble("q_min_" + name, registry);
      jointLimitUpper = new YoDouble("q_max_" + name, registry);
      velocityLimitLower = new YoDouble("qd_min_" + name, registry);
      velocityLimitUpper = new YoDouble("qd_max_" + name, registry);
      effortLimitLower = new YoDouble("tau_min_" + name, registry);
      effortLimitUpper = new YoDouble("tau_max_" + name, registry);

      jointInternalLimitLower = new YoDouble("q_min_internal_" + name, registry);
      jointInternalLimitUpper = new YoDouble("q_max_internal_" + name, registry);
      internalVelocityLimitLower = new YoDouble("qd_min_internal_" + name, registry);
      internalVelocityLimitUpper = new YoDouble("qd_max_internal_" + name, registry);

      jointLimitLower.set(Double.NEGATIVE_INFINITY);
      jointLimitUpper.set(Double.POSITIVE_INFINITY);
      velocityLimitLower.set(Double.NEGATIVE_INFINITY);
      velocityLimitUpper.set(Double.POSITIVE_INFINITY);
      effortLimitLower.set(Double.NEGATIVE_INFINITY);
      effortLimitUpper.set(Double.POSITIVE_INFINITY);
      updateJointLimits();
      updateVelocityLimits();

      // Setup all the YoVariables listeners:
      q.addListener(yoListener(() -> setQ(q.getValue())));
      qd.addListener(yoListener(() -> setQd(qd.getValue())));
      qdd.addListener(yoListener(() -> setQdd(qdd.getValue())));
      tau.addListener(yoListener(() -> setTau(tau.getValue())));
      actuatedJoint.getYoQ().addListener(yoListener(() -> setQ(actuatedJoint.getQ() * (1.0 + constraintRatio) + constraintOffset)));
      actuatedJoint.getYoQd().addListener(yoListener(() -> setQd(actuatedJoint.getQd() * (1.0 + constraintRatio))));
      actuatedJoint.getYoQdd().addListener(yoListener(() -> setQdd(actuatedJoint.getQdd() * (1.0 + constraintRatio))));
      actuatedJoint.getYoTau().addListener(yoListener(() -> setTau(actuatedJoint.getTau() / (1.0 + constraintRatio))));
      constrainedJoint.getYoQ().addListener(yoListener(() -> setQ((constrainedJoint.getQ() - constraintOffset) / constraintRatio + constrainedJoint.getQ())));
      constrainedJoint.getYoQd().addListener(yoListener(() -> setQd(constrainedJoint.getQd() * (1.0 + 1.0 / constraintRatio))));
      constrainedJoint.getYoQdd().addListener(yoListener(() -> setQdd(constrainedJoint.getQdd() * (1.0 + 1.0 / constraintRatio))));
      constrainedJoint.getYoTau().addListener(yoListener(() -> constrainedJoint.setTau(0.0)));

      actuatedJoint.getYoJointLimitLower().addListener(yoListener(() -> updateJointLimits()));
      actuatedJoint.getYoJointLimitUpper().addListener(yoListener(() -> updateJointLimits()));
      actuatedJoint.getYoVelocityLimitLower().addListener(yoListener(() -> updateVelocityLimits()));
      actuatedJoint.getYoVelocityLimitUpper().addListener(yoListener(() -> updateVelocityLimits()));
      constrainedJoint.getYoJointLimitLower().addListener(yoListener(() -> updateJointLimits()));
      constrainedJoint.getYoJointLimitUpper().addListener(yoListener(() -> updateJointLimits()));
      constrainedJoint.getYoVelocityLimitLower().addListener(yoListener(() -> updateVelocityLimits()));
      constrainedJoint.getYoVelocityLimitUpper().addListener(yoListener(() -> updateVelocityLimits()));
   }

   private void updateJointLimits()
   {
      jointInternalLimitLower.set(RevoluteTwinsJointReadOnly.computeJointLimitLower(this));
      jointInternalLimitUpper.set(RevoluteTwinsJointReadOnly.computeJointLimitUpper(this));
   }

   private void updateVelocityLimits()
   {
      internalVelocityLimitLower.set(RevoluteTwinsJointReadOnly.computeVelocityLimitLower(this));
      internalVelocityLimitUpper.set(RevoluteTwinsJointReadOnly.computeVelocityLimitUpper(this));
   }

   private static YoVariableChangedListener yoListener(Runnable action)
   {
      return new YoVariableChangedListener()
      {
         private boolean isInsideListener = false;

         @Override
         public void changed(YoVariable source)
         {
            if (isInsideListener)
               return;
            isInsideListener = true;
            action.run();
            isInsideListener = false;
         }
      };
   }

   /** {@inheritDoc} */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      jointWrench = MecanoFactories.newWrenchReadOnly(this::getTau, unitJointWrench);
   }

   /** {@inheritDoc} */
   @Override
   public void updateFrame()
   {
      double q_actuated = actuatedJoint.getQ();
      double qDot_actuated = actuatedJoint.getQd();
      double qDDot_actuated = actuatedJoint.getQdd();

      double q_constrained = constraintRatio.getValue() * q_actuated + constraintOffset.getValue();
      double qDot_constrained = constraintRatio.getValue() * qDot_actuated;
      double qDDot_constrained = constraintRatio.getValue() * qDDot_actuated;

      constrainedJoint.setQ(q_constrained);
      constrainedJoint.setQd(qDot_constrained);
      constrainedJoint.setQdd(qDDot_constrained);

      jointA.updateFrame();
      jointB.updateFrame();

      updateMotionSubspace();
   }

   private final Twist deltaTwist = new Twist();
   private final Twist bodyTwist = new Twist();

   /** {@inheritDoc} */
   @Override
   public void updateMotionSubspace()
   {
      RevoluteTwinsJoint.updateUnitJointTwist(this, unitJointTwist);
      // Since we're ignoring the bias terms, the unit-accelerations are the same as the unit-twists.
      unitJointAcceleration.setIncludingFrame(unitJointTwist);

      RevoluteTwinsJoint.updateBiasAcceleration(this, deltaTwist, bodyTwist, jointBiasAcceleration);

      if (getSuccessor() != null)
      {
         unitSuccessorTwist.setIncludingFrame(unitJointTwist);
         unitSuccessorTwist.setBaseFrame(predecessor.getBodyFixedFrame());
         unitSuccessorTwist.setBodyFrame(successor.getBodyFixedFrame());
         unitSuccessorTwist.changeFrame(successor.getBodyFixedFrame());

         unitPredecessorTwist.setIncludingFrame(unitSuccessorTwist);
         unitPredecessorTwist.invert();
         unitPredecessorTwist.changeFrame(predecessor.getBodyFixedFrame());

         // Since we're ignoring the bias terms, the unit-accelerations are the same as the unit-twists.
         unitSuccessorAcceleration.setIncludingFrame(unitSuccessorTwist);
         unitPredecessorAcceleration.setIncludingFrame(unitPredecessorTwist);

         successorBiasAcceleration.setIncludingFrame(jointBiasAcceleration);
         successorBiasAcceleration.setBaseFrame(getPredecessor().getBodyFixedFrame());
         successorBiasAcceleration.setBodyFrame(getSuccessor().getBodyFixedFrame());
         successorBiasAcceleration.changeFrame(getSuccessor().getBodyFixedFrame());

         unitJointWrench.setIncludingFrame(actuatedJoint.getUnitJointTwist());
         unitJointWrench.changeFrame(afterJointFrame);
         unitJointWrench.setBodyFrame(getSuccessor().getBodyFixedFrame());
      }
   }

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getActuatedJoint()
   {
      return actuatedJoint;
   }

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getConstrainedJoint()
   {
      return constrainedJoint;
   }

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getJointA()
   {
      return jointA;
   }

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getJointB()
   {
      return jointB;
   }

   /** {@inheritDoc} */
   @Override
   public int getActuatedJointIndex()
   {
      return actuatedJointIndex;
   }

   /** {@inheritDoc} */
   @Override
   public double getConstraintRatio()
   {
      return constraintRatio.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getConstraintOffset()
   {
      return constraintOffset.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public DMatrixRMaj getConstraintJacobian()
   {
      return constraintJacobian;
   }

   /** {@inheritDoc} */
   @Override
   public DMatrixRMaj getConstraintConvectiveTerm()
   {
      return constraintConvectiveTerm;
   }

   /** {@inheritDoc} */
   @Override
   public MovingReferenceFrame getFrameBeforeJoint()
   {
      return beforeJointFrame;
   }

   /** {@inheritDoc} */
   @Override
   public MovingReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getPredecessor()
   {
      return predecessor;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getSuccessor()
   {
      return successor;
   }

   /** {@inheritDoc} */
   @Override
   public MovingReferenceFrame getLoopClosureFrame()
   {
      return null;
   }

   /** {@inheritDoc} */
   @Override
   public String getName()
   {
      return name;
   }

   /** {@inheritDoc} */
   @Override
   public String getNameId()
   {
      return nameId;
   }

   /** This feature is not supported. */
   @Override
   public void setupLoopClosure(RigidBodyBasics successor, RigidBodyTransformReadOnly transformFromSuccessorParentJoint)
   {
      throw new UnsupportedOperationException("Loop closure using a four bar joint has not been implemented.");
   }

   /** {@inheritDoc} */
   @Override
   public double getQ()
   {
      q.set(RevoluteTwinsJointBasics.super.getQ(), false);
      return q.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getQd()
   {
      qd.set(RevoluteTwinsJointBasics.super.getQd(), false);
      return qd.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getQdd()
   {
      qdd.set(RevoluteTwinsJointBasics.super.getQdd(), false);
      return qdd.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getTau()
   {
      tau.set(actuatedJoint.getTau() / (1.0 + constraintRatio.getValue()), false);
      return tau.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public TwistReadOnly getUnitJointTwist()
   {
      return unitJointTwist;
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
   public void getJointConfiguration(RigidBodyTransform jointConfigurationToPack)
   {
      afterJointFrame.getTransformToDesiredFrame(jointConfigurationToPack, beforeJointFrame);
   }

   /** {@inheritDoc} */
   @Override
   public TwistReadOnly getJointTwist()
   {
      return jointTwist;
   }

   /** {@inheritDoc} */
   @Override
   public List<TwistReadOnly> getUnitTwists()
   {
      return unitTwists;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getJointAcceleration()
   {
      return jointAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getJointBiasAcceleration()
   {
      return jointBiasAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public SpatialAccelerationReadOnly getSuccessorBiasAcceleration()
   {
      return successorBiasAcceleration;
   }

   /** This feature is not implemented. */
   @Override
   public void getPredecessorAcceleration(SpatialAccelerationBasics accelerationToPack)
   {
      // OneDoFJointReadOnly.getPredecessorAcceleration(...) was not used when creating this joint.
      // Implementing it would require extra calculation in the updateMotionSubspace().
      throw new UnsupportedOperationException("Implement me!");
   }

   /** This feature is not implemented. */
   @Override
   public SpatialAccelerationReadOnly getPredecessorBiasAcceleration()
   {
      // OneDoFJointReadOnly.getPredecessorBiasAcceleration() was not used when creating this joint.
      // Implementing it would require extra calculation in the updateMotionSubspace().
      throw new UnsupportedOperationException("Implement me!");
   }

   /** {@inheritDoc} */
   @Override
   public WrenchReadOnly getJointWrench()
   {
      return jointWrench;
   }

   /** {@inheritDoc} */
   @Override
   public void setConstraintRatio(double constraintRatio)
   {
      this.constraintRatio.set(constraintRatio);
   }

   /** {@inheritDoc} */
   @Override
   public void setConstraintOffset(double constraintOffset)
   {
      this.constraintOffset.set(constraintOffset);
   }

   /** {@inheritDoc} */
   @Override
   public void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
      jointOrientation.getRotationVector(rotationVector);
      setQ(rotationVector.dot(getJointAxis()));
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
   public void setQ(double q)
   {
      RevoluteTwinsJointBasics.super.setQ(q);
      this.q.set(q, false); // Just to make sure the YoVariable is up-to-date.
   }

   /** {@inheritDoc} */
   @Override
   public void setQd(double qd)
   {
      RevoluteTwinsJointBasics.super.setQd(qd);
      this.qd.set(qd, false); // Just to make sure the YoVariable is up-to-date.
   }

   /** {@inheritDoc} */
   @Override
   public void setQdd(double qdd)
   {
      RevoluteTwinsJointBasics.super.setQdd(qdd);
      this.qdd.set(qdd, false); // Just to make sure the YoVariable is up-to-date.
   }

   /** {@inheritDoc} */
   @Override
   public void setTau(double tau)
   {
      RevoluteTwinsJointBasics.super.setTau(tau);
      this.tau.set(tau, false); // Just to make sure the YoVariable is up-to-date.
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
   public double getJointLimitLower()
   {
      if (jointInternalLimitLower.getValue() > jointInternalLimitUpper.getValue())
      {
         throw new IllegalStateException("The joint limits are inconsistent: [" + jointInternalLimitLower.getValue() + ", " + jointInternalLimitUpper.getValue()
                                         + "]. This probably means that limits for the joints A and B are incompatible given the constraint ratio.");
      }
      return Math.max(jointLimitLower.getValue(), jointInternalLimitLower.getValue());
   }

   /** {@inheritDoc} */
   @Override
   public double getJointLimitUpper()
   {
      if (jointInternalLimitLower.getValue() > jointInternalLimitUpper.getValue())
      {
         throw new IllegalStateException("The joint limits are inconsistent: [" + jointInternalLimitLower.getValue() + ", " + jointInternalLimitUpper.getValue()
                                         + "]. This probably means that limits for the joints A and B are incompatible given the constraint ratio.");
      }
      return Math.min(jointLimitUpper.getValue(), jointInternalLimitUpper.getValue());
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitLower()
   {
      if (internalVelocityLimitLower.getValue() > internalVelocityLimitUpper.getValue())
      {
         throw new IllegalStateException(
               "The velocity limits are inconsistent: [" + internalVelocityLimitLower.getValue() + ", " + internalVelocityLimitUpper.getValue()
               + "]. This probably means that limits for the joints A and B are incompatible given the constraint ratio.");
      }
      return Math.max(velocityLimitLower.getValue(), internalVelocityLimitLower.getValue());
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityLimitUpper()
   {
      if (internalVelocityLimitLower.getValue() > internalVelocityLimitUpper.getValue())
      {
         throw new IllegalStateException(
               "The velocity limits are inconsistent: [" + internalVelocityLimitLower.getValue() + ", " + internalVelocityLimitUpper.getValue()
               + "]. This probably means that limits for the joints A and B are incompatible given the constraint ratio.");
      }
      return Math.min(velocityLimitUpper.getValue(), internalVelocityLimitUpper.getValue());
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitLower()
   {
      return Math.max(RevoluteTwinsJointBasics.super.getEffortLimitLower(), effortLimitLower.getValue());
   }

   /** {@inheritDoc} */
   @Override
   public double getEffortLimitUpper()
   {
      return Math.min(RevoluteTwinsJointBasics.super.getEffortLimitUpper(), effortLimitUpper.getValue());
   }

   /**
    * Gets the yoVariable used to store this joint constraint ratio.
    *
    * @return the constraint ratio's yoVariable.
    */
   public YoDouble getYoConstraintRatio()
   {
      return constraintRatio;
   }

   /**
    * Gets the yoVariable used to store this joint constraint offset.
    *
    * @return the constraint offset's yoVariable.
    */
   public YoDouble getYoConstraintOffset()
   {
      return constraintOffset;
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
    * @return the torque's yoVariable.
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
    * @return the torque lower limit's yoVariable.
    */
   @Override
   public YoDouble getYoEffortLimitLower()
   {
      return effortLimitLower;
   }

   /**
    * Gets the yoVariable used to store this joint's force/torque upper limit.
    *
    * @return the torque upper limit's yoVariable.
    */
   @Override
   public YoDouble getYoEffortLimitUpper()
   {
      return effortLimitUpper;
   }

   /**
    * Returns the implementation name of this joint and the joint name.
    */
   @Override
   public String toString()
   {
      String qAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQ());
      String qdAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQd());
      String qddAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getQdd());
      String tauAsString = String.format(EuclidCoreIOTools.DEFAULT_FORMAT, getTau());
      return getClass().getSimpleName() + " " + getName() + ", q: " + qAsString + ", qd: " + qdAsString + ", qdd: " + qddAsString + ", tau: " + tauAsString;
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
