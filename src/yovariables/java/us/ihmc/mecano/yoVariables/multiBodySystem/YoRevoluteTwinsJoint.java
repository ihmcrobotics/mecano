package us.ihmc.mecano.yoVariables.multiBodySystem;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteTwinsJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.interfaces.YoOneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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
   private final RevoluteJointBasics jointA, jointB;
   private final RevoluteJointBasics actuatedJoint;
   private final RevoluteJointBasics constrainedJoint;
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
   /**
    * Because the constraint ratio is constant, there is no convective term for this joint.
    */
   private final DMatrixRMaj constraintConvectiveTerm = new DMatrixRMaj(2, 1);
   private final double constraintRatio, constraintOffset;

   /**
    * Variable to store intermediate results for garbage-free operations.
    */
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
   private YoDouble jointLimitLower;
   /**
    * The maximum value {@link #getQ()} can have:
    *
    * <pre>
    * this.q &in; [this.jointLimitLower; this.jointLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private YoDouble jointLimitUpper;
   /**
    * The minimum value {@link #getQd()} can have:
    *
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * <p>
    * It is initialized to -&infin;.
    */
   private YoDouble velocityLimitLower;
   /**
    * The maximum value {@link #getQd()} can have:
    *
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private YoDouble velocityLimitUpper;
   /**
    * The minimum value {@link #getTau()} can have:
    *
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * <p>
    * It is initialized to -&infin;.
    */
   private YoDouble effortLimitLower;
   /**
    * The maximum value {@link #getTau()} can have:
    *
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private YoDouble effortLimitUpper;

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
   public RevoluteTwinsJoint(String name,
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
   public RevoluteTwinsJoint(String name,
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
      jointA = new RevoluteJoint(jointNameA, base, transformAToPredecessor, jointAxis); // TODO Override the setQ, setQd, setQdd, and setTau so they update the YoVariables here
      RigidBodyBasics bodyAB = rigidBodyBuilder.build(bodyNameAB, jointA, bodyInertiaAB, bodyMassAB, bodyInertiaPoseAB);
      jointB = new RevoluteJoint(jointNameB, bodyAB, transformBToA, jointAxis); // TODO Override the setQ, setQd, setQdd, and setTau so they update the YoVariables here
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

      this.constraintRatio = constraintRatio;
      this.constraintOffset = constraintOffset;
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
   }
}
