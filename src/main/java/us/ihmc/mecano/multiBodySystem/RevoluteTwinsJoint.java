package us.ihmc.mecano.multiBodySystem;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.*;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.Collections;
import java.util.List;

/**
 * Implementation of a revolute twins joint.
 * <p>
 * A revolute twins joint is a joint that has 1 degree of freedom of rotation and which has a variable center of rotation which changes with its configuration.
 * The joint is composed of two revolute joints that share the same axis of rotation, their angle is constrained such that overall the revolute twins has a
 * single degree of freedom. Example of a revolute twins joint:
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
 * where A and B are both revolute joints around the same axis. It is assumed that only one joint in this sub-system composed of {A, B} is a torque source, this
 * is the actuated joint.
 * </p>
 */
public class RevoluteTwinsJoint implements RevoluteTwinsJointBasics
{
   private final String name;
   private final String nameId;
   private final RigidBodyBasics predecessor;
   private RigidBodyBasics successor;
   private final MovingReferenceFrame beforeJointFrame;
   private final MovingReferenceFrame afterJointFrame;
   private final RevoluteJointBasics jointA, jointB;
   /**
    * The actuated joint is the torque source and is either {@link #jointA} or {@link #jointB}.
    */
   private final RevoluteJointBasics actuatedJoint;
   /**
    * The constrained joint is the joint that is not the torque source, its state is constrained by the state of the {@link #actuatedJoint}, and it is either
    * {@link #jointA} or {@link #jointB}.
    */
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
   private double jointLimitLower = Double.NEGATIVE_INFINITY;
   /**
    * The maximum value {@link #getQ()} can have:
    *
    * <pre>
    * this.q &in; [this.jointLimitLower; this.jointLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private double jointLimitUpper = Double.POSITIVE_INFINITY;
   /**
    * The minimum value {@link #getQd()} can have:
    *
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * <p>
    * It is initialized to -&infin;.
    */
   private double velocityLimitLower = Double.NEGATIVE_INFINITY;
   /**
    * The maximum value {@link #getQd()} can have:
    *
    * <pre>
    * this.qd &in; [this.velocityLimitLower; this.velocityLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private double velocityLimitUpper = Double.POSITIVE_INFINITY;
   /**
    * The minimum value {@link #getTau()} can have:
    *
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * <p>
    * It is initialized to -&infin;.
    */
   private double effortLimitLower = Double.NEGATIVE_INFINITY;
   /**
    * The maximum value {@link #getTau()} can have:
    *
    * <pre>
    * this.tau &in; [this.effortLimitLower; this.effortLimitUpper]
    * </pre>
    * <p>
    * It is initialized to +&infin;.
    */
   private double effortLimitUpper = Double.POSITIVE_INFINITY;

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
                             Vector3DReadOnly jointAxis)
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
           jointAxis);
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
                             Vector3DReadOnly jointAxis)
   {
      if (actuatedJointIndex < 0 || actuatedJointIndex > 1)
         throw new IllegalArgumentException("The actuated joint index has to be either 0 or 1, was: " + actuatedJointIndex);

      this.actuatedJointIndex = actuatedJointIndex;

      JointReadOnly.checkJointNameSanity(name);

      jointNameA = getInternalName(name, jointNameA, "A");
      jointNameB = getInternalName(name, jointNameB, "B");

      bodyNameAB = getInternalName(name, bodyNameAB, "AB");

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
      jointA = new RevoluteJoint(jointNameA, base, transformAToPredecessor, jointAxis);
      RigidBodyBasics bodyAB = rigidBodyBuilder.build(bodyNameAB, jointA, bodyInertiaAB, bodyMassAB, bodyInertiaPoseAB);
      jointB = new RevoluteJoint(jointNameB, bodyAB, transformBToA, jointAxis);
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
   }

   private static String getInternalName(String jointName, String internalName, String defaultNameSuffix)
   {
      if (internalName == null)
         return jointName + "_" + defaultNameSuffix;

      JointReadOnly.checkJointNameSanity(internalName);
      return internalName;
   }

   public RevoluteTwinsJoint(String name, RevoluteJointBasics[] revoluteTwinsJoints, int actuatedJointIndex, double constraintRatio, double constraintOffset)
   {
      if (revoluteTwinsJoints.length != 2)
         throw new IllegalArgumentException("There must be exactly two revolute joints, was: " + revoluteTwinsJoints.length);

      if (actuatedJointIndex < 0 || actuatedJointIndex > 1)
         throw new IllegalArgumentException("The actuated joint index has to be either 0 or 1, was: " + actuatedJointIndex);

      if (MultiBodySystemTools.computeDistanceToRoot(revoluteTwinsJoints[0].getPredecessor())
          < MultiBodySystemTools.computeDistanceToRoot(revoluteTwinsJoints[1].getPredecessor()))
      {
         this.actuatedJointIndex = actuatedJointIndex;
         jointA = revoluteTwinsJoints[0];
         jointB = revoluteTwinsJoints[1];
      }
      else
      {
         this.actuatedJointIndex = 1 - actuatedJointIndex;
         jointA = revoluteTwinsJoints[1];
         jointB = revoluteTwinsJoints[0];
      }
      actuatedJoint = actuatedJointIndex == 0 ? jointA : jointB;
      constrainedJoint = actuatedJointIndex == 0 ? jointB : jointA;

      FrameVector3D jointBAxis = new FrameVector3D(jointB.getJointAxis());
      jointBAxis.changeFrame(jointA.getJointAxis().getReferenceFrame());

      if (jointA.getJointAxis().dot(jointBAxis) < 1.0 - 1.0e-7) // For now, we do not allow for the axes to be flipped.
         throw new IllegalArgumentException(String.format("The axes of the joint %s and %s are not parallel.", jointA.getName(), jointB.getName()));

      this.name = name;
      predecessor = jointA.getPredecessor();
      predecessor.getChildrenJoints().remove(jointA);
      predecessor.addChildJoint(this);

      beforeJointFrame = jointA.getFrameBeforeJoint();
      afterJointFrame = jointB.getFrameAfterJoint();

      nameId = JointReadOnly.computeNameId(this);

      unitTwists = Collections.singletonList(unitJointTwist);
      jointTwist = MecanoFactories.newTwistReadOnly(this::getQd, unitJointTwist);
      jointAcceleration = MecanoFactories.newSpatialAccelerationVectorReadOnly(this::getQdd, unitJointAcceleration, jointBiasAcceleration);

      this.constraintRatio = constraintRatio;
      this.constraintOffset = constraintOffset;
      constraintJacobian = new DMatrixRMaj(2, 1);
      constraintJacobian.set(actuatedJointIndex, 0, 1.0);
      constraintJacobian.set(1 - actuatedJointIndex, 0, constraintRatio);
      // The convection term is zero because the constraint ratio is constant.
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      jointWrench = MecanoFactories.newWrenchReadOnly(this::getTau, unitJointWrench);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void updateFrame()
   {
      double q_actuated = actuatedJoint.getQ();
      double qDot_actuated = actuatedJoint.getQd();
      double qDDot_actuated = actuatedJoint.getQdd();

      double q_constrained = constraintRatio * q_actuated + constraintOffset;
      double qDot_constrained = constraintRatio * qDot_actuated;
      double qDDot_constrained = constraintRatio * qDDot_actuated;

      constrainedJoint.setQ(q_constrained);
      constrainedJoint.setQd(qDot_constrained);
      constrainedJoint.setQdd(qDDot_constrained);

      jointA.updateFrame();
      jointB.updateFrame();

      updateMotionSubspace();
   }

   private final Twist deltaTwist = new Twist();
   private final Twist bodyTwist = new Twist();

   /**
    * {@inheritDoc}
    */
   @Override
   public void updateMotionSubspace()
   {
      updateUnitJointTwist(this, unitJointTwist);
      // Since we're ignoring the bias terms, the unit-accelerations are the same as the unit-twists.
      unitJointAcceleration.setIncludingFrame(unitJointTwist);

      updateBiasAcceleration(this, deltaTwist, bodyTwist, jointBiasAcceleration);

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

   /**
    * Computes the unit-twist for the given revolute twins joint and stores the result in the given
    * twist.
    * <p>
    * This method relies on {@link RevoluteTwinsJointReadOnly#getConstraintJacobian()} to be up-to-date.
    * </p>
    *
    * @param joint                the joint to update the unit-twist of. Not modified.
    * @param unitJointTwistToPack the twist in which the result is stored. Modified.
    */
   private static void updateUnitJointTwist(RevoluteTwinsJointReadOnly joint, TwistBasics unitJointTwistToPack)
   {
      RevoluteJointReadOnly jointA = joint.getJointA();
      TwistReadOnly unitTwistA = jointA.getUnitJointTwist();
      RevoluteJointReadOnly jointB = joint.getJointB();
      TwistReadOnly unitTwistB = jointB.getUnitJointTwist();
      double jA = joint.getConstraintJacobian().get(0, 0);
      double jB = joint.getConstraintJacobian().get(1, 0);

      unitJointTwistToPack.setIncludingFrame(unitTwistA);
      unitJointTwistToPack.scale(jA);
      unitJointTwistToPack.setBodyFrame(jointB.getFrameBeforeJoint());
      unitJointTwistToPack.changeFrame(jointB.getFrameAfterJoint());
      unitJointTwistToPack.getAngularPart().scaleAdd(jB, unitTwistB.getAngularPart(), unitJointTwistToPack.getAngularPart());
      unitJointTwistToPack.getLinearPart().scaleAdd(jB, unitTwistB.getLinearPart(), unitJointTwistToPack.getLinearPart());
      unitJointTwistToPack.scale(1.0 / (jA + jB));
      unitJointTwistToPack.setBodyFrame(joint.getFrameAfterJoint());
   }

   /**
    * Computes the bias acceleration for the given revolute twins joint and stores the result in the
    * given spatial acceleration.
    * <p>
    * This method relies on {@link RevoluteTwinsJointReadOnly#getConstraintConvectiveTerm()} and
    * {@link RevoluteTwinsJointReadOnly#getUnitJointAcceleration()} to be up-to-date.
    *
    * @param joint                 the joint to compute the bias acceleration of. Not Modified.
    * @param deltaTwist            twist used to stores intermediate result. Modified.
    * @param bodyTwist             twist used to stores intermediate result. Modified.
    * @param jointBiasAcceleration the spatial acceleration in which the result is stored. Modified.
    */
   public static void updateBiasAcceleration(RevoluteTwinsJointReadOnly joint,
                                             TwistBasics deltaTwist,
                                             TwistBasics bodyTwist,
                                             SpatialAccelerationBasics jointBiasAcceleration)
   {
      RevoluteJointReadOnly jointA = joint.getJointA();
      RevoluteJointReadOnly jointB = joint.getJointB();

      /*
       * This next block is for computing the bias acceleration. I ended up using tests to figure out
       * exactly what it should, but I feel that it can be simplified.
       */
      jointB.getFrameAfterJoint().getTwistRelativeToOther(jointA.getFrameAfterJoint(), deltaTwist);
      jointB.getFrameBeforeJoint().getTwistRelativeToOther(jointA.getFrameBeforeJoint(), bodyTwist);
      deltaTwist.changeFrame(jointB.getFrameAfterJoint());
      bodyTwist.changeFrame(jointB.getFrameAfterJoint());
      jointBiasAcceleration.setToZero(jointB.getFrameBeforeJoint(), joint.getFrameBeforeJoint(), jointA.getFrameAfterJoint());
      jointBiasAcceleration.changeFrame(jointB.getFrameAfterJoint(), deltaTwist, bodyTwist);
      jointBiasAcceleration.setBodyFrame(joint.getFrameAfterJoint());
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RevoluteJointBasics getActuatedJoint()
   {
      return actuatedJoint;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RevoluteJointBasics getJointA()
   {
      return jointA;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RevoluteJointBasics getJointB()
   {
      return jointB;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public int getActuatedJointIndex()
   {
      return actuatedJointIndex;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getConstraintRatio()
   {
      return constraintRatio;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getConstraintOffset()
   {
      return constraintOffset;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public DMatrixRMaj getConstraintJacobian()
   {
      return constraintJacobian;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public DMatrixRMaj getConstraintConvectiveTerm()
   {
      return constraintConvectiveTerm;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public MovingReferenceFrame getFrameBeforeJoint()
   {
      return beforeJointFrame;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public MovingReferenceFrame getFrameAfterJoint()
   {
      return afterJointFrame;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RigidBodyBasics getPredecessor()
   {
      return predecessor;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RigidBodyBasics getSuccessor()
   {
      return successor;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public MovingReferenceFrame getLoopClosureFrame()
   {
      return null;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public String getName()
   {
      return name;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public String getNameId()
   {
      return nameId;
   }

   /**
    * This feature is not supported.
    */
   @Override
   public void setupLoopClosure(RigidBodyBasics successor, RigidBodyTransformReadOnly transformFromSuccessorParentJoint)
   {
      throw new UnsupportedOperationException("Loop closure using a four bar joint has not been implemented.");
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getTau()
   {
      // First we update the actuated joint tau
      double tau_actuated = actuatedJoint.getTau() + constrainedJoint.getTau() * constraintRatio;
      constrainedJoint.setTau(0.0);
      actuatedJoint.setTau(tau_actuated);
      // equivalent to: actuatedJoint.getTau() / (constraintJacobian.get(0, 0) + constraintJacobian.get(1, 0));
      return actuatedJoint.getTau() / (1.0 + constraintRatio);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public TwistReadOnly getUnitJointTwist()
   {
      return unitJointTwist;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public TwistReadOnly getUnitSuccessorTwist()
   {
      return unitSuccessorTwist;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public TwistReadOnly getUnitPredecessorTwist()
   {
      return unitPredecessorTwist;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public SpatialAccelerationReadOnly getUnitJointAcceleration()
   {
      return unitJointAcceleration;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public SpatialAccelerationReadOnly getUnitSuccessorAcceleration()
   {
      return unitSuccessorAcceleration;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public SpatialAccelerationReadOnly getUnitPredecessorAcceleration()
   {
      return unitPredecessorAcceleration;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void getJointConfiguration(RigidBodyTransform jointConfigurationToPack)
   {
      afterJointFrame.getTransformToDesiredFrame(jointConfigurationToPack, beforeJointFrame);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public TwistReadOnly getJointTwist()
   {
      return jointTwist;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public List<TwistReadOnly> getUnitTwists()
   {
      return unitTwists;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public SpatialAccelerationReadOnly getJointAcceleration()
   {
      return jointAcceleration;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public SpatialAccelerationReadOnly getJointBiasAcceleration()
   {
      return jointBiasAcceleration;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public SpatialAccelerationReadOnly getSuccessorBiasAcceleration()
   {
      return successorBiasAcceleration;
   }

   /**
    * This feature is not implemented.
    */
   @Override
   public void getPredecessorAcceleration(SpatialAccelerationBasics accelerationToPack)
   {
      // OneDoFJointReadOnly.getPredecessorAcceleration(...) was not used when creating this joint.
      // Implementing it would require extra calculation in the updateMotionSubspace().
      throw new UnsupportedOperationException("Implement me!");
   }

   /**
    * This feature is not implemented.
    */
   @Override
   public SpatialAccelerationReadOnly getPredecessorBiasAcceleration()
   {
      // OneDoFJointReadOnly.getPredecessorBiasAcceleration() was not used when creating this joint.
      // Implementing it would require extra calculation in the updateMotionSubspace().
      throw new UnsupportedOperationException("Implement me!");
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public WrenchReadOnly getJointWrench()
   {
      return jointWrench;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
      jointOrientation.getRotationVector(rotationVector);
      setQ(rotationVector.dot(getJointAxis()));
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double computeActuatedJointQ(double q)
   {
      //             q = q_actuated + q_constrained
      // q_constrained = constraintRatio * q_actuated + constraintOffset
      //             q = q_actuated + constraintRatio * q_actuated + constraintOffset
      //             q = (1 + constraintRatio) * q_actuated + constraintOffset
      //    q_actuated = (q - constraintOffset) / (1 + constraintRatio)
      return (q - constraintOffset) / (1.0 + constraintRatio);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double computeActuatedJointQd(double qd)
   {
      //             qd = qd_actuated + qd_constrained
      // qd_constrained = constraintRatio * qd_actuated
      //             qd = qd_actuated + constraintRatio * qd_actuated
      //             qd = (1 + constraintRatio) * qd_actuated
      //    qd_actuated = qd / (1 + constraintRatio)
      return qd / (1.0 + constraintRatio);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double computeActuatedJointQdd(double qdd)
   {
      //             qdd = qdd_actuated + qdd_constrained
      // qdd_constrained = constraintRatio * qdd_actuated
      //             qdd = qdd_actuated + constraintRatio * qdd_actuated
      //             qdd = (1 + constraintRatio) * qdd_actuated
      //    qdd_actuated = qdd / (1 + constraintRatio)
      return qdd / (1.0 + constraintRatio);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double computeActuatedJointTau(double tau)
   {
      return tau * (1.0 + constraintRatio);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setJointLimitLower(double jointLimitLower)
   {
      this.jointLimitLower = jointLimitLower;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setJointLimitUpper(double jointLimitUpper)
   {
      this.jointLimitUpper = jointLimitUpper;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setVelocityLimitLower(double velocityLimitLower)
   {
      this.velocityLimitLower = velocityLimitLower;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setVelocityLimitUpper(double velocityLimitUpper)
   {
      this.velocityLimitUpper = velocityLimitUpper;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setEffortLimitLower(double effortLimitLower)
   {
      this.effortLimitLower = effortLimitLower;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setEffortLimitUpper(double effortLimitUpper)
   {
      this.effortLimitUpper = effortLimitUpper;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getJointLimitLower()
   {
      return Math.max(RevoluteTwinsJointBasics.super.getJointLimitLower(), jointLimitLower);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getJointLimitUpper()
   {
      return Math.min(RevoluteTwinsJointBasics.super.getJointLimitUpper(), jointLimitUpper);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getVelocityLimitLower()
   {
      return Math.max(RevoluteTwinsJointBasics.super.getVelocityLimitLower(), velocityLimitLower);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getVelocityLimitUpper()
   {
      return Math.min(RevoluteTwinsJointBasics.super.getVelocityLimitUpper(), velocityLimitUpper);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getEffortLimitLower()
   {
      return Math.max(RevoluteTwinsJointBasics.super.getEffortLimitLower(), effortLimitLower);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getEffortLimitUpper()
   {
      return Math.min(RevoluteTwinsJointBasics.super.getEffortLimitUpper(), effortLimitUpper);
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

   /**
    * Clones the given joint and attached the clone to a stationary frame.
    *
    * @param original        the original cross four bar joint to be cloned.
    * @param stationaryFrame the frame to which the clone should be attached to.
    * @param cloneSuffix     the suffix for name of the clone, i.e. the name of the clone is
    *                        {@code original.getName() + cloneSuffix}.
    * @return the clone cross four bar joint.
    */
   public static RevoluteTwinsJoint cloneCrossFourBarJoint(RevoluteTwinsJointReadOnly original, ReferenceFrame stationaryFrame, String cloneSuffix)
   {
      RigidBodyReadOnly originalPredecessor = original.getPredecessor();
      RigidBodyBasics clonePredecessor = new RigidBody(originalPredecessor.getName() + cloneSuffix, stationaryFrame);
      return cloneCrossFourBarJoint(original, clonePredecessor, cloneSuffix);
   }

   /**
    * Clones the given cross four bar joint {@code original} and attach the clone to
    * {@code clonePredecessor}.
    *
    * @param original         the original cross four bar joint to be cloned.
    * @param cloneSuffix      the suffix for name of the clone, i.e. the name of the clone is
    *                         {@code original.getName() + cloneSuffix}.
    * @param clonePredecessor the predecessor of the clone.
    * @return the clone cross four bar joint.
    */
   public static RevoluteTwinsJoint cloneCrossFourBarJoint(RevoluteTwinsJointReadOnly original, RigidBodyBasics clonePredecessor, String cloneSuffix)
   {
      return (RevoluteTwinsJoint) MultiBodySystemFactories.DEFAULT_JOINT_BUILDER.cloneRevoluteTwinsJoint(original, cloneSuffix, clonePredecessor);
   }
}
