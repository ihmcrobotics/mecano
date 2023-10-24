package us.ihmc.mecano.multiBodySystem;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
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

   private final double actuatedJointIndex;
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

      FrameVector3D jointBAxis = new FrameVector3D(jointB.getJointAxis());
      jointBAxis.changeFrame(jointA.getJointAxis().getReferenceFrame());

      if (!EuclidFrameTools.areVector3DsParallel(jointA.getJointAxis(), jointBAxis, 1.0e-7))
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
   }

   private static String getInternalName(String jointName, String internalName, String defaultNameSuffix)
   {
      if (internalName == null)
         return jointName + "_" + defaultNameSuffix;

      JointReadOnly.checkJointNameSanity(internalName);
      return internalName;
   }
}
