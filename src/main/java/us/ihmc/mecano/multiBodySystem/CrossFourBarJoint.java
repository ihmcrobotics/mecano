package us.ihmc.mecano.multiBodySystem;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.fourBar.CrossFourBarJointIKBinarySolver;
import us.ihmc.mecano.fourBar.CrossFourBarJointIKSolver;
import us.ihmc.mecano.fourBar.FourBarKinematicLoopFunction;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;

/**
 * Implementation of a cross four bar joint.
 * <p>
 * A cross four bar joint is a joint that has 1 degree of freedom of rotation and which has a
 * variable center of rotation which changes with its configuration. Example of a cross four bar
 * joint:
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
 * where A, B, C, and D are all revolute joint around the same axis. It is assumed that only one
 * joint in this sub-system composed of {A, B, C, and D} is a torque source, this is the actuated
 * joint.
 * </p>
 */
public class CrossFourBarJoint implements CrossFourBarJointBasics
{
   private final String name;
   private final String nameId;
   private final RigidBodyBasics predecessor;
   private RigidBodyBasics successor;
   private final MovingReferenceFrame beforeJointFrame;
   private final MovingReferenceFrame afterJointFrame;

   private final FourBarKinematicLoopFunction fourBarFunction;
   private CrossFourBarJointIKSolver ikSolver;

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
    * </p>
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
    * </p>
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
    * </p>
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
    * </p>
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
    * </p>
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
    * </p>
    */
   private double effortLimitUpper = Double.POSITIVE_INFINITY;

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
    * joint is connected to. This internal kinematics is only used to facilitate computation of this
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
    */
   public CrossFourBarJoint(String name,
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
                            Vector3DReadOnly jointAxis)
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
           jointAxis);
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
    */
   public CrossFourBarJoint(String name,
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
                            Vector3DReadOnly jointAxis)
   {
      if (actuatedJointIndex == loopClosureJointIndex)
         throw new IllegalArgumentException("The actuated joint cannot be the loop closure.");
      if (loopClosureJointIndex < 2)
         throw new UnsupportedOperationException("Only the joint C and D support the loop closure.");

      JointReadOnly.checkJointNameSanity(name);

      jointNameA = getInternalName(name, jointNameA, "A");
      jointNameB = getInternalName(name, jointNameB, "B");
      jointNameC = getInternalName(name, jointNameC, "C");
      jointNameD = getInternalName(name, jointNameD, "D");

      bodyNameBC = getInternalName(name, bodyNameBC, "BC");
      bodyNameDA = getInternalName(name, bodyNameDA, "DA");

      if (bodyInertiaBC == null)
         bodyInertiaBC = new Matrix3D();
      if (bodyInertiaDA == null)
         bodyInertiaDA = new Matrix3D();
      if (bodyInertiaPoseBC == null)
         bodyInertiaPoseBC = new RigidBodyTransform();
      if (bodyInertiaPoseDA == null)
         bodyInertiaPoseDA = new RigidBodyTransform();

      MovingReferenceFrame parentFrame;
      if (predecessor.isRootBody())
         parentFrame = predecessor.getBodyFixedFrame();
      else
         parentFrame = predecessor.getParentJoint().getFrameAfterJoint();

      RigidBody base = new RigidBody(name + "InternalBase", parentFrame);
      RevoluteJoint jointA = new RevoluteJoint(jointNameA, base, transformAToPredecessor, jointAxis);
      RevoluteJoint jointB = new RevoluteJoint(jointNameB, base, transformBToPredecessor, jointAxis);
      RigidBodyBasics bodyBC = rigidBodyBuilder.build(bodyNameBC, jointB, bodyInertiaBC, bodyMassBC, bodyInertiaPoseBC);
      RigidBodyBasics bodyDA = rigidBodyBuilder.build(bodyNameDA, jointA, bodyInertiaDA, bodyMassDA, bodyInertiaPoseDA);
      RevoluteJoint jointC = new RevoluteJoint(jointNameC, bodyBC, transformCToB, jointAxis);
      RevoluteJoint jointD = new RevoluteJoint(jointNameD, bodyDA, transformDToA, jointAxis);

      RigidBodyTransform transformDToC = new RigidBodyTransform();
      transformDToC.multiply(transformCToB);
      transformDToC.multiply(transformBToPredecessor);
      transformDToC.multiplyInvertOther(transformAToPredecessor);
      transformDToC.multiplyInvertOther(transformDToA);

      if (loopClosureJointIndex == 2)
      { // The loop closure happens at joint C
         RigidBody end = new RigidBody(name + "InternalEnd", jointD, new Matrix3D(), 0.0, new RigidBodyTransform());
         jointC.setupLoopClosure(end, transformDToC);
      }
      else
      { // The loop closure happens at joint D
         RigidBody end = new RigidBody(name + "InternalEnd", jointC, new Matrix3D(), 0.0, new RigidBodyTransform());
         RigidBodyTransform transformCToD = new RigidBodyTransform();
         transformCToD.setAndInvert(transformDToC);
         jointD.setupLoopClosure(end, transformCToD);
      }

      fourBarFunction = new FourBarKinematicLoopFunction(name, Arrays.asList(jointA, jointB, jointC, jointD), actuatedJointIndex);
      if (!fourBarFunction.isCrossed())
         throw new IllegalArgumentException("The given joint configuration does not represent a cross four bar.");
      setIKSolver(new CrossFourBarJointIKBinarySolver(1.0e-5));

      this.name = name;
      this.predecessor = predecessor;
      predecessor.addChildJoint(this);
      nameId = JointReadOnly.computeNameId(this);

      if (getJointB().isLoopClosure() || getJointC().isLoopClosure())
      {
         beforeJointFrame = getJointA().getFrameBeforeJoint();
         afterJointFrame = getJointD().getFrameAfterJoint();
      }
      else
      {
         beforeJointFrame = getJointB().getFrameBeforeJoint();
         afterJointFrame = getJointC().getFrameAfterJoint();
      }

      unitTwists = Collections.singletonList(unitJointTwist);
      jointTwist = MecanoFactories.newTwistReadOnly(this::getQd, unitJointTwist);
      jointAcceleration = MecanoFactories.newSpatialAccelerationVectorReadOnly(this::getQdd, unitJointAcceleration, jointBiasAcceleration);
   }

   /**
    * Utility method to compute the name of the internal joints and rigid-bodies used for cross four bar joint or revolute twins joint.
    *
    * @param jointName         the name of the joint to use if {@code internalName} is {@code null}.
    * @param internalName      the name of the joint.
    * @param defaultNameSuffix the suffix to use if {@code internalName} is {@code null}.
    * @return the name of the internal joint.
    */
   public static String getInternalName(String jointName, String internalName, String defaultNameSuffix)
   {
      if (internalName == null)
         return jointName + "_" + defaultNameSuffix;

      JointReadOnly.checkJointNameSanity(internalName);
      return internalName;
   }

   /**
    * Creates a new cross four bar joint that is to wrap the 4 given revolute joints into a single
    * 1-DoF joint.
    * <p>
    * <b>WARNING: This joint is somewhat tricky to create, as the 4 given revolute joints are only used
    * as a template to setup this complex joint and for internal calculation.</b><br>
    * Here are the expected construction steps of a robot system:
    * <ol>
    * <li>The user should create the branch of the robot up to the 4 revolute joints composing the four
    * bar.
    * <li>Instead of adding the successor to the last 2 joints, create a dummy rigid-body to terminate
    * the four bar.
    * <li>Create the {@code CrossFourBarJoint} given the four joints.
    * <li>Finally proceed to creating the subtree following the four bar by attaching the next
    * successor to this new four bar joint. The transform, a.k.a. inertia pose, that is to be provided
    * to the successor should expressed with respect to {@link #getJointD()}'s frame after joint.
    * </ol>
    * </p>
    *
    * @param name               the name of this joint.
    * @param fourBarJoints      the 4 revolute joints composing the four bar.
    * @param actuatedJointIndex the index in {@code fourBarJoints} of the joints that is actuated.
    * @throws IllegalArgumentException if the given joints do not represent a cross four bar joints.
    * @throws IllegalArgumentException if a subtree is already attached to the last two joints closing
    *       the four bar.
    * @see FourBarKinematicLoopFunction#FourBarKinematicLoopFunction(String, RevoluteJointBasics[],
    *       int)
    */
   public CrossFourBarJoint(String name, RevoluteJointBasics[] fourBarJoints, int actuatedJointIndex)
   {
      fourBarFunction = new FourBarKinematicLoopFunction(name, fourBarJoints, actuatedJointIndex);
      if (!fourBarFunction.isCrossed())
         throw new IllegalArgumentException("The given joint configuration does not represent a cross four bar.");
      setIKSolver(new CrossFourBarJointIKBinarySolver(1.0e-5));

      this.name = name;
      predecessor = getJointA().getPredecessor();
      // Detaching the joints A & B from the predecessor and attaching this joint.
      predecessor.getChildrenJoints().remove(getJointA());
      predecessor.getChildrenJoints().remove(getJointB());
      predecessor.addChildJoint(this);

      if (getJointB().isLoopClosure() || getJointC().isLoopClosure())
      {
         beforeJointFrame = getJointA().getFrameBeforeJoint();
         afterJointFrame = getJointD().getFrameAfterJoint();
      }
      else
      {
         beforeJointFrame = getJointB().getFrameBeforeJoint();
         afterJointFrame = getJointC().getFrameAfterJoint();
      }

      nameId = JointReadOnly.computeNameId(this);

      unitTwists = Collections.singletonList(unitJointTwist);
      jointTwist = MecanoFactories.newTwistReadOnly(this::getQd, unitJointTwist);
      jointAcceleration = MecanoFactories.newSpatialAccelerationVectorReadOnly(this::getQdd, unitJointAcceleration, jointBiasAcceleration);
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      this.successor = successor;
      jointWrench = MecanoFactories.newWrenchReadOnly(this::getTau, unitJointWrench);
   }

   /**
    * Sets the solver to use for computing the four bar configuration given the joint angle via
    * {@link #setQ(double)}.
    *
    * @param ikSolver the solver to use.
    */
   public void setIKSolver(CrossFourBarJointIKSolver ikSolver)
   {
      this.ikSolver = ikSolver;
      ikSolver.setConverters(fourBarFunction.getConverters());
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void updateFrame()
   {
      fourBarFunction.updateState(true, true);
      getJointA().getFrameBeforeJoint().update();
      getJointB().getFrameBeforeJoint().update();
      getJointC().getFrameBeforeJoint().update();
      getJointD().getFrameBeforeJoint().update();
      getJointA().getFrameAfterJoint().update();
      getJointB().getFrameAfterJoint().update();
      getJointC().getFrameAfterJoint().update();
      getJointD().getFrameAfterJoint().update();

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

         unitJointWrench.setIncludingFrame(fourBarFunction.getActuatedJoint().getUnitJointTwist());
         unitJointWrench.changeFrame(afterJointFrame);
         unitJointWrench.setBodyFrame(getSuccessor().getBodyFixedFrame());
      }
   }

   /**
    * Computes the unit-twist for the given cross four bar joint and stores the result in the given
    * twist.
    * <p>
    * This method relies on {@link CrossFourBarJointReadOnly#getLoopJacobian()} to be up-to-date.
    * </p>
    *
    * @param joint                the joint to update the unit-twist of. Not modified.
    * @param unitJointTwistToPack the twist in which the result is stored. Modified.
    */
   public static void updateUnitJointTwist(CrossFourBarJointReadOnly joint, TwistBasics unitJointTwistToPack)
   {
      DMatrix loopJacobian = joint.getLoopJacobian();

      RevoluteJointReadOnly joint1, joint2;
      double J_1, J_2;

      if (joint.getFrameBeforeJoint() == joint.getJointA().getFrameBeforeJoint())
      {
         joint1 = joint.getJointA();
         joint2 = joint.getJointD();
         J_1 = loopJacobian.get(0, 0);
         J_2 = loopJacobian.get(3, 0);
      }
      else
      {
         joint1 = joint.getJointB();
         joint2 = joint.getJointC();
         J_1 = loopJacobian.get(1, 0);
         J_2 = loopJacobian.get(2, 0);
      }

      TwistReadOnly j1UnitTwist = joint1.getUnitJointTwist();
      TwistReadOnly j2UnitTwist = joint2.getUnitJointTwist();

      unitJointTwistToPack.setIncludingFrame(j1UnitTwist);
      unitJointTwistToPack.scale(J_1);
      unitJointTwistToPack.setBodyFrame(joint2.getFrameBeforeJoint());
      unitJointTwistToPack.changeFrame(joint2.getFrameAfterJoint());
      unitJointTwistToPack.getAngularPart().scaleAdd(J_2, j2UnitTwist.getAngularPart(), unitJointTwistToPack.getAngularPart());
      unitJointTwistToPack.getLinearPart().scaleAdd(J_2, j2UnitTwist.getLinearPart(), unitJointTwistToPack.getLinearPart());
      unitJointTwistToPack.scale(1.0 / (J_1 + J_2));
      unitJointTwistToPack.setBodyFrame(joint.getFrameAfterJoint());
   }

   /**
    * Computes the bias acceleration for the given cross four bar joint and stores the result in the
    * given spatial acceleration.
    * <p>
    * This method relies on {@link CrossFourBarJointReadOnly#getLoopConvectiveTerm()} and
    * {@link CrossFourBarJointReadOnly#getUnitJointAcceleration()} to be up-to-date.
    *
    * @param joint                 the joint to compute the bias acceleration of. Not Modified.
    * @param deltaTwist            twist used to stores intermediate result. Modified.
    * @param bodyTwist             twist used to stores intermediate result. Modified.
    * @param jointBiasAcceleration the spatial acceleration in which the result is stored. Modified.
    */
   public static void updateBiasAcceleration(CrossFourBarJointReadOnly joint,
                                             TwistBasics deltaTwist,
                                             TwistBasics bodyTwist,
                                             SpatialAccelerationBasics jointBiasAcceleration)
   {
      DMatrix loopConvectiveTerm = joint.getLoopConvectiveTerm();
      RevoluteJointReadOnly joint1, joint2;
      double c_1, c_2;

      if (joint.getFrameBeforeJoint() == joint.getJointA().getFrameBeforeJoint())
      {
         joint1 = joint.getJointA();
         joint2 = joint.getJointD();
         c_1 = loopConvectiveTerm.get(0, 0);
         c_2 = loopConvectiveTerm.get(3, 0);
      }
      else
      {
         joint1 = joint.getJointB();
         joint2 = joint.getJointC();
         c_1 = loopConvectiveTerm.get(1, 0);
         c_2 = loopConvectiveTerm.get(2, 0);
      }

      SpatialAccelerationReadOnly jUnitAccel = joint.getUnitJointAcceleration();
      SpatialAccelerationReadOnly j1UnitAccel = joint1.getUnitJointAcceleration();
      SpatialAccelerationReadOnly j2UnitAccel = joint2.getUnitJointAcceleration();

      /*
       * This next block is for computing the bias acceleration. I ended up using tests to figure out
       * exactly what it should, but I feel that it can be simplified.
       */
      joint2.getFrameAfterJoint().getTwistRelativeToOther(joint1.getFrameAfterJoint(), deltaTwist);
      joint2.getFrameBeforeJoint().getTwistRelativeToOther(joint1.getFrameBeforeJoint(), bodyTwist);
      deltaTwist.changeFrame(joint2.getFrameAfterJoint());
      bodyTwist.changeFrame(joint2.getFrameAfterJoint());
      jointBiasAcceleration.setIncludingFrame(j1UnitAccel);
      jointBiasAcceleration.scale(c_1);
      jointBiasAcceleration.setBodyFrame(joint2.getFrameBeforeJoint());
      jointBiasAcceleration.changeFrame(joint2.getFrameAfterJoint(), deltaTwist, bodyTwist);

      FixedFrameVector3DBasics jointBiasAngularAcceleration = jointBiasAcceleration.getAngularPart();
      FixedFrameVector3DBasics jointBiasLinearAcceleration = jointBiasAcceleration.getLinearPart();
      jointBiasAngularAcceleration.scaleAdd(c_2, j2UnitAccel.getAngularPart(), jointBiasAngularAcceleration);
      jointBiasLinearAcceleration.scaleAdd(c_2, j2UnitAccel.getLinearPart(), jointBiasLinearAcceleration);

      jointBiasAngularAcceleration.scaleAdd(-(c_1 + c_2), jUnitAccel.getAngularPart(), jointBiasAngularAcceleration);
      jointBiasLinearAcceleration.scaleAdd(-(c_1 + c_2), jUnitAccel.getLinearPart(), jointBiasLinearAcceleration);

      jointBiasAcceleration.setBodyFrame(joint.getFrameAfterJoint());
   }

   /**
    * Gets the reference to the calculator this joint uses to compute the internal Jacobian and
    * convective term.
    *
    * @return the reference to the calculator.
    */
   public FourBarKinematicLoopFunction getFourBarFunction()
   {
      return fourBarFunction;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RevoluteJointBasics getActuatedJoint()
   {
      return fourBarFunction.getActuatedJoint();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RevoluteJointBasics getJointA()
   {
      return fourBarFunction.getJointA();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RevoluteJointBasics getJointB()
   {
      return fourBarFunction.getJointB();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RevoluteJointBasics getJointC()
   {
      return fourBarFunction.getJointC();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public RevoluteJointBasics getJointD()
   {
      return fourBarFunction.getJointD();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public int getActuatedJointIndex()
   {
      return fourBarFunction.getActuatedJointIndex();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public DMatrixRMaj getLoopJacobian()
   {
      return fourBarFunction.getLoopJacobian();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public DMatrixRMaj getLoopConvectiveTerm()
   {
      return fourBarFunction.getLoopConvectiveTerm();
   }

   /**
    * Gets the reference to the solver this joint uses to compute the actuated joint angle given this
    * joint angle.
    *
    * @return the reference to the solver.
    */
   public CrossFourBarJointIKSolver getIKSolver()
   {
      return ikSolver;
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
      // TODO This method ignores potentially non-zero torques set in the other joints.
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      fourBarFunction.updateEffort();
      if (getActuatedJoint() == getJointA() || getActuatedJoint() == getJointD())
         return getActuatedJoint().getTau() / (loopJacobian.get(0) + loopJacobian.get(3));
      else
         return getActuatedJoint().getTau() / (loopJacobian.get(1) + loopJacobian.get(2));
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
      return ikSolver.solve(q, fourBarFunction.getActuatedVertex());
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double computeActuatedJointQd(double qd)
   {
      fourBarFunction.updateState(false, false);
      // qd = (J_A + J_D) qd_M = (J_B + J_C) qd_M
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      return qd / (loopJacobian.get(0) + loopJacobian.get(3));
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double computeActuatedJointQdd(double qdd)
   {
      fourBarFunction.updateState(false, false);
      // qdd = (J_A + J_D) qdd_M + c_A + c_D = (J_B + J_C) qdd_M + c_B + c_C
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      DMatrixRMaj loopConvectiveTerm = fourBarFunction.getLoopConvectiveTerm();
      qdd = qdd - loopConvectiveTerm.get(0) - loopConvectiveTerm.get(3);
      double qdd_actuated = qdd / (loopJacobian.get(0) + loopJacobian.get(3));
      return qdd_actuated;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double computeActuatedJointTau(double tau)
   {
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      if (getActuatedJoint() == getJointA() || getActuatedJoint() == getJointD())
         return ((loopJacobian.get(0) + loopJacobian.get(3)) * tau);
      else
         return ((loopJacobian.get(1) + loopJacobian.get(2)) * tau);
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
      return Math.max(CrossFourBarJointBasics.super.getJointLimitLower(), jointLimitLower);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getJointLimitUpper()
   {
      return Math.min(CrossFourBarJointBasics.super.getJointLimitUpper(), jointLimitUpper);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getVelocityLimitLower()
   {
      return Math.max(CrossFourBarJointBasics.super.getVelocityLimitLower(), velocityLimitLower);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getVelocityLimitUpper()
   {
      return Math.min(CrossFourBarJointBasics.super.getVelocityLimitUpper(), velocityLimitUpper);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getEffortLimitLower()
   {
      return Math.max(CrossFourBarJointBasics.super.getEffortLimitLower(), effortLimitLower);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getEffortLimitUpper()
   {
      return Math.min(CrossFourBarJointBasics.super.getEffortLimitUpper(), effortLimitUpper);
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
   public static CrossFourBarJoint cloneCrossFourBarJoint(CrossFourBarJointReadOnly original, ReferenceFrame stationaryFrame, String cloneSuffix)
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
   public static CrossFourBarJoint cloneCrossFourBarJoint(CrossFourBarJointReadOnly original, RigidBodyBasics clonePredecessor, String cloneSuffix)
   {
      return (CrossFourBarJoint) MultiBodySystemFactories.DEFAULT_JOINT_BUILDER.cloneCrossFourBarJoint(original, cloneSuffix, clonePredecessor);
   }
}
