package us.ihmc.mecano.multiBodySystem;

import java.util.Collections;
import java.util.List;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.fourBar.CrossFourBarJointIKBinarySolver;
import us.ihmc.mecano.fourBar.CrossFourBarJointIKSolver;
import us.ihmc.mecano.fourBar.FourBarKinematicLoopFunction;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
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
 * 
 * where A, B, C, and D are all revolute joint around the same axis. It is assumed that only one
 * joint in this sub-system composed of {A, B, C, and D} is a torque source, this is the master
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

   /** Variable to store intermediate results for garbage-free operations. */
   private final Vector3D rotationVector = new Vector3D();

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
    * @param name             the name of this joint.
    * @param fourBarJoints    the 4 revolute joints composing the four bar.
    * @param masterJointIndex the index in {@code fourBarJoints} of the joints that is actuated.
    * @throws IllegalArgumentException if the given joints do not represent a cross four bar joints.
    * @throws IllegalArgumentException if a subtree is already attached to the last two joints closing
    *                                  the four bar.
    * @see FourBarKinematicLoopFunction#FourBarKinematicLoopFunction(String, RevoluteJointBasics[],
    *      int)
    */
   public CrossFourBarJoint(String name, RevoluteJointBasics[] fourBarJoints, int masterJointIndex)
   {
      fourBarFunction = new FourBarKinematicLoopFunction(name, fourBarJoints, masterJointIndex);
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

      if (predecessor.isRootBody())
         nameId = name;
      else
         nameId = predecessor.getParentJoint().getNameId() + NAME_ID_SEPARATOR + name;

      unitTwists = Collections.singletonList(unitJointTwist);
      jointTwist = MecanoFactories.newTwistReadOnly(this::getQd, unitJointTwist);
      jointAcceleration = MecanoFactories.newSpatialAccelerationVectorReadOnly(this::getQdd, unitJointAcceleration, jointBiasAcceleration);
   }

   @Override
   public void setSuccessor(RigidBodyBasics successor)
   {
      if (this.successor != null)
         throw new IllegalStateException("The successor of this joint has already been set.");

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

   /** {@inheritDoc} */
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

   /** {@inheritDoc} */
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

         unitJointWrench.setIncludingFrame(fourBarFunction.getMasterJoint().getUnitJointTwist());
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

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getMasterJoint()
   {
      return fourBarFunction.getMasterJoint();
   }

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getJointA()
   {
      return fourBarFunction.getJointA();
   }

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getJointB()
   {
      return fourBarFunction.getJointB();
   }

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getJointC()
   {
      return fourBarFunction.getJointC();
   }

   /** {@inheritDoc} */
   @Override
   public RevoluteJointBasics getJointD()
   {
      return fourBarFunction.getJointD();
   }

   /** {@inheritDoc} */
   @Override
   public int getMasterJointIndex()
   {
      return fourBarFunction.getMasterJointIndex();
   }

   /** {@inheritDoc} */
   @Override
   public DMatrixRMaj getLoopJacobian()
   {
      return fourBarFunction.getLoopJacobian();
   }

   /** {@inheritDoc} */
   @Override
   public DMatrixRMaj getLoopConvectiveTerm()
   {
      return fourBarFunction.getLoopConvectiveTerm();
   }

   /**
    * Gets the reference to the solver this joint uses to compute the master joint angle given this
    * joint angle.
    * 
    * @return the reference to the solver.
    */
   public CrossFourBarJointIKSolver getIKSolver()
   {
      return ikSolver;
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

   /**
    * This feature is not supported.
    */
   @Override
   public void setupLoopClosure(RigidBodyBasics successor, RigidBodyTransformReadOnly transformFromSuccessorParentJoint)
   {
      throw new UnsupportedOperationException("Loop closure using a four bar joint has not been implemented.");
   }

   /** {@inheritDoc} */
   @Override
   public double getTau()
   {
      // TODO This method ignores potentially non-zero torques set in the other joints.
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      fourBarFunction.updateEffort();
      if (getMasterJoint() == getJointA() || getMasterJoint() == getJointD())
         return getMasterJoint().getTau() / (loopJacobian.get(0) + loopJacobian.get(3));
      else
         return getMasterJoint().getTau() / (loopJacobian.get(1) + loopJacobian.get(2));
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

   /** {@inheritDoc} */
   @Override
   public WrenchReadOnly getJointWrench()
   {
      return jointWrench;
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
   public double computeMasterJointQ(double q)
   {
      return ikSolver.solve(q, fourBarFunction.getMasterVertex());
   }

   /** {@inheritDoc} */
   @Override
   public double computeMasterJointQd(double qd)
   {
      fourBarFunction.updateState(false, false);
      // qd = (J_A + J_D) qd_M = (J_B + J_C) qd_M
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      return qd / (loopJacobian.get(0) + loopJacobian.get(3));
   }

   /** {@inheritDoc} */
   @Override
   public double computeMasterJointQdd(double qdd)
   {
      fourBarFunction.updateState(false, false);
      // qdd = (J_A + J_D) qdd_M + c_A + c_D = (J_B + J_C) qdd_M + c_B + c_C
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      DMatrixRMaj loopConvectiveTerm = fourBarFunction.getLoopConvectiveTerm();
      qdd = qdd - loopConvectiveTerm.get(0) - loopConvectiveTerm.get(3);
      double qdd_master = qdd / (loopJacobian.get(0) + loopJacobian.get(3));
      return qdd_master;
   }

   /** {@inheritDoc} */
   @Override
   public double computeMasterJointTau(double tau)
   {
      DMatrixRMaj loopJacobian = fourBarFunction.getLoopJacobian();
      if (getMasterJoint() == getJointA() || getMasterJoint() == getJointD())
         return ((loopJacobian.get(0) + loopJacobian.get(3)) * tau);
      else
         return ((loopJacobian.get(1) + loopJacobian.get(2)) * tau);
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
   public static CrossFourBarJoint cloneCrossFourBarJoint(CrossFourBarJoint original, ReferenceFrame stationaryFrame, String cloneSuffix)
   {
      RigidBodyBasics originalPredecessor = original.getPredecessor();
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
   public static CrossFourBarJoint cloneCrossFourBarJoint(CrossFourBarJoint original, RigidBodyBasics clonePredecessor, String cloneSuffix)
   {
      return (CrossFourBarJoint) MultiBodySystemFactories.DEFAULT_JOINT_BUILDER.cloneCrossFourBarJoint(original, cloneSuffix, clonePredecessor);
   }
}
