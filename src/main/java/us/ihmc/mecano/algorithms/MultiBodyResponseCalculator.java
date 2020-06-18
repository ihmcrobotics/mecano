package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator.ArticulatedBodyRecursionStep;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Inspired from Mirtich's thesis, this calculator allows to evaluate the perturbation in terms of
 * change in acceleration (or twist) due to a wrench (or impulse) applied on a rigid-body or a
 * joint.
 * <p>
 * This can be used to compute an adequate wrench (or impulse) to apply on a rigid-body or joint to
 * obtain a given resulting acceleration (or twist).
 * </p>
 * <p>
 * Example of how to use this calculator:
 *
 * <pre>
 * MultiBodyResponseCalculator calculator = new MultiBodyResponseCalculator(multiBodySystem);
 * calculator.getForwardDynamicsCalculator().compute();
 * // To compute the linear part of the inverse of the apparent inertia for targetRigidBody:
 * calculator.computeApparentLinearInertiaInverse(targetRigidBody, pointInTarget, externalWrenchFrame);
 *
 * // The following is for propagating a disturbance to the multi-body system and evaluate the resulting change in motion:
 * // First apply the disturbance:
 * calculator.applyWrench(targetRigidBody, externalWrench);
 * // To obtain the resulting change in acceleration caused by the externalWrench on another rigid-body:
 * SpatialAccelerationReadOnly resultingAccelerationChangeOnOtherBody = calculator.getAccelerationChangeProvider().getAccelerationOfBody(anotherRigidBody);
 * // To obtain the change in joint accelerationL
 * DenseMatrix64F jointAccelerationChange = calculator.propagateWrench();
 * </pre>
 *
 * The equivalent calculations can be done using an impulse as disturbance the change in motion then
 * being in the velocity space.
 * </p>
 * <p>
 * Mirtich, Brian Vincent. <i>Impulse-based dynamic simulation of rigid body systems</i>. University
 * of California, Berkeley, 1996.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class MultiBodyResponseCalculator
{
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;
   /** The root of the internal recursive algorithm. */
   private final ResponseRecursionStep initialRecursionStep;
   /** Map to quickly retrieve information for each rigid-body. */
   private final Map<RigidBodyReadOnly, ResponseRecursionStep> rigidBodyToRecursionStepMap = new HashMap<>();
   /**
    * The output of this algorithm: the acceleration/velocity matrix for all the joints to consider.
    */
   private final DMatrixRMaj jointMotionChangeMatrix;
   /**
    * This algorithm relies on the pre-computed internal data from a forward dynamics algorithm.
    */
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;

   /**
    * Extension of this algorithm into a rigid-body acceleration provider that can be used to retrieve
    * change in acceleration for any rigid-body of the system.
    */
   private final RigidBodyAccelerationProvider accelerationChangeProvider;
   /**
    * Extension of this algorithm into a rigid-body twist provider that can be used to retrieve change
    * in twist for any rigid-body of the system.
    */
   private final RigidBodyTwistProvider twistChangeProvider;

   private enum ResponseType
   {
      ACCELERATION, TWIST
   }

   private ResponseType currentResponseType = null;
   private final DMatrixRMaj singleElementMatrix = new DMatrixRMaj(1, 1);

   /**
    * Creates a calculator for computing the response to external disturbances for a system defined by
    * the given {@code input}.
    * <p>
    * This calculator creates a {@link ForwardDynamicsCalculator} that needs to configured and updated
    * manually by the user before using the features of this calculator.
    * </p>
    *
    * @param input the definition of the system to be evaluated by this calculator.
    */
   public MultiBodyResponseCalculator(MultiBodySystemReadOnly input)
   {
      this(new ForwardDynamicsCalculator(input));
   }

   /**
    * Creates a calculator for computing the response to external disturbances for a system defined by
    * the given {@code forwardDynamicsCalculator.getInput()}.
    * <p>
    * This calculator does not manage the given {@code forwardDynamicsCalculator}. It is on the user to
    * update the forward dynamics before accessing features of this calculator.
    * </p>
    *
    * @param forwardDynamicsCalculator the forward dynamics calculator required to perform additional
    *                                  calculation in this calculator.
    */
   public MultiBodyResponseCalculator(ForwardDynamicsCalculator forwardDynamicsCalculator)
   {
      this.forwardDynamicsCalculator = forwardDynamicsCalculator;
      input = forwardDynamicsCalculator.getInput();
      initialRecursionStep = new ResponseRecursionStep(forwardDynamicsCalculator.getInitialRecursionStep(), null);
      buildMultiBodyTree(initialRecursionStep);

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      jointMotionChangeMatrix = new DMatrixRMaj(nDoFs, 1);

      accelerationChangeProvider = RigidBodyAccelerationProvider.toRigidBodyAccelerationProvider(buildAccelerationSupplier(), input.getInertialFrame());
      twistChangeProvider = RigidBodyTwistProvider.toRigidBodyTwistProvider(buildTwistSupplier(), input.getInertialFrame());
   }

   private Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> buildAccelerationSupplier()
   {
      SpatialAcceleration bodyAcceleration = new SpatialAcceleration();

      Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> accelerationFunction = body ->
      {
         if (currentResponseType == ResponseType.TWIST)
            throw new IllegalStateException("This calculator is currently setup for calculating twists.");

         ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);
         if (recursionStep == null)
            return null;
         if (currentResponseType == null || !initialRecursionStep.isUpToDate)
         { // This calculator has not been initialized with an wrench yet.
            bodyAcceleration.setToZero(body.getBodyFixedFrame(), input.getInertialFrame(), body.getBodyFixedFrame());
         }
         else
         {
            recursionStep.updateRigidBodyMotionChange();
            // The algorithm computes the acceleration expressed in the parent joint frame.
            // To prevent unnecessary computation, let's only change the frame when needed.
            bodyAcceleration.setIncludingFrame(recursionStep.rigidBodyMotionChange);
            bodyAcceleration.changeFrame(body.getBodyFixedFrame());
         }
         return bodyAcceleration;
      };
      return accelerationFunction;
   }

   private Function<RigidBodyReadOnly, TwistReadOnly> buildTwistSupplier()
   {
      Twist bodyTwist = new Twist();

      Function<RigidBodyReadOnly, TwistReadOnly> twistFunction = body ->
      {
         if (currentResponseType == ResponseType.ACCELERATION)
            throw new IllegalStateException("This calculator is currently setup for calculating accelerations.");

         ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);
         if (recursionStep == null)
            return null;

         if (!initialRecursionStep.isUpToDate)
         { // This calculator has not been initialized with an impulse yet.
            bodyTwist.setToZero(body.getBodyFixedFrame(), input.getInertialFrame(), body.getBodyFixedFrame());
         }
         else
         {
            recursionStep.updateRigidBodyMotionChange();
            // The algorithm computes the twist expressed in the parent joint frame.
            // To prevent unnecessary computation, let's only change the frame when needed.
            bodyTwist.setIncludingFrame(recursionStep.rigidBodyMotionChange);
            bodyTwist.changeFrame(body.getBodyFixedFrame());
         }
         return bodyTwist;
      };
      return twistFunction;
   }

   private void buildMultiBodyTree(ResponseRecursionStep recursionStep)
   {
      for (ArticulatedBodyRecursionStep childInertia : recursionStep.articulatedBodyRecursionStep.children)
      {
         ResponseRecursionStep child = new ResponseRecursionStep(childInertia, recursionStep);
         rigidBodyToRecursionStepMap.put(childInertia.rigidBody, child);
         buildMultiBodyTree(child);
      }
   }

   /**
    * Gets the internal forward-dynamics calculator that this calculator is using.
    *
    * @return the forward dynamics calculator.
    */
   public ForwardDynamicsCalculator getForwardDynamicsCalculator()
   {
      return forwardDynamicsCalculator;
   }

   /**
    * Clears the internal state of this calculator.
    */
   public void reset()
   {
      currentResponseType = null;
      initialRecursionStep.reset();
   }

   /**
    * Computes the matrix representing the inverse of the apparent inertia expressed at
    * {@code inertiaFrame} such that:
    *
    * <pre>
    * / &Delta;&omega;Dot<sub>target</sub> \     <sup> </sup> <sup>  </sup> / &tau;<sub>target</sub> \
    * |   <sub>      </sub>    | = (I<sup>A</sup>)<sup>-1</sup> |  <sub>      </sub> |
    * \ &Delta;&alpha;<sub>target</sub>    /     <sup> </sup> <sup>  </sup> \ F<sub>target</sub> /
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>(I<sup>A</sup>)<sup>-1</sup></tt> is the inverse of the 6-by-6 linear part of the
    * apparent inertia matrix.
    * <li><tt>&tau;<sub>target</sub></tt> is a torque applied at {@code inertiaFrame}.
    * <li><tt>F<sub>target</sub></tt> is a force applied at {@code inertiaFrame}.
    * <li><tt>&Delta;&omega;Dot<sub>target</sub></tt> is the resulting change in angular acceleration
    * of {@code target} at {@code inertiaFrame} due to a wrench (torque and force).
    * <li><tt>&Delta;&alpha;<sub>target</sub></tt> is the resulting change in linear acceleration of
    * {@code target} at {@code inertiaFrame} due to a wrench (torque and force).
    * </ul>
    * <p>
    * Note that the apparent inertia can also be used to relate an impulse to a change in twist as
    * follows:
    * </p>
    *
    * <pre>
    * / &Delta;&omega;<sub>target</sub> \     <sup> </sup> <sup>  </sup> / Y<sup>ang</sup><sub>target</sub> \
    * |   <sub>      </sub> | = (I<sup>A</sup>)<sup>-1</sup> |  <sub>      </sub><sup>   </sup> |
    * \ &Delta;&nu;<sub>target</sub> /     <sup> </sup> <sup>  </sup> \ Y<sup>lin</sup><sub>target</sub> /
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>Y<sup>ang</sup><sub>target</sub></sub></tt> is the angular part of an impulse applied at
    * {@code inertiaFrame}.
    * <li><tt>Y<sup>lin</sup><sub>target</sub></tt> is the linear part of an impulse applied at
    * {@code inertiaFrame}.
    * <li><tt>&Delta;&omega;<sub>target</sub></tt> is the resulting change in angular velocity of
    * {@code target} at {@code inertiaFrame} due to an impulse.
    * <li><tt>&Delta;&nu;<sub>target</sub></tt> is the resulting change in linear velocity of
    * {@code target} at {@code inertiaFrame} due to an impulse.
    * </ul>
    *
    * @param target                       the rigid-body to compute the apparent inertia at.
    * @param inertiaFrame                 the frame at which the apparent inertia is to be expressed.
    * @param apparentSpatialInertiaToPack the matrix in which to store the result. Modified.
    * @return {@code true} is the apparent inertia matrix was successfully computed, {@code false}
    *         otherwise.
    */
   public boolean computeRigidBodyApparentSpatialInertiaInverse(RigidBodyReadOnly target, ReferenceFrame inertiaFrame,
                                                                DMatrix1Row apparentSpatialInertiaToPack)
   {
      return computeRigidBodyApparentSpatialInertiaInverse(target, inertiaFrame, null, apparentSpatialInertiaToPack);
   }

   /**
    * Computes the matrix representing the inverse of the apparent inertia expressed at
    * {@code inertiaFrame} such that:
    *
    * <pre>
    * / &Delta;&omega;Dot<sub>target</sub> \     <sup> </sup> <sup>  </sup> / &tau;<sub>target</sub> \
    * |   <sub>      </sub>    | = (I<sup>A</sup>)<sup>-1</sup> |  <sub>      </sub> |
    * \ &Delta;&alpha;<sub>target</sub>    /     <sup> </sup> <sup>  </sup> \ F<sub>target</sub> /
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>(I<sup>A</sup>)<sup>-1</sup></tt> is the inverse of the 6-by-6 linear part of the
    * apparent inertia matrix.
    * <li><tt>&tau;<sub>target</sub></tt> is a torque applied at {@code inertiaFrame}.
    * <li><tt>F<sub>target</sub></tt> is a force applied at {@code inertiaFrame}.
    * <li><tt>&Delta;&omega;Dot<sub>target</sub></tt> is the resulting change in angular acceleration
    * of {@code target} at {@code inertiaFrame} due to a wrench (torque and force).
    * <li><tt>&Delta;&alpha;<sub>target</sub></tt> is the resulting change in linear acceleration of
    * {@code target} at {@code inertiaFrame} due to a wrench (torque and force).
    * </ul>
    * <p>
    * Note that the apparent inertia can also be used to relate an impulse to a change in twist as
    * follows:
    * </p>
    *
    * <pre>
    * / &Delta;&omega;<sub>target</sub> \     <sup> </sup> <sup>  </sup> / Y<sup>ang</sup><sub>target</sub> \
    * |   <sub>      </sub> | = (I<sup>A</sup>)<sup>-1</sup> |  <sub>      </sub><sup>   </sup> |
    * \ &Delta;&nu;<sub>target</sub> /     <sup> </sup> <sup>  </sup> \ Y<sup>lin</sup><sub>target</sub> /
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>Y<sup>ang</sup><sub>target</sub></sub></tt> is the angular part of an impulse applied at
    * {@code inertiaFrame}.
    * <li><tt>Y<sup>lin</sup><sub>target</sub></tt> is the linear part of an impulse applied at
    * {@code inertiaFrame}.
    * <li><tt>&Delta;&omega;<sub>target</sub></tt> is the resulting change in angular velocity of
    * {@code target} at {@code inertiaFrame} due to an impulse.
    * <li><tt>&Delta;&nu;<sub>target</sub></tt> is the resulting change in linear velocity of
    * {@code target} at {@code inertiaFrame} due to an impulse.
    * </ul>
    *
    * @param target                       the rigid-body to compute the apparent inertia at.
    * @param inertiaFrame                 the frame at which the apparent inertia is to be expressed.
    * @param selectedAxes                 a 6-element array of boolean allowing to specify for which
    *                                     axes the apparent inertia is to be computed. The resulting
    *                                     matrix is a 6-by-6 matrix regardless of whether all axes are
    *                                     selected or not. This allows to skip calculation for axes
    *                                     that are not of interest.
    * @param apparentSpatialInertiaToPack the matrix in which to store the result. Modified.
    * @return {@code true} is the apparent inertia matrix was successfully computed, {@code false}
    *         otherwise.
    */
   public boolean computeRigidBodyApparentSpatialInertiaInverse(RigidBodyReadOnly target, ReferenceFrame inertiaFrame, boolean[] selectedAxes,
                                                                DMatrix1Row apparentSpatialInertiaToPack)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      ReferenceFrame bodyFrame = target.getBodyFixedFrame();

      apparentSpatialInertiaToPack.reshape(6, 6);

      reset();

      for (int axis = 0; axis < 3; axis++)
      {
         if (selectedAxes == null || selectedAxes[axis])
         {
            recursionStep.testDisturbancePlus.setIncludingFrame(bodyFrame, inertiaFrame, Axis3D.values[axis], EuclidCoreTools.zeroVector3D);
            recursionStep.initializeDisturbance();
            recursionStep.updateRigidBodyMotionChange();
            recursionStep.rigidBodyMotionChange.changeFrame(inertiaFrame);
            recursionStep.rigidBodyMotionChange.get(0, axis, apparentSpatialInertiaToPack);
            reset();
         }
         else
         {
            apparentSpatialInertiaToPack.unsafe_set(0, axis, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(1, axis, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(2, axis, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(0, axis + 3, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(2, axis + 3, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(3, axis + 3, 0.0);
         }
      }

      for (int axis = 0; axis < 3; axis++)
      {
         if (selectedAxes == null || selectedAxes[axis + 3])
         {
            recursionStep.testDisturbancePlus.setIncludingFrame(bodyFrame, inertiaFrame, EuclidCoreTools.zeroVector3D, Axis3D.values[axis]);
            recursionStep.initializeDisturbance();
            recursionStep.updateRigidBodyMotionChange();
            recursionStep.rigidBodyMotionChange.changeFrame(inertiaFrame);
            recursionStep.rigidBodyMotionChange.get(0, axis + 3, apparentSpatialInertiaToPack);
            reset();
         }
         else
         {
            apparentSpatialInertiaToPack.unsafe_set(3, axis, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(4, axis, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(5, axis, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(3, axis + 3, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(4, axis + 3, 0.0);
            apparentSpatialInertiaToPack.unsafe_set(5, axis + 3, 0.0);
         }
      }

      return true;
   }

   /**
    * Computes the matrix representing the inverse of the linear part of the apparent inertia expressed
    * at {@code inertiaFrame} such that:
    *
    * <pre>
    * &Delta;&alpha;<sub>target</sub> = (I<sup>A</sup>)<sup>-1</sup> F<sub>target</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>(I<sup>A</sup>)<sup>-1</sup></tt> is the inverse of the 3-by-3 linear part of the
    * apparent inertia matrix.
    * <li><tt>F<sub>target</sub></tt> is a linear force applied at {@code inertiaFrame}.
    * <li><tt>&Delta;&alpha;<sub>target</sub></tt> is the resulting change in linear acceleration of
    * {@code target} at {@code inertiaFrame} due a force <tt>F<sub>target</sub></tt>.
    * </ul>
    * <p>
    * Note that the apparent inertia can also be used to relate an impulse to a change in twist as
    * follows:
    * </p>
    *
    * <pre>
    * &Delta;&nu;<sub>target</sub> = (I<sup>A</sup>)<sup>-1</sup> Y<sub>target</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>Y<sub>target</sub></tt> is a linear impulse applied at {@code inertiaFrame}.
    * <li><tt>&Delta;&nu;<sub>target</sub></tt> is the resulting change in linear velocity of
    * {@code target} at {@code inertiaFrame} due a linear impulse <tt>Y<sub>target</sub></tt>.
    * </ul>
    *
    * @param target                      the rigid-body to compute the apparent inertia at.
    * @param inertiaFrame                the frame at which the output is to be expressed.
    * @param apparentLinearInertiaToPack the matrix in which to store the result. Modified.
    * @return {@code true} is the apparent inertia matrix was successfully computed, {@code false}
    *         otherwise.
    */
   public boolean computeRigidBodyApparentLinearInertiaInverse(RigidBodyReadOnly target, ReferenceFrame inertiaFrame,
                                                               DMatrix1Row apparentLinearInertiaToPack)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      ReferenceFrame bodyFrame = target.getBodyFixedFrame();
      apparentLinearInertiaToPack.reshape(3, 3);

      reset();

      for (int axis = 0; axis < 3; axis++)
      {
         recursionStep.testDisturbancePlus.setIncludingFrame(bodyFrame,
                                                             inertiaFrame,
                                                             EuclidCoreTools.zeroVector3D,
                                                             Axis3D.values[axis],
                                                             EuclidCoreTools.origin3D);
         recursionStep.initializeDisturbance();
         recursionStep.updateRigidBodyMotionChange();
         recursionStep.rigidBodyMotionChange.changeFrame(inertiaFrame);
         recursionStep.rigidBodyMotionChange.getLinearPart().get(0, axis, apparentLinearInertiaToPack);
         reset();
      }

      return true;
   }

   /**
    * Computes the matrix that is the equivalent of the inverse of the apparent inertia but for the
    * joint, such that:
    *
    * <pre>
    * &Delta;qdd<sub>target</sub> = (I<sup>A</sup>)<sup>-1</sup> &tau;<sub>target</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>(I<sup>A</sup>)<sup>-1</sup></tt> is the N-by-N inverse of the pseudo inertia matrix,
    * with N being the number of DoFs of the joint.
    * <li><tt>&tau;<sub>target</sub></tt> is a joint wrench.
    * <li><tt>&Delta;qdd<sub>target</sub></tt> is the resulting change in joint acceleration.
    * </ul>
    * <p>
    * Note that the pseudo apparent inertia can also be used to relate a joint impulse to a change in
    * joint twist as follows:
    * </p>
    *
    * <pre>
    * &Delta;qd<sub>target</sub> = (I<sup>A</sup>)<sup>-1</sup> y<sub>target</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>&y;<sub>target</sub></tt> is a joint impulse.
    * <li><tt>&Delta;qd<sub>target</sub></tt> is the resulting change in joint twist.
    * </ul>
    *
    * @param target        the joint to compute the pseudo apparent inertia for.
    * @param inertiaToPack the matrix in which to store the result. Modified.
    * @return {@code true} is the matrix was successfully computed, {@code false} otherwise.
    */
   public boolean computeJointApparentInertiaInverse(JointReadOnly target, DMatrix1Row inertiaToPack)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target.getSuccessor());

      if (recursionStep == null)
         return false;

      inertiaToPack.reshape(target.getDegreesOfFreedom(), target.getDegreesOfFreedom());

      reset();

      for (int i = 0; i < target.getDegreesOfFreedom(); i++)
      {
         recursionStep.tauPlus.set(i, 0, 1.0);
         recursionStep.initializeDisturbance();
         recursionStep.updateRigidBodyMotionChange();
         CommonOps_DDRM.insert(recursionStep.qddPlus, inertiaToPack, 0, i);
         recursionStep.tauPlus.set(i, 0, 0.0);
         reset();
      }

      return true;
   }

   /**
    * Computes the value the inverse of the pseudo apparent inertia for the joint, such that:
    *
    * <pre>
    * &Delta;qdd<sub>target</sub> = (I<sup>A</sup>)<sup>-1</sup> &tau;<sub>target</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>(I<sup>A</sup>)<sup>-1</sup></tt> is the inverse of the pseudo inertia matrix.
    * <li><tt>&tau;<sub>target</sub></tt> is a joint effort.
    * <li><tt>&Delta;qdd<sub>target</sub></tt> is the resulting change in joint acceleration.
    * </ul>
    * <p>
    * Note that the pseudo apparent inertia can also be used to relate a joint impulse to a change in
    * joint velocity as follows:
    * </p>
    *
    * <pre>
    * &Delta;qd<sub>target</sub> = (I<sup>A</sup>)<sup>-1</sup> y<sub>target</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>&y;<sub>target</sub></tt> is a joint impulse.
    * <li><tt>&Delta;qd<sub>target</sub></tt> is the resulting change in joint velocity.
    * </ul>
    *
    * @param target the joint to compute the pseudo apparent inertia for.
    * @return {@code true} is the matrix was successfully computed, {@code false} otherwise.
    */
   public double computeJointApparentInertiaInverse(OneDoFJointReadOnly target)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target.getSuccessor());

      if (recursionStep == null)
         return Double.NaN;

      reset();
      recursionStep.tauPlus.set(0, 1.0);
      recursionStep.initializeDisturbance();
      recursionStep.updateRigidBodyMotionChange();
      reset();

      return recursionStep.qddPlus.get(0);
   }

   /**
    * Applies a 6-D wrench to {@code target} and compute the apparent wrench for each rigid-body
    * between {@code target} and the root-body of the system.
    * <p>
    * IMPORTANT: This calculator relies on the internal forward-dynamics calculator to be updated. It
    * is up to the user of this calculator to do so: {@code multiBodyCollision}
    * </p>
    * <p>
    * After applying a wrench, the following features are available:
    * <ul>
    * <li>{@link #getAccelerationChangeProvider()} can then be used to access the resulting change in
    * acceleration to any rigid-body in the system.
    * <li>{@link #propagateWrench()} can then be used to compute the change in acceleration caused by
    * the impulse on all rigid-bodies and get the change in joint velocity.
    * <li>{@link #getJointAccelerationChange(JointReadOnly)} or
    * {@link #getJointAccelerationChange(OneDoFJointReadOnly)} can then be used to compute the change
    * in acceleration for any joint.
    * </ul>
    * </p>
    *
    * @param target the rigid-body to which the wrench is applied to.
    * @param wrench the wrench to be applied. Not modified.
    * @return {@code true} if the wrench was successfully applied and the response computed,
    *         {@code false} otherwise.
    */
   public boolean applyRigidBodyWrench(RigidBodyReadOnly target, WrenchReadOnly wrench)
   {
      if (currentResponseType == ResponseType.TWIST)
         return false;

      if (!applyRigidBodyDisturbance(target, wrench))
         return false;

      currentResponseType = ResponseType.ACCELERATION;
      return true;
   }

   /**
    * Applies a 6-D impulse to {@code target} and compute the apparent impulse for each rigid-body
    * between {@code target} and the root-body of the system.
    * <p>
    * After applying an impulse, the following features are available:
    * <ul>
    * <li>{@link #getTwistChangeProvider()} can then be used to access the resulting change in twist to
    * any rigid-body in the system.
    * <li>{@link #propagateImpulse()} can then be used to compute the change in twist caused by the
    * impulse on all rigid-bodies and get the change in joint velocity.
    * <li>{@link #getJointTwistChange(JointReadOnly)} or
    * {@link #getJointTwistChange(OneDoFJointReadOnly)} can then be used to compute the change in twist
    * for any joint.
    * </ul>
    * </p>
    *
    * @param target  the rigid-body to which the impulse is applied to.
    * @param impulse the impulse to be applied. Not modified.
    * @return {@code true} if the impulse was successfully applied, {@code false} otherwise.
    */
   public boolean applyRigidBodyImpulse(RigidBodyReadOnly target, SpatialImpulseReadOnly impulse)
   {
      if (currentResponseType == ResponseType.ACCELERATION)
         return false;

      if (!applyRigidBodyDisturbance(target, impulse))
         return false;

      currentResponseType = ResponseType.TWIST;
      return true;
   }

   private boolean applyRigidBodyDisturbance(RigidBodyReadOnly target, SpatialForceReadOnly disturbance)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      ReferenceFrame bodyFrame = target.getBodyFixedFrame();
      recursionStep.testDisturbancePlus.setIncludingFrame(bodyFrame, disturbance);
      recursionStep.initializeDisturbance();
      return true;
   }

   /**
    * Applies a 1-D effort to the joint {@code target} and compute the apparent wrench for each
    * rigid-body between {@code target} and the root-body of the system.
    * <p>
    * After applying an effort, the following features are available:
    * <ul>
    * <li>{@link #getAccelerationChangeProvider()} can then be used to access the resulting change in
    * acceleration to any rigid-body in the system.
    * <li>{@link #propagateWrench()} can then be used to compute the change in acceleration caused by
    * the impulse on all rigid-bodies and get the change in joint velocity.
    * <li>{@link #getJointAccelerationChange(JointReadOnly)} or
    * {@link #getJointAccelerationChange(OneDoFJointReadOnly)} can then be used to compute the change
    * in acceleration for any joint.
    * </ul>
    * </p>
    *
    * @param target the joint at which the effort is applied.
    * @param effort the effort to be applied. Not modified.
    * @return {@code true} if the effort was successfully applied, {@code false} otherwise.
    */
   public boolean applyJointWrench(OneDoFJointReadOnly target, double effort)
   {
      if (currentResponseType == ResponseType.TWIST)
         return false;

      if (!applyJointDisturbance(target, effort))
         return false;

      currentResponseType = ResponseType.ACCELERATION;
      return true;
   }

   /**
    * Applies a N-dimensional wrench to the joint {@code target} and compute the apparent wrench for
    * each rigid-body between {@code target} and the root-body of the system, where N is the number of
    * degrees of freedom of {@code target}.
    * <p>
    * After applying an wrench, the following features are available:
    * <ul>
    * <li>{@link #getAccelerationChangeProvider()} can then be used to access the resulting change in
    * acceleration to any rigid-body in the system.
    * <li>{@link #propagateWrench()} can then be used to compute the change in acceleration caused by
    * the impulse on all rigid-bodies and get the change in joint velocity.
    * <li>{@link #getJointAccelerationChange(JointReadOnly)} or
    * {@link #getJointAccelerationChange(OneDoFJointReadOnly)} can then be used to compute the change
    * in acceleration for any joint.
    * </ul>
    * </p>
    *
    * @param target the joint at which the wrench is applied.
    * @param wrench the wrench to be applied. Not modified.
    * @return {@code true} if the wrench was successfully applied, {@code false} otherwise.
    */
   public boolean applyJointWrench(JointReadOnly target, DMatrix wrench)
   {
      if (currentResponseType == ResponseType.TWIST)
         return false;

      if (!applyJointDisturbance(target, wrench))
         return false;

      currentResponseType = ResponseType.ACCELERATION;
      return true;
   }

   /**
    * Applies a 1-D impulse to the joint {@code target} and compute the apparent impulse for each
    * rigid-body between {@code target} and the root-body of the system.
    * <p>
    * After applying an impulse, the following features are available:
    * <ul>
    * <li>{@link #getTwistChangeProvider()} can then be used to access the resulting change in twist to
    * any rigid-body in the system.
    * <li>{@link #propagateImpulse()} can then be used to compute the change in twist caused by the
    * impulse on all rigid-bodies and get the change in joint velocity.
    * <li>{@link #getJointTwistChange(JointReadOnly)} or
    * {@link #getJointTwistChange(OneDoFJointReadOnly)} can then be used to compute the change in twist
    * for any joint.
    * </ul>
    * </p>
    *
    * @param target  the joint at which the impulse is applied.
    * @param impulse the impulse to be applied. Not modified.
    * @return {@code true} if the impulse was successfully applied, {@code false} otherwise.
    */
   public boolean applyJointImpulse(OneDoFJointReadOnly target, double impulse)
   {
      if (currentResponseType == ResponseType.ACCELERATION)
         return false;

      if (!applyJointDisturbance(target, impulse))
         return false;

      currentResponseType = ResponseType.TWIST;
      return true;
   }

   /**
    * Applies a N-dimensional impulse to the joint {@code target} and compute the apparent impulse for
    * each rigid-body between {@code target} and the root-body of the system, where N is the number of
    * degrees of freedom of {@code target}.
    * <p>
    * After applying an impulse, the following features are available:
    * <ul>
    * <li>{@link #getTwistChangeProvider()} can then be used to access the resulting change in twist to
    * any rigid-body in the system.
    * <li>{@link #propagateImpulse()} can then be used to compute the change in twist caused by the
    * impulse on all rigid-bodies and get the change in joint velocity.
    * <li>{@link #getJointTwistChange(JointReadOnly)} or
    * {@link #getJointTwistChange(OneDoFJointReadOnly)} can then be used to compute the change in twist
    * for any joint.
    * </ul>
    * </p>
    *
    * @param target  the joint at which the impulse is applied.
    * @param impulse the impulse to be applied. Not modified.
    * @return {@code true} if the impulse was successfully applied, {@code false} otherwise.
    */
   public boolean applyJointImpulse(JointReadOnly target, DMatrix impulse)
   {
      if (currentResponseType == ResponseType.ACCELERATION)
         return false;

      if (!applyJointDisturbance(target, impulse))
         return false;

      currentResponseType = ResponseType.TWIST;
      return true;
   }

   private boolean applyJointDisturbance(OneDoFJointReadOnly target, double disturbance)
   {
      singleElementMatrix.set(0, disturbance);
      return applyJointDisturbance(target, singleElementMatrix);
   }

   private boolean applyJointDisturbance(JointReadOnly target, DMatrix disturbance)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target.getSuccessor());

      if (recursionStep == null)
         return false;
      if (disturbance.getNumRows() != recursionStep.getJoint().getDegreesOfFreedom() || disturbance.getNumCols() != 1)
         throw new IllegalArgumentException("Matrix dimension mismatch: expected " + recursionStep.getJoint().getDegreesOfFreedom() + "-by-1, was "
               + disturbance.getNumRows() + "-by-" + disturbance.getNumCols());

      recursionStep.tauPlus.set(disturbance);
      recursionStep.initializeDisturbance();
      return true;
   }

   /**
    * Propagates a <b>previously</b> applied wrench to all rigid-bodies in the system and computes the
    * resulting change in acceleration.
    *
    * @return the matrix with the resulting change in joint acceleration, or {@code null} if
    *         {@link #applyRigidBodyWrench(RigidBodyReadOnly, WrenchReadOnly)} was not called last.
    */
   public DMatrixRMaj propagateWrench()
   {
      return currentResponseType == ResponseType.ACCELERATION ? propagateDisturbance() : null;
   }

   /**
    * Propagates a <b>previously</b> applied impulse to all rigid-bodies in the system and computes the
    * resulting change in twist.
    *
    * @return the matrix with the resulting change in joint velocity, or {@code null} if
    *         {@link #applyRigidBodyImpulse(RigidBodyReadOnly, SpatialImpulseReadOnly)} was not called
    *         last.
    */
   public DMatrixRMaj propagateImpulse()
   {
      return currentResponseType == ResponseType.TWIST ? propagateDisturbance() : null;
   }

   private DMatrixRMaj propagateDisturbance()
   {
      if (!initialRecursionStep.isUpToDate)
         return null;

      initialRecursionStep.propagateDownDisturbance();
      return jointMotionChangeMatrix;
   }

   /**
    * Gets the rigid-body acceleration provider that can be used to access change in acceleration of
    * any rigid-body in the system due to the test wrench.
    * <p>
    * This provider requires that a wrench was applied before performing queries.
    * </p>
    *
    * @return the acceleration change provider.
    */
   public RigidBodyAccelerationProvider getAccelerationChangeProvider()
   {
      return accelerationChangeProvider;
   }

   /**
    * Gets the rigid-body twist provider that can be used to access change in twist of any rigid-body
    * in the system due to the test impulse.
    * <p>
    * This provider requires that an impulse was applied before performing queries.
    * </p>
    *
    * @return the twist change provider.
    */
   public RigidBodyTwistProvider getTwistChangeProvider()
   {
      return twistChangeProvider;
   }

   /**
    * Gets the change in joint acceleration due to the test wrench.
    * <p>
    * This method requires that a wrench was applied.
    * </p>
    *
    * @param joint the joint to get the change in acceleration for.
    * @return the change in acceleration of the {@code joint}.
    */
   public double getJointAccelerationChange(OneDoFJointReadOnly joint)
   {
      return currentResponseType != ResponseType.ACCELERATION ? Double.NaN : getJointMotionChange(joint);
   }

   /**
    * Gets the change in joint acceleration due to the test wrench.
    * <p>
    * This method requires that a wrench was applied.
    * </p>
    *
    * @param joint the joint to get the change in acceleration for.
    * @return the N-by-1 matrix containing the change in acceleration of the {@code joint}.
    */
   public DMatrixRMaj getJointAccelerationChange(JointReadOnly joint)
   {
      return currentResponseType != ResponseType.ACCELERATION ? null : getJointMotionChange(joint);
   }

   /**
    * Gets the change in joint velocity due to the test impulse.
    * <p>
    * This method requires that an impulse was applied.
    * </p>
    *
    * @param joint the joint to get the change in velocity for.
    * @return the change in velocity of the {@code joint}.
    */
   public double getJointTwistChange(OneDoFJointReadOnly joint)
   {
      return currentResponseType != ResponseType.TWIST ? Double.NaN : getJointMotionChange(joint);
   }

   /**
    * Gets the change in joint twist due to the test impulse.
    * <p>
    * This method requires that an impulse was applied.
    * </p>
    *
    * @param joint the joint to get the change in twist for.
    * @return the N-by-1 matrix containing the change in twist of the {@code joint}.
    */
   public DMatrixRMaj getJointTwistChange(JointReadOnly joint)
   {
      return currentResponseType != ResponseType.TWIST ? null : getJointMotionChange(joint);
   }

   private double getJointMotionChange(OneDoFJointReadOnly joint)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());
      if (recursionStep == null)
         return Double.NaN;

      if (currentResponseType == null || !initialRecursionStep.isUpToDate)
      { // This calculator has not been initialized with a disturbance yet.
         return Double.NaN;
      }

      recursionStep.updateRigidBodyMotionChange();
      return recursionStep.qddPlus.get(0);
   }

   private DMatrixRMaj getJointMotionChange(JointReadOnly joint)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());
      if (recursionStep == null)
         return null;

      if (currentResponseType == null || !initialRecursionStep.isUpToDate)
      { // This calculator has not been initialized with a disturbance yet.
         return null;
      }

      recursionStep.updateRigidBodyMotionChange();
      return recursionStep.qddPlus;
   }

   class ResponseRecursionStep
   {
      /**
       * Test wrench containing external disturbance applied to this body to test the resulting change
       * motion with.
       * <p>
       * This can either represent a wrench or impulse. The equations are the same either way.
       * </p>
       */
      final Wrench testDisturbancePlus;
      /**
       * Wrench containing all external disturbance applied to this body to test the resulting change
       * motion with.
       * <p>
       * This can either represent a wrench or impulse. The equations are the same either way.
       * </p>
       */
      final Wrench totalDisturbancePlus;
      /**
       * Pre-transformed test disturbance for the parent.
       * <p>
       * This can either represent a wrench or impulse. The equations are the same either way.
       * </p>
       */
      final SpatialForce totalDisturbancePlusForParent;
      /**
       * Change of motion of this rigid-body.
       * <p>
       * This can either represent the change in acceleration or twist. The equations are the same either
       * way.
       * </p>
       */
      final SpatialAcceleration rigidBodyMotionChange = new SpatialAcceleration();
      /**
       * Change of motion of this rigid-body's direct predecessor.
       * <p>
       * This can either represent the change in acceleration or twist. The equations are the same either
       * way.
       * </p>
       */
      final SpatialAcceleration parentMotionChange;
      /**
       * Test effort containing the additional disturbance at the joint to test the resulting change in
       * motion with.
       * <p>
       * The matrix is a N-by-1 vector representing either an extra effort or impulse, where N is equal to
       * the number of DoFs that the joint has.
       * </p>
       */
      final DMatrixRMaj tauPlus;
      /**
       * This is the apparent force for this joint resulting from {@code testWrenchPlus}:
       *
       * <pre>
       * p<sup>A+</sup> = p<sup>+</sup> + </sub> p<sup>a+</sup><sub>child</sub>
       * </pre>
       */
      final DMatrixRMaj pAPlus;
      /**
       * Intermediate result to save computation:
       *
       * <pre>
       * u<sup>+</sup> = - S<sup>T</sup> p<sup>A+</sup>
       * </pre>
       *
       * where <tt>&tau;</tt> is the N-by-1 vector representing the joint effort, N being the number of
       * DoFs for this joint, <tt>S</tt> is the joint motion subspace, and <tt>p<sup>A+</sup></tt> the
       * apparent force resulting from {@code testWrenchPlus}.
       */
      final DMatrixRMaj uPlus;
      /**
       * The apparent bias forces/impulses of this joint for the parent resulting from the
       * {@code testWrenchPlus}:
       *
       * <pre>
       * p<sup>a+</sup> = p<sup>A+</sup> + U D<sup>-1</sup> u<sup>+</sup>
       *  <sup>  </sup> = p<sup>A+</sup> - I<sup>A</sup> S ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup> S<sup>T</sup> p<sup>A+</sup>
       * </pre>
       *
       * where <tt>p<sup>A+</sup></tt> is {@code testDisturbancePlus} acting on this joint, <tt>S</tt> is
       * the joint motion subspace, <tt>I<sup>A</sup></tt> this handle's articulated-body inertia,
       * <tt>&tau;</tt> this joint effort.
       */
      final DMatrixRMaj paPlus;
      /**
       * The change in parent body motion.
       * <p>
       * This can either represent the change in acceleration or twist. The equations are the same either
       * way.
       * </p>
       */
      final DMatrixRMaj aParentPlus;
      /**
       * <b>Output of this algorithm: the change in body motion:</b>
       *
       * <pre>
       * a<sup>+</sup> = a<sup>+</sup><sub>parent</sub> + S qDDot<sup>+</sup>
       * </pre>
       * <p>
       * This can either represent the change in acceleration or twist. The equations are the same either
       * way.
       * </p>
       */
      final DMatrixRMaj aPlus;
      /**
       * Intermediate result for garbage-free operation.
       */
      final DMatrixRMaj qddPlus_intermediate;
      /**
       * <b>Output of this algorithm: the change in joint motion:</b>
       *
       * <pre>
       * qDDot<sup>+</sup> = D<sup>-1</sup> ( u<sup>+</sup> - U<sup>T</sup> a<sup>+</sup><sub>parent</sub> )
       * </pre>
       * <p>
       * This can either represent the change in acceleration or twist. The equations are the same either
       * way.
       * </p>
       */
      final DMatrixRMaj qddPlus;
      /**
       * The forward-dynamics recursion step for the rigid-body of this recursion step.
       */
      private ArticulatedBodyRecursionStep articulatedBodyRecursionStep;
      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid-body.
       */
      private ResponseRecursionStep parent;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      final List<ResponseRecursionStep> children = new ArrayList<>();

      /**
       * Whether the rigid-body is located on the branch that starts a the root and ends at the target
       * body that is subject to an external disturbance.
       */
      private boolean isOnDisturbedBranch = false;
      /**
       * Flag used for lazy update.
       * <p>
       * When {@code true}, this means that the change of motion of this step is up-to-date. When
       * {@code false}, either the calculator has been reset, or a disturbance has been applied but not
       * propagated down the tree.
       * </p>
       */
      private boolean isUpToDate = false;

      public ResponseRecursionStep(ArticulatedBodyRecursionStep articulatedBodyRecursionStep, ResponseRecursionStep parent)
      {
         this.articulatedBodyRecursionStep = articulatedBodyRecursionStep;
         this.parent = parent;

         if (parent == null)
         {
            testDisturbancePlus = null;
            totalDisturbancePlus = null;
            totalDisturbancePlusForParent = null;
            parentMotionChange = null;
            rigidBodyMotionChange.setToZero(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());

            tauPlus = null;
            pAPlus = null;
            uPlus = null;
            paPlus = null;
            qddPlus_intermediate = null;
            qddPlus = null;
            aParentPlus = null;
            aPlus = null;
         }
         else
         {
            parent.children.add(this);
            int nDoFs = getJoint().getDegreesOfFreedom();

            testDisturbancePlus = new Wrench(getBodyFixedFrame(), getBodyFixedFrame());
            totalDisturbancePlus = new Wrench(getBodyFixedFrame(), getFrameAfterJoint());
            totalDisturbancePlusForParent = parent.isRoot() ? null : new SpatialForce();
            parentMotionChange = new SpatialAcceleration();

            tauPlus = new DMatrixRMaj(nDoFs, 1);
            pAPlus = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            uPlus = new DMatrixRMaj(nDoFs, 1);
            paPlus = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            qddPlus_intermediate = new DMatrixRMaj(nDoFs, 1);
            qddPlus = new DMatrixRMaj(nDoFs, 1);
            aParentPlus = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
            aPlus = new DMatrixRMaj(SpatialVectorReadOnly.SIZE, 1);
         }
      }

      public void reset()
      {
         if (!isOnDisturbedBranch && !isUpToDate)
            return; // Early termination for this branch.

         isOnDisturbedBranch = false;
         isUpToDate = false;

         if (!isRoot())
         {
            testDisturbancePlus.setToZero();
            tauPlus.zero();
         }

         for (ResponseRecursionStep child : children)
            child.reset();
      }

      public void markDirty()
      {
         if (!isUpToDate)
            return;

         isUpToDate = false;

         for (ResponseRecursionStep child : children)
            child.markDirty();
      }

      /**
       * Propagates the test disturbance from the solicited body through up the system until reaching the
       * root.
       * <p>
       * The disturbance can either be a wrench or impulse.
       * </p>
       */
      public void initializeDisturbance()
      {
         // Going bottom-up in the tree.
         if (isRoot())
         {
            isUpToDate = true;
            isOnDisturbedBranch = true;
            return;
         }

         stepUpDisturbance();
         isOnDisturbedBranch = true;
         parent.initializeDisturbance();
      }

      public void stepUpDisturbance()
      {
         isUpToDate = false;

         DMatrixRMaj S = articulatedBodyRecursionStep.S;

         totalDisturbancePlus.setToZero();

         for (int i = 0; i < children.size(); i++)
         {
            ResponseRecursionStep child = children.get(i);

            if (child.isOnDisturbedBranch)
               totalDisturbancePlus.add(child.totalDisturbancePlusForParent);

            child.markDirty();
         }

         testDisturbancePlus.changeFrame(getFrameAfterJoint());
         totalDisturbancePlus.sub(testDisturbancePlus);

         totalDisturbancePlus.get(pAPlus);
         CommonOps_DDRM.multTransA(-1.0, S, pAPlus, uPlus);
         CommonOps_DDRM.addEquals(uPlus, tauPlus);

         if (!parent.isRoot())
         {
            DMatrixRMaj U_Dinv = articulatedBodyRecursionStep.U_Dinv;
            CommonOps_DDRM.mult(U_Dinv, uPlus, paPlus);
            CommonOps_DDRM.addEquals(paPlus, pAPlus);
            totalDisturbancePlusForParent.setIncludingFrame(totalDisturbancePlus.getReferenceFrame(), paPlus);
            totalDisturbancePlusForParent.applyTransform(articulatedBodyRecursionStep.transformToParentJointFrame);
            totalDisturbancePlusForParent.setReferenceFrame(parent.getFrameAfterJoint());
         }
      }

      /**
       * Propagates the change in acceleration from the root down to the leaves.
       */
      public void propagateDownDisturbance()
      {
         stepDownDisturbance();

         if (!isRoot())
         {
            int[] jointIndices = articulatedBodyRecursionStep.jointIndices;

            for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
            {
               jointMotionChangeMatrix.set(jointIndices[dofIndex], 0, qddPlus.get(dofIndex, 0));
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).propagateDownDisturbance();
      }

      public void updateRigidBodyMotionChange()
      {
         if (isUpToDate || isRoot())
            return;

         parent.updateRigidBodyMotionChange();
         stepDownDisturbance();
      }

      private void stepDownDisturbance()
      {
         if (isUpToDate)
            return;

         if (!isRoot())
         {
            DMatrixRMaj S = articulatedBodyRecursionStep.S;
            DMatrixRMaj U = articulatedBodyRecursionStep.U;
            DMatrixRMaj Dinv = articulatedBodyRecursionStep.Dinv;

            // Going top-down in the tree.
            parentMotionChange.setIncludingFrame(parent.rigidBodyMotionChange);
            parentMotionChange.applyInverseTransform(articulatedBodyRecursionStep.transformToParentJointFrame);
            parentMotionChange.setReferenceFrame(getFrameAfterJoint());
            parentMotionChange.get(aParentPlus);

            if (isOnDisturbedBranch)
            {
               // Computing qdd = D^-1 * ( u - U^T * a_parent )
               CommonOps_DDRM.multTransA(-1.0, U, aParentPlus, qddPlus_intermediate);
               CommonOps_DDRM.addEquals(qddPlus_intermediate, uPlus);
               CommonOps_DDRM.mult(Dinv, qddPlus_intermediate, qddPlus);
            }
            else
            {
               // Computing qdd = -D^-1 * U^T * a_parent )
               CommonOps_DDRM.multTransA(-1.0, U, aParentPlus, qddPlus_intermediate);
               CommonOps_DDRM.mult(Dinv, qddPlus_intermediate, qddPlus);
            }

            CommonOps_DDRM.mult(S, qddPlus, aPlus);
            CommonOps_DDRM.addEquals(aPlus, aParentPlus);
            rigidBodyMotionChange.setIncludingFrame(getBodyFixedFrame(), input.getInertialFrame(), getFrameAfterJoint(), aPlus);
         }

         isUpToDate = true;
      }

      public boolean isRoot()
      {
         return parent == null;
      }

      public MovingReferenceFrame getBodyFixedFrame()
      {
         return articulatedBodyRecursionStep.getBodyFixedFrame();
      }

      public MovingReferenceFrame getFrameAfterJoint()
      {
         return getJoint().getFrameAfterJoint();
      }

      public JointReadOnly getJoint()
      {
         return articulatedBodyRecursionStep.getJoint();
      }

      @Override
      public String toString()
      {
         return "RigidBody: " + articulatedBodyRecursionStep.rigidBody + ", parent: " + parent.articulatedBodyRecursionStep.rigidBody;
      }
   }
}
