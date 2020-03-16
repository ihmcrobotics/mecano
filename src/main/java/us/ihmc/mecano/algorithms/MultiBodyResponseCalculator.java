package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.Axis;
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
import us.ihmc.mecano.spatial.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Inspired from Mirtich's thesis, this calculator allows to evaluate the perturbation in terms of
 * change in acceleration (or twist) due to a wrench (or impulse) applied on a rigid-body.
 * <p>
 * This can be used to compute an adequate wrench (or impulse) to apply on a rigid-body to obtain a
 * given resulting acceleration (or twist).
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
   private final DenseMatrix64F jointMotionChangeMatrix;
   /**
    * This algorithm relies on the pre-computed internal data from a forward dynamics algorithm.
    */
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;

   /**
    * Extension of this algorithm into an acceleration provider that can be used to retrieve change in
    * acceleration to any rigid-body of the system.
    */
   private final RigidBodyAccelerationProvider accelerationChangeProvider;
   /**
    * Extension of this algorithm into an twist provider that can be used to retrieve change in twist
    * to any rigid-body of the system.
    */
   private final RigidBodyTwistProvider twistChangeProvider;

   private enum DisturbanceSource
   {
      RIGID_BODY, JOINT;
   }

   private enum ResponseType
   {
      ACCELERATION, TWIST
   };

   private ResponseType currentResponseType = null;
   private final DenseMatrix64F singleElementMatrix = new DenseMatrix64F(1, 1);

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
      jointMotionChangeMatrix = new DenseMatrix64F(nDoFs, 1);

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
   public boolean computeRigidBodyApparentSpatialInertiaInverse(RigidBodyReadOnly target, ReferenceFrame inertiaFrame, DenseMatrix64F apparentSpatialInertiaToPack)
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
                                                                DenseMatrix64F apparentSpatialInertiaToPack)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      ReferenceFrame bodyFrame = target.getBodyFixedFrame();

      apparentSpatialInertiaToPack.reshape(6, 6);

      for (int axis = 0; axis < 3; axis++)
      {
         if (selectedAxes == null || selectedAxes[axis])
         {
            recursionStep.testDisturbancePlus.setIncludingFrame(bodyFrame, inertiaFrame, Axis.values[axis], EuclidCoreTools.zeroVector3D);
            recursionStep.initializeDisturbance(null, DisturbanceSource.RIGID_BODY);
            recursionStep.updateRigidBodyMotionChange();
            recursionStep.rigidBodyMotionChange.changeFrame(inertiaFrame);
            recursionStep.rigidBodyMotionChange.get(0, axis, apparentSpatialInertiaToPack);
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
            recursionStep.testDisturbancePlus.setIncludingFrame(bodyFrame, inertiaFrame, EuclidCoreTools.zeroVector3D, Axis.values[axis]);
            recursionStep.initializeDisturbance(null, DisturbanceSource.RIGID_BODY);
            recursionStep.updateRigidBodyMotionChange();
            recursionStep.rigidBodyMotionChange.changeFrame(inertiaFrame);
            recursionStep.rigidBodyMotionChange.get(0, axis + 3, apparentSpatialInertiaToPack);
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
      reset();

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
    * 
    * @param target                      the rigid-body to compute the apparent inertia at.
    * @param inertiaFrame                the frame at which the output is to be expressed.
    * @param apparentLinearInertiaToPack the matrix in which to store the result. Modified.
    * @return {@code true} is the apparent inertia matrix was successfully computed, {@code false}
    *         otherwise.
    */
   public boolean computeRigidBodyApparentLinearInertiaInverse(RigidBodyReadOnly target, ReferenceFrame inertiaFrame,
                                                               DenseMatrix64F apparentLinearInertiaToPack)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      ReferenceFrame bodyFrame = target.getBodyFixedFrame();
      apparentLinearInertiaToPack.reshape(3, 3);

      for (int axis = 0; axis < 3; axis++)
      {
         recursionStep.testDisturbancePlus.setIncludingFrame(bodyFrame,
                                                             inertiaFrame,
                                                             EuclidCoreTools.zeroVector3D,
                                                             Axis.values[axis],
                                                             EuclidCoreTools.origin3D);
         recursionStep.initializeDisturbance(null, DisturbanceSource.RIGID_BODY);
         recursionStep.updateRigidBodyMotionChange();
         recursionStep.rigidBodyMotionChange.changeFrame(inertiaFrame);
         recursionStep.rigidBodyMotionChange.getLinearPart().get(0, axis, apparentLinearInertiaToPack);
      }
      reset();

      return true;
   }

   public boolean computeJointApparentInertiaInverse(JointReadOnly target, DenseMatrix64F inertiaToPack)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target.getSuccessor());

      if (recursionStep == null)
         return false;

      inertiaToPack.reshape(target.getDegreesOfFreedom(), target.getDegreesOfFreedom());

      for (int i = 0; i < target.getDegreesOfFreedom(); i++)
      {
         recursionStep.tauPlus.set(i, 0, 1.0);
         recursionStep.initializeDisturbance(null, DisturbanceSource.JOINT);
         recursionStep.updateRigidBodyMotionChange();
         CommonOps.insert(recursionStep.qddPlus, inertiaToPack, 0, i);
         recursionStep.tauPlus.set(i, 0, 0.0);
      }
      reset();

      return true;
   }

   public double computeJointApparentInertiaInverse(OneDoFJointReadOnly target)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target.getSuccessor());

      if (recursionStep == null)
         return Double.NaN;

      recursionStep.tauPlus.set(0, 1.0);
      recursionStep.initializeDisturbance(null, DisturbanceSource.JOINT);
      recursionStep.updateRigidBodyMotionChange();
      reset();

      return recursionStep.qddPlus.get(0);
   }

   /**
    * Applies a 6-D wrench to {@code target} and propagates the effect to the rest of the multi-body
    * system.
    * 
    * @param target the rigid-body to which the wrench is applied to.
    * @param wrench the wrench to be applied. Not modified.
    * @return the response to the wrench on the multi-body system in terms of change of joint
    *         accelerations, or {@code null} if this methods failed.
    * @see #applyRigidBodyWrench(RigidBodyReadOnly, WrenchReadOnly)
    * @see #propagateWrench()
    */
   public DenseMatrix64F applyAndPropagateRigidBodyWrench(RigidBodyReadOnly target, WrenchReadOnly wrench)
   {
      if (!applyRigidBodyWrench(target, wrench))
         return null;

      return propagateWrench();
   }

   /**
    * Applies a 6-D impulse to {@code target} and propagates the effect to the rest of the multi-body
    * system.
    * 
    * @param target  the rigid-body to which the impulse is applied to.
    * @param impulse the impulse to be applied. Not modified.
    * @return the response to the impulse on the multi-body system in terms of change of joint
    *         velocities, or {@code null} if this methods failed.
    * @see #applyRigidBodyImpulse(RigidBodyReadOnly, SpatialImpulseReadOnly)
    * @see #propagateImpulse()
    */
   public DenseMatrix64F applyAndPropagateRigidBodyImpulse(RigidBodyReadOnly target, SpatialImpulseReadOnly impulse)
   {
      if (!applyRigidBodyImpulse(target, impulse))
         return null;

      return propagateImpulse();
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
    * the wrench on all rigid-bodies and get the change in joint acceleration.
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
    * </ul>
    * </p>
    * 
    * @param target  the rigid-body to which the impulse is applied to.
    * @param impulse the impulse to be applied. Not modified.
    * @return {@code true} if the impulse was successfully applied, {@code false} otherwise.
    */
   public boolean applyRigidBodyImpulse(RigidBodyReadOnly target, SpatialImpulseReadOnly impulse)
   {
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

      reset();
      ReferenceFrame bodyFrame = target.getBodyFixedFrame();
      recursionStep.testDisturbancePlus.setIncludingFrame(bodyFrame, disturbance);
      recursionStep.initializeDisturbance(null, DisturbanceSource.RIGID_BODY);
      return true;
   }

   public DenseMatrix64F applyAndPropagateJointWrench(OneDoFJointReadOnly target, double effort)
   {
      if (applyJointWrench(target, effort))
         return propagateWrench();
      else
         return null;
   }

   public DenseMatrix64F applyAndPropagateJointWrench(JointReadOnly target, DenseMatrix64F wrench)
   {
      if (applyJointWrench(target, wrench))
         return propagateWrench();
      else
         return null;
   }

   public DenseMatrix64F applyAndPropagateJointImpulse(OneDoFJointReadOnly target, double impulse)
   {
      if (applyJointImpulse(target, impulse))
         return propagateImpulse();
      else
         return null;
   }

   public DenseMatrix64F applyAndPropagateJointImpulse(JointReadOnly target, DenseMatrix64F impulse)
   {
      if (applyJointImpulse(target, impulse))
         return propagateImpulse();
      else
         return null;
   }

   public boolean applyJointWrench(OneDoFJointReadOnly target, double effort)
   {
      if (!applyJointDisturbance(target, effort))
         return false;

      currentResponseType = ResponseType.ACCELERATION;
      return true;
   }

   public boolean applyJointWrench(JointReadOnly target, DenseMatrix64F wrench)
   {
      if (!applyJointDisturbance(target, wrench))
         return false;

      currentResponseType = ResponseType.ACCELERATION;
      return true;
   }

   public boolean applyJointImpulse(OneDoFJointReadOnly target, double impulse)
   {
      if (!applyJointDisturbance(target, impulse))
         return false;

      currentResponseType = ResponseType.TWIST;
      return true;
   }

   public boolean applyJointImpulse(JointReadOnly target, DenseMatrix64F impulse)
   {
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

   private boolean applyJointDisturbance(JointReadOnly target, DenseMatrix64F disturbance)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target.getSuccessor());

      if (recursionStep == null)
         return false;
      if (disturbance.getNumRows() != recursionStep.getJoint().getDegreesOfFreedom() || disturbance.getNumCols() != 1)
         throw new IllegalArgumentException("Matrix dimension mismatch: expected " + recursionStep.getJoint().getDegreesOfFreedom() + "-by-1, was "
               + disturbance.getNumRows() + "-by-" + disturbance.getNumCols());

      reset();
      recursionStep.tauPlus.set(disturbance);
      recursionStep.initializeDisturbance(null, DisturbanceSource.JOINT);
      return true;
   }

   /**
    * Propagates a <b>previously</b> applied wrench to all rigid-bodies in the system and computes the
    * resulting change in acceleration.
    * 
    * @return the matrix with the resulting change in joint acceleration, or {@code null} if
    *         {@link #applyRigidBodyWrench(RigidBodyReadOnly, WrenchReadOnly)} was not called last.
    */
   public DenseMatrix64F propagateWrench()
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
   public DenseMatrix64F propagateImpulse()
   {
      return currentResponseType == ResponseType.TWIST ? propagateDisturbance() : null;
   }

   private DenseMatrix64F propagateDisturbance()
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
    * The provider is initialized only after calling either
    * {@link #applyRigidBodyWrench(RigidBodyReadOnly, WrenchReadOnly)} or
    * {@link #applyAndPropagateRigidBodyWrench(RigidBodyReadOnly, WrenchReadOnly)}.
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
    * The provider is initialized only after calling either
    * {@link #applyRigidBodyWrench(RigidBodyReadOnly, WrenchReadOnly)} or
    * {@link #applyAndPropagateRigidBodyWrench(RigidBodyReadOnly, WrenchReadOnly)}.
    * </p>
    * 
    * @return the twist change provider.
    */
   public RigidBodyTwistProvider getTwistChangeProvider()
   {
      return twistChangeProvider;
   }

   public double getJointAccelerationChange(OneDoFJointReadOnly joint)
   {
      return currentResponseType != ResponseType.ACCELERATION ? Double.NaN : getJointMotionChange(joint);
   }

   public DenseMatrix64F getJointAccelerationChange(JointReadOnly joint)
   {
      return currentResponseType != ResponseType.ACCELERATION ? null : getJointMotionChange(joint);
   }

   public double getJointTwistChange(OneDoFJointReadOnly joint)
   {
      return currentResponseType != ResponseType.TWIST ? Double.NaN : getJointMotionChange(joint);
   }

   public DenseMatrix64F getJointTwistChange(JointReadOnly joint)
   {
      return currentResponseType != ResponseType.TWIST ? null : getJointMotionChange(joint);
   }

   private double getJointMotionChange(OneDoFJointReadOnly joint)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());
      if (recursionStep == null)
         return Double.NaN;

      recursionStep.updateRigidBodyMotionChange();
      return recursionStep.qddPlus.get(0);
   }

   private DenseMatrix64F getJointMotionChange(JointReadOnly joint)
   {
      ResponseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());
      if (recursionStep == null)
         return null;

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
       * Pre-transformed test disturbance for the parent.
       * <p>
       * This can either represent a wrench or impulse. The equations are the same either way.
       * </p>
       */
      final SpatialForce testDisturbancePlusForParent;
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
      final DenseMatrix64F tauPlus;
      /**
       * This is the apparent force for this joint resulting from {@code testWrenchPlus}:
       * 
       * <pre>
       * p<sup>A+</sup> = p<sup>+</sup> + </sub> p<sup>a+</sup><sub>child</sub>
       * </pre>
       */
      final DenseMatrix64F pAPlus;
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
      final DenseMatrix64F uPlus;
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
      final DenseMatrix64F paPlus;
      /**
       * The change in parent body motion.
       * <p>
       * This can either represent the change in acceleration or twist. The equations are the same either
       * way.
       * </p>
       */
      final DenseMatrix64F aParentPlus;
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
      final DenseMatrix64F aPlus;
      /**
       * Intermediate result for garbage-free operation.
       */
      final DenseMatrix64F qddPlus_intermediate;
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
      final DenseMatrix64F qddPlus;
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
            testDisturbancePlusForParent = null;
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

            testDisturbancePlus = new Wrench();
            testDisturbancePlusForParent = parent.isRoot() ? null : new SpatialForce();
            parentMotionChange = new SpatialAcceleration();

            tauPlus = new DenseMatrix64F(nDoFs, 1);
            pAPlus = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
            uPlus = new DenseMatrix64F(nDoFs, 1);
            paPlus = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
            qddPlus_intermediate = new DenseMatrix64F(nDoFs, 1);
            qddPlus = new DenseMatrix64F(nDoFs, 1);
            aParentPlus = new DenseMatrix64F(SpatialAccelerationReadOnly.SIZE, 1);
            aPlus = new DenseMatrix64F(SpatialAccelerationReadOnly.SIZE, 1);
         }
      }

      public void reset()
      {
         isOnDisturbedBranch = false;
         isUpToDate = false;
         for (ResponseRecursionStep child : children)
            child.reset();
      }

      /**
       * Propagates the test disturbance from the solicited body through up the system until reaching the
       * root.
       * <p>
       * The disturbance can either be a wrenhc or impulse.
       * </p>
       * 
       * @param sourceChild the child that is located between this and the target of the test wrench.
       */
      public void initializeDisturbance(ResponseRecursionStep sourceChild, DisturbanceSource source)
      {
         // Going bottom-up in the tree.
         isOnDisturbedBranch = true;

         if (isRoot())
         {
            isUpToDate = true;
            return;
         }

         stepUpDisturbance(sourceChild, source);
         parent.initializeDisturbance(this, source);
      }

      public void stepUpDisturbance(ResponseRecursionStep sourceChild, DisturbanceSource source)
      {
         isUpToDate = false;

         if (sourceChild != null)
         {
            testDisturbancePlus.setIncludingFrame(sourceChild.testDisturbancePlusForParent);
            testDisturbancePlus.applyTransform(sourceChild.articulatedBodyRecursionStep.transformToParentJointFrame);
            testDisturbancePlus.setReferenceFrame(getFrameAfterJoint());
            testDisturbancePlus.get(pAPlus);
            DenseMatrix64F S = articulatedBodyRecursionStep.S;
            CommonOps.multTransA(-1.0, S, pAPlus, uPlus);
         }
         else
         {
            if (source == DisturbanceSource.RIGID_BODY)
            {
               testDisturbancePlus.changeFrame(getFrameAfterJoint());
               testDisturbancePlus.get(pAPlus);
               CommonOps.changeSign(pAPlus);
               DenseMatrix64F S = articulatedBodyRecursionStep.S;
               CommonOps.multTransA(-1.0, S, pAPlus, uPlus);
            }
            else if (source == DisturbanceSource.JOINT)
            {
               pAPlus.zero();
               uPlus.set(tauPlus);
            }
            else
            {
               throw new IllegalStateException("Unexpected DisturbanceSource: " + source);
            }
         }

         if (!parent.isRoot())
         {
            DenseMatrix64F U_Dinv = articulatedBodyRecursionStep.U_Dinv;
            CommonOps.mult(U_Dinv, uPlus, paPlus);
            CommonOps.addEquals(paPlus, pAPlus);
            testDisturbancePlusForParent.setIncludingFrame(testDisturbancePlus.getReferenceFrame(), paPlus);
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
            DenseMatrix64F S = articulatedBodyRecursionStep.S;
            DenseMatrix64F U = articulatedBodyRecursionStep.U;
            DenseMatrix64F Dinv = articulatedBodyRecursionStep.Dinv;

            // Going top-down in the tree.
            parentMotionChange.setIncludingFrame(parent.rigidBodyMotionChange);
            parentMotionChange.applyInverseTransform(articulatedBodyRecursionStep.transformToParentJointFrame);
            parentMotionChange.setReferenceFrame(getFrameAfterJoint());
            parentMotionChange.get(aParentPlus);

            if (isOnDisturbedBranch)
            {
               // Computing qdd = D^-1 * ( u - U^T * a_parent )
               CommonOps.multTransA(-1.0, U, aParentPlus, qddPlus_intermediate);
               CommonOps.addEquals(qddPlus_intermediate, uPlus);
               CommonOps.mult(Dinv, qddPlus_intermediate, qddPlus);
            }
            else
            {
               // Computing qdd = -D^-1 * U^T * a_parent )
               CommonOps.multTransA(-1.0, U, aParentPlus, qddPlus_intermediate);
               CommonOps.mult(Dinv, qddPlus_intermediate, qddPlus);
            }

            CommonOps.mult(S, qddPlus, aPlus);
            CommonOps.addEquals(aPlus, aParentPlus);
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
