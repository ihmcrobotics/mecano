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
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.*;
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
 * // To compute the linear part of apparent inertia 
 * calculator.computeApparentInertia(targetRigidBody, pointInTarget, externalWrenchFrame);
 * calculator.applyWrench(targetRigidBody, externalWrench);
 * // To obtain the resulting change in acceleration caused by the externalWrench on another rigid-body:
 * SpatialAccelerationReadOnly resultingAccelerationChangeOnOtherBody = calculator.getAccelerationChangeProvider().getAccelerationOfBody(anotherRigidBody);
 * </pre>
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class MultiBodyResponseCalculator
{
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;
   /** The root of the internal recursive algorithm. */
   private final CollisionRecursionStep initialRecursionStep;
   /** Map to quickly retrieve information for each rigid-body. */
   private final Map<RigidBodyReadOnly, CollisionRecursionStep> rigidBodyToRecursionStepMap = new HashMap<>();
   /** The output of this algorithm: the acceleration matrix for all the joints to consider. */
   private final DenseMatrix64F jointAccelerationChangeMatrix;
   /** The output of this algorithm: the velocity matrix for all the joints to consider. */
   private final DenseMatrix64F jointVelocityChangeMatrix;
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

   private enum Mode
   {
      ACCELERATION, TWIST
   };

   private Mode currentMode = null;

   public MultiBodyResponseCalculator(MultiBodySystemReadOnly input)
   {
      this(new ForwardDynamicsCalculator(input));
   }

   public MultiBodyResponseCalculator(ForwardDynamicsCalculator forwardDynamicsCalculator)
   {
      this.forwardDynamicsCalculator = forwardDynamicsCalculator;
      input = forwardDynamicsCalculator.getInput();
      initialRecursionStep = new CollisionRecursionStep(forwardDynamicsCalculator.getInitialRecursionStep(), null);
      buildMultiBodyTree(initialRecursionStep);

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      jointAccelerationChangeMatrix = new DenseMatrix64F(nDoFs, 1);
      jointVelocityChangeMatrix = new DenseMatrix64F(nDoFs, 1);

      accelerationChangeProvider = RigidBodyAccelerationProvider.toRigidBodyAccelerationProvider(buildAccelerationSupplier(), input.getInertialFrame());
      twistChangeProvider = RigidBodyTwistProvider.toRigidBodyTwistProvider(buildTwistSupplier(), input.getInertialFrame());
   }

   private Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> buildAccelerationSupplier()
   {
      SpatialAcceleration bodyAcceleration = new SpatialAcceleration();

      Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> accelerationFunction = body ->
      {
         if (currentMode == Mode.TWIST)
            throw new IllegalStateException("This calculator is currently setup for calculating twists.");

         CollisionRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);
         if (recursionStep == null)
            return null;
         if (currentMode == null || !initialRecursionStep.isUpToDate)
         { // This calculator has not been initialized with an wrench yet.
            bodyAcceleration.setToZero(body.getBodyFixedFrame(), input.getInertialFrame(), body.getBodyFixedFrame());
         }
         else
         {
            recursionStep.updateRigidBodyAccelerationChange();
            // The algorithm computes the acceleration expressed in the parent joint frame.
            // To prevent unnecessary computation, let's only change the frame when needed.
            bodyAcceleration.setIncludingFrame(recursionStep.rigidBodyAccelerationChange);
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
         if (currentMode == Mode.ACCELERATION)
            throw new IllegalStateException("This calculator is currently setup for calculating accelerations.");

         CollisionRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);
         if (recursionStep == null)
            return null;

         if (!initialRecursionStep.isUpToDate)
         { // This calculator has not been initialized with an impulse yet.
            bodyTwist.setToZero(body.getBodyFixedFrame(), input.getInertialFrame(), body.getBodyFixedFrame());
         }
         else
         {
            recursionStep.updateRigidBodyTwistChange();
            // The algorithm computes the twist expressed in the parent joint frame.
            // To prevent unnecessary computation, let's only change the frame when needed.
            bodyTwist.setIncludingFrame(recursionStep.rigidBodyTwistChange);
            bodyTwist.changeFrame(body.getBodyFixedFrame());
         }
         return bodyTwist;
      };
      return twistFunction;
   }

   private void buildMultiBodyTree(CollisionRecursionStep recursionStep)
   {
      for (ArticulatedBodyRecursionStep childInertia : recursionStep.articulatedBodyRecursionStep.children)
      {
         CollisionRecursionStep child = new CollisionRecursionStep(childInertia, recursionStep);
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
      currentMode = null;
      initialRecursionStep.reset();
   }

   public boolean computeApparentSpatialInertiaInverse(RigidBodyReadOnly target, ReferenceFrame inertiaFrame, DenseMatrix64F apparentSpatialInertiaToPack)
   {
      return computeApparentSpatialInertiaInverse(target, inertiaFrame, null, apparentSpatialInertiaToPack);
   }

   public boolean computeApparentSpatialInertiaInverse(RigidBodyReadOnly target, ReferenceFrame inertiaFrame, boolean[] selectedAxes,
                                                DenseMatrix64F apparentSpatialInertiaToPack)
   {
      CollisionRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      ReferenceFrame bodyFrame = target.getBodyFixedFrame();

      apparentSpatialInertiaToPack.reshape(6, 6);

      for (int axis = 0; axis < 3; axis++)
      {
         if (selectedAxes == null || selectedAxes[axis])
         {
            recursionStep.testWrenchPlus.setIncludingFrame(bodyFrame, inertiaFrame, Axis.values[axis], EuclidCoreTools.zeroVector3D);
            recursionStep.initializeWrench(null);
            recursionStep.updateRigidBodyAccelerationChange();
            recursionStep.rigidBodyAccelerationChange.changeFrame(inertiaFrame);
            recursionStep.rigidBodyAccelerationChange.get(0, axis, apparentSpatialInertiaToPack);
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
            recursionStep.testWrenchPlus.setIncludingFrame(bodyFrame, inertiaFrame, EuclidCoreTools.zeroVector3D, Axis.values[axis]);
            recursionStep.initializeWrench(null);
            recursionStep.updateRigidBodyAccelerationChange();
            recursionStep.rigidBodyAccelerationChange.changeFrame(inertiaFrame);
            recursionStep.rigidBodyAccelerationChange.get(0, axis + 3, apparentSpatialInertiaToPack);
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
    * @param apparentLinearInertiaToPack the matrix in which to store the result.
    * @return {@code true} is the apparent inertia matrix was successfully computed, {@code false}
    *         otherwise.
    */
   public boolean computeApparentLinearInertiaInverse(RigidBodyReadOnly target, ReferenceFrame inertiaFrame, DenseMatrix64F apparentLinearInertiaToPack)
   {
      CollisionRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      ReferenceFrame bodyFrame = target.getBodyFixedFrame();
      apparentLinearInertiaToPack.reshape(3, 3);

      for (int axis = 0; axis < 3; axis++)
      {
         recursionStep.testWrenchPlus.setIncludingFrame(bodyFrame, inertiaFrame, EuclidCoreTools.zeroVector3D, Axis.values[axis], EuclidCoreTools.origin3D);
         recursionStep.initializeWrench(null);
         recursionStep.updateRigidBodyAccelerationChange();
         recursionStep.rigidBodyAccelerationChange.changeFrame(inertiaFrame);
         recursionStep.rigidBodyAccelerationChange.getLinearPart().get(0, axis, apparentLinearInertiaToPack);
      }
      reset();

      return true;
   }

   /**
    * Applies a 6-D wrench to {@code target} and propagates the effect to the rest of the multi-body
    * system.
    * 
    * @param target the rigid-body to which the wrench is applied to.
    * @param wrench the wrench to be applied.
    * @return the response to the wrench on the multi-body system in terms of change of joint
    *         accelerations, or {@code null} if this methods failed.
    */
   public DenseMatrix64F applyAndPropagateWrench(RigidBodyReadOnly target, WrenchReadOnly wrench)
   {
      if (!applyWrench(target, wrench))
         return null;

      return propagateWrench();
   }

   /**
    * Applies a 6-D impulse to {@code target} and propagates the effect to the rest of the multi-body
    * system.
    * 
    * @param target  the rigid-body to which the impulse is applied to.
    * @param impulse the impulse to be applied.
    * @return the response to the impulse on the multi-body system in terms of change of joint
    *         velocities, or {@code null} if this methods failed.
    */
   public DenseMatrix64F applyAndPropagateImpulse(RigidBodyReadOnly target, SpatialImpulseReadOnly impulse)
   {
      if (!applyImpulse(target, impulse))
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
    * @param wrench the wrench to be applied.
    * @return {@code true} if the wrench was successfully applied and the response computed,
    *         {@code false} otherwise.
    */
   public boolean applyWrench(RigidBodyReadOnly target, WrenchReadOnly wrench)
   {
      CollisionRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      reset();
      ReferenceFrame bodyFrame = target.getBodyFixedFrame();
      recursionStep.testWrenchPlus.setIncludingFrame(bodyFrame, wrench);
      recursionStep.initializeWrench(null);
      currentMode = Mode.ACCELERATION;
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
    * @param impulse the impulse to be applied.
    * @return {@code true} if the impulse was successfully applied, {@code false} otherwise.
    */
   public boolean applyImpulse(RigidBodyReadOnly target, SpatialImpulseReadOnly impulse)
   {
      CollisionRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return false;

      reset();
      ReferenceFrame bodyFrame = target.getBodyFixedFrame();
      recursionStep.testImpulsePlus.setIncludingFrame(bodyFrame, impulse);
      recursionStep.initializeImpulse(null);
      currentMode = Mode.TWIST;
      return true;
   }

   /**
    * Propagates a <b>previously</b> applied wrench to all rigid-bodies in the system and computes the
    * resulting change in acceleration.
    * 
    * @return the matrix with the resulting change in joint acceleration, or {@code null} if
    *         {@link #applyWrench(RigidBodyReadOnly, WrenchReadOnly)} was not called last.
    */
   public DenseMatrix64F propagateWrench()
   {
      if (currentMode != Mode.ACCELERATION || !initialRecursionStep.isUpToDate)
         return null;

      initialRecursionStep.propagateDownWrench();
      return jointAccelerationChangeMatrix;
   }

   /**
    * Propagates a <b>previously</b> applied impulse to all rigid-bodies in the system and computes the
    * resulting change in twist.
    * 
    * @return the matrix with the resulting change in joint velocity, or {@code null} if
    *         {@link #applyImpulse(RigidBodyReadOnly, SpatialImpulseReadOnly)} was not called last.
    */
   public DenseMatrix64F propagateImpulse()
   {
      if (currentMode != Mode.TWIST || !initialRecursionStep.isUpToDate)
         return null;

      initialRecursionStep.propagateDownImpulse();
      return jointVelocityChangeMatrix;
   }

   /**
    * Gets the rigid-body acceleration provider that can be used to access change in acceleration of
    * any rigid-body in the system due to the test wrench.
    * <p>
    * The provider is initialized only after calling either
    * {@link #applyWrench(RigidBodyReadOnly, WrenchReadOnly)} or
    * {@link #applyAndPropagateWrench(RigidBodyReadOnly, WrenchReadOnly)}.
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
    * {@link #applyWrench(RigidBodyReadOnly, WrenchReadOnly)} or
    * {@link #applyAndPropagateWrench(RigidBodyReadOnly, WrenchReadOnly)}.
    * </p>
    * 
    * @return the twist change provider.
    */
   public RigidBodyTwistProvider getTwistChangeProvider()
   {
      return twistChangeProvider;
   }

   class CollisionRecursionStep
   {
      /**
       * Test wrench containing external force applied to this body to test the resulting change
       * acceleration with.
       */
      final Wrench testWrenchPlus;
      /**
       * Pre-transformed test wrench for the parent.
       */
      final SpatialForce testWrenchPlusForParent;
      /**
       * Change of acceleration of this rigid-body.
       */
      final SpatialAcceleration rigidBodyAccelerationChange = new SpatialAcceleration();
      /**
       * Change of acceleration of this rigid-body's direct predecessor.
       */
      final SpatialAcceleration parentAccelerationChange;
      /**
       * Test wrench containing external force applied to this body to test the resulting change
       * acceleration with.
       */
      final SpatialImpulse testImpulsePlus;
      /**
       * Pre-transformed test wrench for the parent.
       */
      final SpatialForce testImpulsePlusForParent;
      /**
       * Change of acceleration of this rigid-body.
       */
      final Twist rigidBodyTwistChange = new Twist();
      /**
       * Change of acceleration of this rigid-body's direct predecessor.
       */
      final Twist parentTwistChange;
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
       * The apparent bias forces of this joint for the parent resulting from the {@code testWrenchPlus}:
       * 
       * <pre>
       * p<sup>a+</sup> = p<sup>A+</sup> + U D<sup>-1</sup> u<sup>+</sup>
       *  <sup>  </sup> = p<sup>A+</sup> - I<sup>A</sup> S ( S<sup>T</sup> I<sup>A</sup> S )<sup>-1</sup> S<sup>T</sup> p<sup>A+</sup>
       * </pre>
       * 
       * where <tt>p<sup>A+</sup></tt> is {@code testWrenchPlus} acting on this joint, <tt>S</tt> is the
       * joint motion subspace, <tt>I<sup>A</sup></tt> this handle's articulated-body inertia,
       * <tt>&tau;</tt> this joint effort.
       */
      final DenseMatrix64F paPlus;
      /**
       * The change in parent body acceleration.
       */
      final DenseMatrix64F aParentPlus;
      /**
       * <b>Output of this algorithm: the change in body acceleration:</b>
       * 
       * <pre>
       * a<sup>+</sup> = a<sup>+</sup><sub>parent</sub> + S qDDot<sup>+</sup>
       * </pre>
       */
      final DenseMatrix64F aPlus;
      /**
       * Intermediate result for garbage-free operation.
       */
      final DenseMatrix64F qddPlus_intermediate;
      /**
       * <b>Output of this algorithm: the change in joint acceleration:</b>
       * 
       * <pre>
       * qDDot<sup>+</sup> = D<sup>-1</sup> ( u<sup>+</sup> - U<sup>T</sup> a<sup>+</sup><sub>parent</sub> )
       * </pre>
       */
      final DenseMatrix64F qddPlus;
      /**
       * The forward-dynamics recursion step for the rigid-body of this recursion step.
       */
      private ArticulatedBodyRecursionStep articulatedBodyRecursionStep;
      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid-body.
       */
      private CollisionRecursionStep parent;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      final List<CollisionRecursionStep> children = new ArrayList<>();

      /**
       * Whether the rigid-body is located on the branch that starts a the root and ends at the target
       * body.
       */
      private boolean isOnCollisionBranch = false;
      /**
       * Flag used for lazy update.
       * <p>
       * When {@code true}, this means that the twist or acceleration of this step is up-to-date. When
       * {@code false}, either the calculator has been reset, or a wrench/impulse has been applied but not
       * propagated down the tree.
       * </p>
       */
      private boolean isUpToDate = false;

      public CollisionRecursionStep(ArticulatedBodyRecursionStep articulatedBodyRecursionStep, CollisionRecursionStep parent)
      {
         this.articulatedBodyRecursionStep = articulatedBodyRecursionStep;
         this.parent = parent;

         if (parent == null)
         {
            testWrenchPlus = null;
            testWrenchPlusForParent = null;
            parentAccelerationChange = null;
            rigidBodyAccelerationChange.setToZero(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());
            testImpulsePlus = null;
            testImpulsePlusForParent = null;
            parentTwistChange = null;
            rigidBodyTwistChange.setToZero(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());

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

            testWrenchPlus = new Wrench();
            testWrenchPlusForParent = parent.isRoot() ? null : new SpatialForce();
            parentAccelerationChange = new SpatialAcceleration();

            testImpulsePlus = new SpatialImpulse();
            testImpulsePlusForParent = parent.isRoot() ? null : new SpatialForce();
            parentTwistChange = new Twist();

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
         isOnCollisionBranch = false;
         isUpToDate = false;
         for (CollisionRecursionStep child : children)
            child.reset();
      }

      /**
       * Propagates the test wrench from the solicited body through up the system until reaching the root.
       * 
       * @param sourceChild the child that is located between this and the target of the test wrench.
       */
      public void initializeWrench(CollisionRecursionStep sourceChild)
      {
         // Going bottom-up in the tree.
         isOnCollisionBranch = true;

         if (isRoot())
         {
            isUpToDate = true;
            return;
         }

         stepUpWrench(sourceChild);
         parent.initializeWrench(this);
      }

      /**
       * Propagates the test impulse from the solicited body through up the system until reaching the
       * root.
       * 
       * @param sourceChild the child that is located between this and the target of the test impulse.
       */
      public void initializeImpulse(CollisionRecursionStep sourceChild)
      {
         // Going bottom-up in the tree.
         isOnCollisionBranch = true;

         if (isRoot())
         {
            isUpToDate = true;
            return;
         }

         stepUpImpulse(sourceChild);
         parent.initializeImpulse(this);
      }

      public void stepUpWrench(CollisionRecursionStep sourceChild)
      {
         isUpToDate = false;

         if (sourceChild != null)
         {
            testWrenchPlus.setIncludingFrame(sourceChild.testWrenchPlusForParent);
            testWrenchPlus.applyTransform(sourceChild.articulatedBodyRecursionStep.transformToParentJointFrame);
            testWrenchPlus.setReferenceFrame(getFrameAfterJoint());
         }
         else
         {
            testWrenchPlus.changeFrame(getFrameAfterJoint());
         }
         testWrenchPlus.get(pAPlus);
         if (sourceChild == null)
            CommonOps.changeSign(pAPlus);

         DenseMatrix64F S = articulatedBodyRecursionStep.S;
         CommonOps.multTransA(-1.0, S, pAPlus, uPlus);

         if (!parent.isRoot())
         {
            DenseMatrix64F U_Dinv = articulatedBodyRecursionStep.U_Dinv;
            CommonOps.mult(U_Dinv, uPlus, paPlus);
            CommonOps.addEquals(paPlus, pAPlus);
            testWrenchPlusForParent.setIncludingFrame(testWrenchPlus.getReferenceFrame(), paPlus);
         }
      }

      public void stepUpImpulse(CollisionRecursionStep sourceChild)
      {
         if (sourceChild != null)
         {
            testImpulsePlus.setIncludingFrame(sourceChild.testImpulsePlusForParent);
            testImpulsePlus.applyTransform(sourceChild.articulatedBodyRecursionStep.transformToParentJointFrame);
            testImpulsePlus.setReferenceFrame(getFrameAfterJoint());
         }
         else
         {
            testImpulsePlus.changeFrame(getFrameAfterJoint());
         }
         testImpulsePlus.get(pAPlus);
         if (sourceChild == null)
            CommonOps.changeSign(pAPlus);

         DenseMatrix64F S = articulatedBodyRecursionStep.S;
         CommonOps.multTransA(-1.0, S, pAPlus, uPlus);

         if (!parent.isRoot())
         {
            DenseMatrix64F U_Dinv = articulatedBodyRecursionStep.U_Dinv;
            CommonOps.mult(U_Dinv, uPlus, paPlus);
            CommonOps.addEquals(paPlus, pAPlus);
            testImpulsePlusForParent.setIncludingFrame(testImpulsePlus.getReferenceFrame(), paPlus);
         }
      }

      /**
       * Propagates the change in acceleration from the root down to the leaves.
       */
      public void propagateDownWrench()
      {
         stepDownWrench();

         if (!isRoot())
         {
            int[] jointIndices = articulatedBodyRecursionStep.jointIndices;

            for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
            {
               jointAccelerationChangeMatrix.set(jointIndices[dofIndex], 0, qddPlus.get(dofIndex, 0));
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).propagateDownWrench();
      }

      /**
       * Propagates the change in twist from the root down to the leaves.
       */
      public void propagateDownImpulse()
      {
         stepDownImpulse();

         if (!isRoot())
         {
            int[] jointIndices = articulatedBodyRecursionStep.jointIndices;

            for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
            {
               jointVelocityChangeMatrix.set(jointIndices[dofIndex], 0, qddPlus.get(dofIndex, 0));
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).propagateDownImpulse();
      }

      public void updateRigidBodyAccelerationChange()
      {
         if (isUpToDate || isRoot())
            return;

         parent.updateRigidBodyAccelerationChange();
         stepDownWrench();
      }

      public void updateRigidBodyTwistChange()
      {
         if (isUpToDate || isRoot())
            return;

         parent.updateRigidBodyTwistChange();
         stepDownImpulse();
      }

      private void stepDownWrench()
      {
         if (isUpToDate)
            return;

         if (!isRoot())
         {
            DenseMatrix64F S = articulatedBodyRecursionStep.S;
            DenseMatrix64F U = articulatedBodyRecursionStep.U;
            DenseMatrix64F Dinv = articulatedBodyRecursionStep.Dinv;

            // Going top-down in the tree.
            parentAccelerationChange.setIncludingFrame(parent.rigidBodyAccelerationChange);
            parentAccelerationChange.applyInverseTransform(articulatedBodyRecursionStep.transformToParentJointFrame);
            parentAccelerationChange.setReferenceFrame(getFrameAfterJoint());
            parentAccelerationChange.get(aParentPlus);

            if (isOnCollisionBranch)
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
            rigidBodyAccelerationChange.setIncludingFrame(getBodyFixedFrame(), input.getInertialFrame(), getFrameAfterJoint(), aPlus);
         }

         isUpToDate = true;
      }

      private void stepDownImpulse()
      {
         if (isUpToDate)
            return;

         if (!isRoot())
         {
            DenseMatrix64F S = articulatedBodyRecursionStep.S;
            DenseMatrix64F U = articulatedBodyRecursionStep.U;
            DenseMatrix64F Dinv = articulatedBodyRecursionStep.Dinv;

            // Going top-down in the tree.
            parentTwistChange.setIncludingFrame(parent.rigidBodyTwistChange);
            parentTwistChange.applyInverseTransform(articulatedBodyRecursionStep.transformToParentJointFrame);
            parentTwistChange.setReferenceFrame(getFrameAfterJoint());
            parentTwistChange.get(aParentPlus);

            if (isOnCollisionBranch)
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
            rigidBodyTwistChange.setIncludingFrame(getBodyFixedFrame(), input.getInertialFrame(), getFrameAfterJoint(), aPlus);
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

      public TwistReadOnly getBodyTwist()
      {
         return getBodyFixedFrame().getTwistOfFrame();
      }

      @Override
      public String toString()
      {
         return "RigidBody: " + articulatedBodyRecursionStep.rigidBody + ", parent: " + parent.articulatedBodyRecursionStep.rigidBody;
      }
   }
}
