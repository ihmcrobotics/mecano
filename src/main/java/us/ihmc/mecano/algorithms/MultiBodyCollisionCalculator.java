package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator.ArticulatedBodyRecursionStep;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Inspired from Mirtich's thesis, this calculator allows to evaluate the perturbation in terms of
 * change in acceleration due to a wrench applied on a rigid-body.
 * <p>
 * This can be used to compute an adequate wrench to apply on a contacting rigid-body that is in
 * contact with another body.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class MultiBodyCollisionCalculator
{
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;
   /** The root of the internal recursive algorithm. */
   private final CollisionRecursionStep initialRecursionStep;
   /** Map to quickly retrieve information for each rigid-body. */
   private final Map<RigidBodyReadOnly, CollisionRecursionStep> rigidBodyToRecursionStepMap = new HashMap<>();
   /** Array of all the recursion steps to quickly performs independent operations on all of them. */
   private final CollisionRecursionStep[] allRecursionSteps;
   /** The output of this algorithm: the acceleration matrix for all the joints to consider. */
   private final DenseMatrix64F jointAccelerationChangeMatrix;
   /**
    * This algorithm relies on the pre-computed internal data from a forward dynamics algorithm.
    */
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;

   /**
    * Extension of this algorithm into an acceleration provider that can be used to retrieve change in
    * acceleration to any rigid-body of the system.
    */
   private final RigidBodyAccelerationProvider accelerationProvider;

   public MultiBodyCollisionCalculator(MultiBodySystemReadOnly input)
   {
      this(new ForwardDynamicsCalculator(input));
   }

   public MultiBodyCollisionCalculator(ForwardDynamicsCalculator forwardDynamicsCalculator)
   {
      this.forwardDynamicsCalculator = forwardDynamicsCalculator;
      input = forwardDynamicsCalculator.getInput();
      initialRecursionStep = new CollisionRecursionStep(forwardDynamicsCalculator.getInitialRecursionStep(), null);
      buildMultiBodyTree(initialRecursionStep);
      allRecursionSteps = rigidBodyToRecursionStepMap.values().toArray(new CollisionRecursionStep[0]);

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      jointAccelerationChangeMatrix = new DenseMatrix64F(nDoFs, 1);

      Function<RigidBodyReadOnly, SpatialAccelerationReadOnly> accelerationFunction = body ->
      {
         CollisionRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(body);
         if (recursionStep == null)
            return null;
         // The algorithm computes the acceleration expressed in the parent joint frame.
         // To prevent unnecessary computation, let's only change the frame when needed.
         recursionStep.rigidBodyAccelerationChange.changeFrame(body.getBodyFixedFrame());
         return recursionStep.rigidBodyAccelerationChange;
      };
      accelerationProvider = RigidBodyAccelerationProvider.toRigidBodyAccelerationProvider(accelerationFunction, input.getInertialFrame());
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

   private void reset()
   {
      for (CollisionRecursionStep recursionStep : allRecursionSteps)
         recursionStep.reset();
   }

   /**
    * Computes the change joint accelerations resulting from the given test wrench.
    * <p>
    * WARNING: {@link ForwardDynamicsCalculator#compute()} has to be manually called before calling
    * this method.
    * </p>
    * 
    * @param target                   the rigid-body to apply the test wrench to.
    * @param testWrench               the wrench to apply and evaluate the change in acceleration. Not
    *                                 modified.
    * @param accelerationChangeToPack the change in acceleration of the target body. Can be
    *                                 {@code null}. Modified.
    * @return joint acceleration matrix containing the change in acceleration for every joint of the
    *         system.
    */
   public DenseMatrix64F compute(RigidBodyReadOnly target, WrenchReadOnly testWrench, SpatialAccelerationBasics accelerationChangeToPack)
   {
      CollisionRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(target);

      if (recursionStep == null)
         return null;

      reset();
      jointAccelerationChangeMatrix.zero();
      recursionStep.testWrenchPlus.setIncludingFrame(testWrench);
      recursionStep.passOne(null);
      initialRecursionStep.passTwo();
      if (accelerationChangeToPack != null)
         accelerationChangeToPack.setIncludingFrame(recursionStep.rigidBodyAccelerationChange);
      return jointAccelerationChangeMatrix;
   }

   /**
    * Gets the rigid-body acceleration provider that can be used to access change in acceleration of
    * any rigid-body in the system due to the test wrench.
    * 
    * @return the acceleration change provider.
    */
   public RigidBodyAccelerationProvider getAccelerationChangeProvider()
   {
      return accelerationProvider;
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
      }

      /**
       * Propagates the test wrench from the solicited body through up the system until reaching the root.
       * 
       * @param sourceChild the child that is located between this and the target of the test wrench.
       */
      public void passOne(CollisionRecursionStep sourceChild)
      {
         if (isRoot())
            return;

         // Going bottom-up in the tree.
         isOnCollisionBranch = true;

         if (sourceChild != null)
            testWrenchPlus.setIncludingFrame(sourceChild.testWrenchPlusForParent);

         DenseMatrix64F S = articulatedBodyRecursionStep.S;
         DenseMatrix64F U_Dinv = articulatedBodyRecursionStep.U_Dinv;

         testWrenchPlus.changeFrame(getFrameAfterJoint());
         testWrenchPlus.get(pAPlus);
         if (sourceChild == null)
         { // p+ = -f_ext.
            CommonOps.multTransA(S, pAPlus, uPlus);
            CommonOps.mult(U_Dinv, uPlus, paPlus);
            CommonOps.subtractEquals(paPlus, pAPlus);
         }
         else
         {
            CommonOps.multTransA(-1.0, S, pAPlus, uPlus);
            CommonOps.mult(U_Dinv, uPlus, paPlus);
            CommonOps.addEquals(paPlus, pAPlus);
         }

         if (!parent.isRoot())
         {
            testWrenchPlusForParent.setIncludingFrame(testWrenchPlus.getReferenceFrame(), paPlus);
         }

         parent.passOne(this);
      }

      /**
       * Propagates the change in acceleration from the root down to the leaves.
       */
      public void passTwo()
      {
         if (!isRoot())
         {
            DenseMatrix64F S = articulatedBodyRecursionStep.S;
            DenseMatrix64F U = articulatedBodyRecursionStep.U;
            DenseMatrix64F Dinv = articulatedBodyRecursionStep.Dinv;

            // Going top-down in the tree.
            parentAccelerationChange.setIncludingFrame(parent.rigidBodyAccelerationChange);
            parentAccelerationChange.changeFrame(getFrameAfterJoint());
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

            int[] jointIndices = articulatedBodyRecursionStep.jointIndices;

            for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
            {
               jointAccelerationChangeMatrix.set(jointIndices[dofIndex], 0, qddPlus.get(dofIndex, 0));
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).passTwo();
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
