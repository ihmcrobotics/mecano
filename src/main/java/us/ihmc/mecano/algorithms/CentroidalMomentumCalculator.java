package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.FixedFrameMomentumBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Computes the centroidal momentum matrix that maps from joint velocity space to the system linear
 * and angular momentum.
 * <p>
 * Note on kinematic loops: the computed centroidal momentum matrix will be filled of zeros for the
 * loop closure joints. By externally constraining the configuration and velocity of the joints
 * composing a kinematic loop, the results from this calculator will remain accurate.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class CentroidalMomentumCalculator implements ReferenceFrameHolder
{
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;
   /** The center of mass centroidal momentum matrix. */
   private final ReferenceFrame matrixFrame;

   /** Array for each iteration of this algorithm. */
   private final IterativeStep[] iterativeSteps;

   /** Intermediate variable to store the unit-twist of the parent joint. */
   private final Twist jointUnitTwist;
   /** Intermediate variable for garbage free operations. */
   private final Twist intermediateUnitTwist;
   /** Intermediate variable to store one column of the centroidal momentum matrix. */
   private final Momentum unitMomentum;
   /** Intermediate variable for garbage free operations. */
   private final Momentum intermediateMomentum;
   /** The total momentum of the system. */
   private final FixedFrameMomentumBasics momentum;
   /** The center of mass velocity. */
   private final FixedFrameVector3DBasics centerOfMassVelocity;

   /** The centroidal momentum matrix. */
   private final DMatrixRMaj centroidalMomentumMatrix;
   /** Matrix containing the velocities of the joints to consider. */
   private final DMatrixRMaj jointVelocityMatrix;
   /** The total momentum of the system. */
   private final DMatrixRMaj momentumMatrix = new DMatrixRMaj(6, 1);

   /** The total system mass. */
   private double totalMass = 0.0;

   /**
    * Whether the centroidal momentum matrix has been updated since the last call to {@link #reset()}.
    */
   private boolean isCentroidalMomentumUpToDate = false;
   /**
    * Whether the joint velocity matrix has been updated since the last call to {@link #reset()}.
    */
   private boolean isJointVelocityMatrixUpToDate = false;
   /** Whether the momentum has been updated since the last call to {@link #reset()}. */
   private boolean isMomentumUpToDate = false;
   /** Whether the total mass has been updated since the last call to {@link #reset()}. */
   private boolean isTotalMassUpToDate = false;
   /**
    * Whether the center of mass velocity has been updated since the last call to {@link #reset()}.
    */
   private boolean isCenterOfMassVelocityUpToDate = false;

   /**
    * Creates a new calculator for the subtree that starts off the given {@code rootBody}.
    *
    * @param rootBody    the start of subtree for which the centroidal momentum matrix is to be
    *                    computed. Not modified.
    * @param matrixFrame the frame in which the centroidal momentum matrix is to be expressed.
    */
   public CentroidalMomentumCalculator(RigidBodyReadOnly rootBody, ReferenceFrame matrixFrame)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), matrixFrame);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input       the definition of the system to be evaluated by this calculator.
    * @param matrixFrame the frame in which the centroidal momentum matrix is to be expressed.
    */
   public CentroidalMomentumCalculator(MultiBodySystemReadOnly input, ReferenceFrame matrixFrame)
   {
      this(input, matrixFrame, true);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input                          the definition of the system to be evaluated by this
    *                                       calculator.
    * @param matrixFrame                    the frame in which the centroidal momentum matrix is to be
    *                                       expressed.
    * @param considerIgnoredSubtreesInertia whether the inertia of the ignored part(s) of the given
    *                                       multi-body system should be considered. When {@code true},
    *                                       this provides a more accurate centroidal momentum matrix,
    *                                       while when {@code false}, this calculator may gain slight
    *                                       performance improvement.
    */
   public CentroidalMomentumCalculator(MultiBodySystemReadOnly input, ReferenceFrame matrixFrame, boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;
      this.matrixFrame = matrixFrame;

      RigidBodyReadOnly rootBody = input.getRootBody();
      IterativeStep initialIterativeStep = new IterativeStep(rootBody, null);
      iterativeSteps = buildMultiBodyTree(initialIterativeStep, input.getJointsToIgnore()).toArray(new IterativeStep[0]);

      if (considerIgnoredSubtreesInertia)
         initialIterativeStep.updateIgnoredSubtreeInertia();

      jointUnitTwist = new Twist();
      unitMomentum = new Momentum(matrixFrame);
      intermediateUnitTwist = new Twist();
      intermediateMomentum = new Momentum();

      momentum = new Momentum(matrixFrame);
      centerOfMassVelocity = new FrameVector3D(matrixFrame);

      int nDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      centroidalMomentumMatrix = new DMatrixRMaj(6, nDegreesOfFreedom);
      jointVelocityMatrix = new DMatrixRMaj(nDegreesOfFreedom, 1);
   }

   private List<IterativeStep> buildMultiBodyTree(IterativeStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<IterativeStep> iterativeSteps = new ArrayList<>();
      iterativeSteps.add(parent);

      List<JointReadOnly> childrenJoints = MultiBodySystemTools.sortLoopClosureInChildrenJoints(parent.rigidBody);

      for (JointReadOnly childJoint : childrenJoints)
      {
         if (jointsToIgnore.contains(childJoint))
            continue;

         if (childJoint.isLoopClosure())
         {
            /*
             * We simply skip any loop closure joint which will leave their columns in the matrix set to zero,
             * which is what we want.
             */
            continue;
         }

         RigidBodyReadOnly childBody = childJoint.getSuccessor();
         if (childBody != null)
         {
            int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
            IterativeStep child = new IterativeStep(childBody, jointIndices);
            parent.children.add(child);
            iterativeSteps.addAll(buildMultiBodyTree(child, jointsToIgnore));
         }
      }

      return iterativeSteps;
   }

   /**
    * Invalidates the internal memory.
    */
   public void reset()
   {
      isCentroidalMomentumUpToDate = false;
      isJointVelocityMatrixUpToDate = false;
      isMomentumUpToDate = false;
      isTotalMassUpToDate = false;
      isCenterOfMassVelocityUpToDate = false;
   }

   private void updateCentroidalMomentum()
   {
      if (isCentroidalMomentumUpToDate)
         return;

      passOne();
      passTwo();
      isCentroidalMomentumUpToDate = true;
   }

   /**
    * Iterative method that calls {@link IterativeStep#passOne()}.
    *
    * @see IterativeStep#passOne()
    */
   private void passOne()
   {
      for (IterativeStep iterativeStep : iterativeSteps)
         iterativeStep.passOne();
   }

   /**
    * Iterative method that calls {@link IterativeStep#passTwo()}.
    *
    * @see IterativeStep#passTwo()
    */
   private void passTwo()
   {
      for (IterativeStep iterativeStep : iterativeSteps)
         iterativeStep.passTwo();
   }

   private DMatrixRMaj getJointVelocityMatrix()
   {
      if (!isJointVelocityMatrixUpToDate)
      {
         List<? extends JointReadOnly> joints = input.getJointMatrixIndexProvider().getIndexedJointsInOrder();
         MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocityMatrix);
         isJointVelocityMatrixUpToDate = true;
      }
      return jointVelocityMatrix;
   }

   /**
    * Gets the definition of the multi-body system that was used to create this calculator.
    *
    * @return this calculator input.
    */
   public MultiBodySystemReadOnly getInput()
   {
      return input;
   }

   /**
    * Gets the total mass of the multi-body system.
    *
    * @return the mass of the multi-body system.
    */
   public double getTotalMass()
   {
      if (!isTotalMassUpToDate)
      {
         totalMass = 0.0;
         List<? extends JointReadOnly> jointsToConsider = input.getJointsToConsider();

         for (int i = 0; i < jointsToConsider.size(); i++)
         {
            totalMass += jointsToConsider.get(i).getSuccessor().getInertia().getMass();
         }
         isTotalMassUpToDate = true;
      }
      return totalMass;
   }

   /**
    * Gets the center of mass velocity of the multi-body system.
    *
    * @return the center of mass velocity.
    */
   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      if (!isCenterOfMassVelocityUpToDate)
      {
         centerOfMassVelocity.setAndScale(1.0 / getTotalMass(), getMomentum().getLinearPart());
         isCenterOfMassVelocityUpToDate = true;
      }
      return centerOfMassVelocity;
   }

   /**
    * Computes and packs the center of mass velocity for the given joint velocities.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    *
    * @param jointVelocityMatrix        the matrix containing the joint velocities to use. Not
    *                                   modified.
    * @param centerOfMassVelocityToPack the vector used to stored the computed center of mass velocity.
    *                                   Modified.
    */
   public void getCenterOfMassVelocity(DMatrix1Row jointVelocityMatrix, FrameVector3DBasics centerOfMassVelocityToPack)
   {
      CommonOps_DDRM.mult(getCentroidalMomentumMatrix(), jointVelocityMatrix, momentumMatrix);
      centerOfMassVelocityToPack.setIncludingFrame(matrixFrame, 3, momentumMatrix);
      centerOfMassVelocityToPack.scale(1.0 / getTotalMass());
   }

   /**
    * Gets the momentum of the multi-body system.
    *
    * @return the momentum.
    */
   public MomentumReadOnly getMomentum()
   {
      if (!isMomentumUpToDate)
      {
         CommonOps_DDRM.mult(getCentroidalMomentumMatrix(), getJointVelocityMatrix(), momentumMatrix);
         momentum.set(momentumMatrix);
         isMomentumUpToDate = true;
      }
      return momentum;
   }

   /**
    * Computes and packs the momentum for the given joint velocities.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    *
    * @param jointVelocityMatrix the matrix containing the joint velocities to use. Not modified.
    * @param momentumToPack      the vector used to stored the computed momentum. Modified.
    */
   public void getMomentum(DMatrix1Row jointVelocityMatrix, MomentumBasics momentumToPack)
   {
      CommonOps_DDRM.mult(getCentroidalMomentumMatrix(), jointVelocityMatrix, momentumMatrix);
      momentumToPack.setIncludingFrame(matrixFrame, momentumMatrix);
   }

   /**
    * Gets the N-by-6 centroidal momentum matrix, where N is the number of degrees of freedom of the
    * multi-body system.
    * <p>
    * The centroidal momentum matrix maps from joint velocity space to momentum space and is expressed
    * in the frame {@link #getReferenceFrame()}. The latter implies that when multiplied to the joint
    * velocity matrix, the result is the momentum expressed in {@link #getReferenceFrame()}.
    * </p>
    *
    * @return the centroidal momentum matrix.
    */
   public DMatrixRMaj getCentroidalMomentumMatrix()
   {
      updateCentroidalMomentum();
      return centroidalMomentumMatrix;
   }

   /**
    * The reference frame in which the centroidal momentum matrix is computed.
    *
    * @return this calculator's reference frame.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return matrixFrame;
   }

   /**
    * Represents a single iteration step with all the intermediate variables needed.
    *
    * @author Sylvain Bertrand
    */
   private class IterativeStep
   {
      /**
       * The rigid-body for which this recursion is.
       */
      private final RigidBodyReadOnly rigidBody;
      /**
       * Used for storing intermediate result when part of the subtree is ignored but the subtree inertia
       * is still to be considered.
       */
      private SpatialInertia bodyInertia;
      /**
       * Combined inertia of the subtree that is ignored but inertia is still to be considered for this
       * recursion step.
       */
      private SpatialInertia bodySubtreeInertia;
      /**
       * Result of this recursion step: the matrix block of the centroidal momentum matrix for the parent
       * joint.
       */
      private final DMatrixRMaj centroidalMomentumMatrixBlock;
      /**
       * Intermediate variable to prevent repetitive calculation of a transform between this rigid-body's
       * body-fixed frame and the matrix frame.
       */
      private final RigidBodyTransform matrixFrameToBodyFixedFrameTransform;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      private final List<IterativeStep> children = new ArrayList<>();
      /**
       * Joint indices for storing {@code centroidalMomentumMatrixBlock} in the main matrix
       * {@code centroidalMomentumMatrix}.
       */
      private final int[] jointIndices;

      public IterativeStep(RigidBodyReadOnly rigidBody, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.jointIndices = jointIndices;

         if (isRoot())
         {
            centroidalMomentumMatrixBlock = null;
            matrixFrameToBodyFixedFrameTransform = null;
         }
         else
         {
            centroidalMomentumMatrixBlock = new DMatrixRMaj(6, getJoint().getDegreesOfFreedom());
            matrixFrameToBodyFixedFrameTransform = new RigidBodyTransform();
         }
      }

      private void updateIgnoredSubtreeInertia()
      {
         if (bodySubtreeInertia != null)
         {
            bodyInertia.setToZero();
            bodySubtreeInertia.setToZero();
         }

         if (!isRoot() && children.size() != rigidBody.getChildrenJoints().size())
         {
            for (JointReadOnly childJoint : rigidBody.getChildrenJoints())
            {
               if (input.getJointsToIgnore().contains(childJoint))
               {
                  SpatialInertia subtreeInertia = MultiBodySystemTools.computeSubtreeInertia(childJoint);
                  subtreeInertia.changeFrame(getBodyFixedFrame());
                  if (bodySubtreeInertia == null)
                  {
                     bodyInertia = new SpatialInertia(getBodyFixedFrame(), getBodyFixedFrame());
                     bodySubtreeInertia = new SpatialInertia(getBodyFixedFrame(), getBodyFixedFrame());
                  }
                  bodySubtreeInertia.add(subtreeInertia);
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).updateIgnoredSubtreeInertia();
      }

      /**
       * First pass that can be done iteratively and each iteration is independent.
       * <p>
       * Computes and stores the transform from the matrix frame to this rigid-body's body-fixed frame.
       * </p>
       */
      public void passOne()
      {
         if (isRoot())
            return;

         ReferenceFrame inertiaFrame = rigidBody.getInertia().getReferenceFrame();
         matrixFrame.getTransformToDesiredFrame(matrixFrameToBodyFixedFrameTransform, inertiaFrame);
      }

      /**
       * Second pass that can be done iteratively and each iteration is independent.
       * <p>
       * Computes the block of the centroidal momentum matrix for this parent joint.
       * </p>
       */
      public void passTwo()
      {
         if (isRoot())
            return;

         for (int i = 0; i < getJoint().getDegreesOfFreedom(); i++)
         {
            unitMomentum.setToZero();
            jointUnitTwist.setIncludingFrame(getJoint().getUnitTwists().get(i));
            jointUnitTwist.changeFrame(matrixFrame);
            addToUnitMomentumRecursively(jointUnitTwist, unitMomentum);
            unitMomentum.get(0, i, centroidalMomentumMatrixBlock);
         }

         for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
         {
            int column = jointIndices[dofIndex];
            CommonOps_DDRM.extract(centroidalMomentumMatrixBlock, 0, 6, dofIndex, dofIndex + 1, centroidalMomentumMatrix, 0, column);
         }
      }

      /**
       * Builds up recursively the {@code unitMomentumToAddTo} to account for the whole subtree inertia.
       * <p>
       * Theoretically, the inertia corresponding to the subtree should be computed to then compute the
       * unit-momentum (left-hand side of the equation below), but this approach (right-hand side of the
       * equation below) is equivalent and much cheaper. The left-hand side formula requires to change the
       * frame of each spatial inertia matrix which results in heavy computation, the right-hand side only
       * requires to change the frame of the unit-twist.
       * </p>
       *
       * <pre>
       * h = (&sum;<sub>i=0:n</sub> I<sub>i</sub>) * T &equiv; &sum;<sub>i=0:n</sub> (I<sub>i</sub> * T)
       * </pre>
       *
       * where <tt>h</tt> is the resulting unit-momentum, <tt>I<sub>i</sub></tt> is the spatial inertia of
       * the i<sup>th</sup> body, and <tt>T</tt> is the unit-twist.
       *
       * @param ancestorUnitTwist   the unit-twist to use for computing a the unit-momentum for this body
       *                            that is then added to {@code unitMomentumToAddTo}. Not modified.
       * @param unitMomentumToAddTo the unit-momentum to build up. Modified.
       */
      private void addToUnitMomentumRecursively(TwistReadOnly ancestorUnitTwist, FixedFrameMomentumBasics unitMomentumToAddTo)
      {
         ReferenceFrame inertiaFrame = rigidBody.getInertia().getReferenceFrame();

         intermediateUnitTwist.setIncludingFrame(ancestorUnitTwist);
         intermediateUnitTwist.applyTransform(matrixFrameToBodyFixedFrameTransform);
         intermediateUnitTwist.setReferenceFrame(inertiaFrame);

         intermediateMomentum.setReferenceFrame(inertiaFrame);

         if (bodyInertia != null)
         {
            bodyInertia.setIncludingFrame(rigidBody.getInertia());
            bodyInertia.add(bodySubtreeInertia);
            intermediateMomentum.compute(bodyInertia, intermediateUnitTwist);
         }
         else
         {
            intermediateMomentum.compute(rigidBody.getInertia(), intermediateUnitTwist);
         }

         intermediateMomentum.applyInverseTransform(matrixFrameToBodyFixedFrameTransform);
         intermediateMomentum.setReferenceFrame(matrixFrame);

         unitMomentumToAddTo.add(intermediateMomentum);

         for (int i = 0; i < children.size(); i++)
            children.get(i).addToUnitMomentumRecursively(ancestorUnitTwist, unitMomentumToAddTo);
      }

      public boolean isRoot()
      {
         return jointIndices == null;
      }

      public JointReadOnly getJoint()
      {
         return rigidBody.getParentJoint();
      }

      public MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }
   }
}