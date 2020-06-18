package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

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
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameMomentumBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialForceBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Computes the centroidal momentum matrix that maps from joint velocity space to the system linear
 * and angular momentum and the convective term representing the Coriolis and centrifugal wrenches
 * acting on the multi-body system. With the latter, the mapping from joint accelerations space to
 * the rate of change of momentum space can be computed as follows:
 *
 * <pre>
 *  d                 /  d   \
 * -- h = A * qDDot + | -- A | * qDot = A * qDDot + b
 * dt                 \ dt   /
 * </pre>
 *
 * where <tt>h</tt> is the system's momentum, <tt>qDot</tt> and <tt>qDDot</tt> are the joint
 * velocity and acceleration vectors, <tt>A</tt> is the centroidal momentum matrix, and <tt>b</tt>
 * represents the convective term that this calculator also compute and that can be obtained via
 * {@link #getBiasSpatialForce()}.
 * <p>
 * Note on kinematic loops: the computed centroidal momentum matrix will be filled of zeros for the
 * loop closure joints. By externally constraining the configuration, velocity, and acceleration of
 * the joints composing a kinematic loop, the results from this calculator will remain accurate.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class CentroidalMomentumRateCalculator implements ReferenceFrameHolder
{
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;
   /** The center of mass centroidal momentum matrix. */
   private final ReferenceFrame matrixFrame;
   /** The root of the internal recursive algorithm. */
   private final RecursionStep initialRecursionStep;
   /**
    * Only the first pass of this algorithm has to be recursive, the rest can be iterative which can
    * provide a slight performance improvement.
    */
   private final RecursionStep[] recursionSteps;
   /** Intermediate variable to store the unit-twist of the parent joint. */
   private final Twist jointUnitTwist;
   /** Intermediate variable for garbage free operations. */
   private final Twist intermediateTwist;
   /** Intermediate variable to store one column of the centroidal momentum matrix. */
   private final Momentum unitMomentum;
   /** Intermediate variable for garbage free operations. */
   private final Momentum intermediateMomentum;
   /**
    * Intermediate variable to store the wrench resulting from Coriolis and centrifugal accelerations.
    */
   private final Wrench netCoriolisBodyWrench;

   /** The centroidal momentum matrix. */
   private final DMatrixRMaj centroidalMomentumMatrix;
   /**
    * The convective term resulting from the Coriolis and centrifugal forces acting on the system.
    */
   private final SpatialForce biasSpatialForce;
   /** The total momentum of the system. */
   private final FixedFrameMomentumBasics momentum;
   /** The total rate of change of momentum of the system. */
   private final FixedFrameSpatialForceBasics momentumRate;
   /** The center of mass velocity. */
   private final FixedFrameVector3DBasics centerOfMassVelocity;
   /** The center of mass acceleration. */
   private final FixedFrameVector3DBasics centerOfMassAcceleration;

   /**
    * The convective term resulting from the Coriolis and centrifugal forces acting on the system.
    */
   private final DMatrixRMaj biasSpatialForceMatrix = new DMatrixRMaj(6, 1);
   /** Matrix containing the velocities of the joints to consider. */
   private final DMatrixRMaj jointVelocityMatrix;
   /** Matrix containing the accelerations of the joints to consider. */
   private final DMatrixRMaj jointAccelerationMatrix;
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
   /**
    * Whether the joint acceleration matrix has been updated since the last call to {@link #reset()}.
    */
   private boolean isJointAccelerationMatrixUpToDate = false;
   /** Whether the momentum has been updated since the last call to {@link #reset()}. */
   private boolean isMomentumUpToDate = false;
   /**
    * Whether the rate of change of momentum has been updated since the last call to {@link #reset()}.
    */
   private boolean isMomentumRateUpToDate = false;
   /** Whether the total mass has been updated since the last call to {@link #reset()}. */
   private boolean isTotalMassUpToDate = false;
   /**
    * Whether the center of mass velocity has been updated since the last call to {@link #reset()}.
    */
   private boolean isCenterOfMassVelocityUpToDate = false;
   /**
    * Whether the center of mass acceleration has been updated since the last call to {@link #reset()}.
    */
   private boolean isCenterOfMassAccelerationUpToDate = false;

   /**
    * Creates a new calculator for the subtree that starts off the given {@code rootBody}.
    *
    * @param rootBody    the start of subtree for which the centroidal momentum matrix and bias force
    *                    is to be computed. Not modified.
    * @param matrixFrame the frame in which the centroidal momentum matrix is to be expressed.
    */
   public CentroidalMomentumRateCalculator(RigidBodyReadOnly rootBody, ReferenceFrame matrixFrame)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), matrixFrame);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input       the definition of the system to be evaluated by this calculator.
    * @param matrixFrame the frame in which the centroidal momentum matrix is to be expressed.
    */
   public CentroidalMomentumRateCalculator(MultiBodySystemReadOnly input, ReferenceFrame matrixFrame)
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
   public CentroidalMomentumRateCalculator(MultiBodySystemReadOnly input, ReferenceFrame matrixFrame, boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;
      this.matrixFrame = matrixFrame;

      RigidBodyReadOnly rootBody = input.getRootBody();
      initialRecursionStep = new RecursionStep(rootBody, null, null);
      recursionSteps = buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore()).toArray(new RecursionStep[0]);

      if (considerIgnoredSubtreesInertia)
         initialRecursionStep.includeIgnoredSubtreeInertia();

      biasSpatialForce = new SpatialForce(matrixFrame);

      jointUnitTwist = new Twist();
      intermediateTwist = new Twist();
      unitMomentum = new Momentum(matrixFrame);
      intermediateMomentum = new Momentum();
      netCoriolisBodyWrench = new Wrench();

      int nDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      centroidalMomentumMatrix = new DMatrixRMaj(6, nDegreesOfFreedom);
      jointVelocityMatrix = new DMatrixRMaj(nDegreesOfFreedom, 1);
      jointAccelerationMatrix = new DMatrixRMaj(nDegreesOfFreedom, 1);

      momentum = new Momentum(matrixFrame);
      momentumRate = new SpatialForce(matrixFrame);
      centerOfMassVelocity = new FrameVector3D(matrixFrame);
      centerOfMassAcceleration = new FrameVector3D(matrixFrame);
   }

   private List<RecursionStep> buildMultiBodyTree(RecursionStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<RecursionStep> recursionSteps = new ArrayList<>();
      recursionSteps.add(parent);

      for (JointReadOnly childJoint : parent.rigidBody.getChildrenJoints())
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
            RecursionStep child = new RecursionStep(childBody, parent, jointIndices);
            recursionSteps.addAll(buildMultiBodyTree(child, jointsToIgnore));
         }
      }

      return recursionSteps;
   }

   /**
    * Invalidates the internal memory.
    */
   public void reset()
   {
      isCentroidalMomentumUpToDate = false;
      isJointVelocityMatrixUpToDate = false;
      isJointAccelerationMatrixUpToDate = false;
      isMomentumUpToDate = false;
      isMomentumRateUpToDate = false;
      isTotalMassUpToDate = false;
      isCenterOfMassVelocityUpToDate = false;
      isCenterOfMassAccelerationUpToDate = false;
   }

   private void updateCentroidalMomentum()
   {
      if (isCentroidalMomentumUpToDate)
         return;

      biasSpatialForce.setToZero();
      passOne(initialRecursionStep);
      passTwo();
      biasSpatialForce.get(biasSpatialForceMatrix);
      isCentroidalMomentumUpToDate = true;
   }

   /**
    * Recursive method that calls {@link RecursionStep#passOne()} starting from the root.
    *
    * @param current the current recursion step.
    * @see RecursionStep#passOne()
    */
   private void passOne(RecursionStep current)
   {
      current.passOne();

      for (int childIndex = 0; childIndex < current.children.size(); childIndex++)
      {
         RecursionStep child = current.children.get(childIndex);
         passOne(child);
      }
   }

   /**
    * Iterative method that calls {@link RecursionStep#passTwo()}.
    *
    * @see RecursionStep#passTwo()
    */
   private void passTwo()
   {
      for (RecursionStep recursionStep : recursionSteps)
         recursionStep.passTwo();
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

   private DMatrixRMaj getJointAccelerationMatrix()
   {
      if (!isJointAccelerationMatrixUpToDate)
      {
         List<? extends JointReadOnly> joints = input.getJointMatrixIndexProvider().getIndexedJointsInOrder();
         MultiBodySystemTools.extractJointsState(joints, JointStateType.ACCELERATION, jointAccelerationMatrix);
         isJointAccelerationMatrixUpToDate = true;
      }

      return jointAccelerationMatrix;
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
    * Gets the center of mass acceleration of the multi-body system.
    *
    * @return the center of mass acceleration.
    */
   public FrameVector3DReadOnly getCenterOfMassAcceleration()
   {
      if (!isCenterOfMassAccelerationUpToDate)
      {
         centerOfMassAcceleration.setAndScale(1.0 / getTotalMass(), getMomentumRate().getLinearPart());
         isCenterOfMassAccelerationUpToDate = true;
      }

      return centerOfMassAcceleration;
   }

   /**
    * Computes and packs the center of mass acceleration for the given joint accelerations.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    *
    * @param jointAccelerationMatrix        the matrix containing the joint accelerations to use. Not
    *                                       modified.
    * @param centerOfMassAccelerationToPack the vector used to stored the computed center of mass
    *                                       acceleration. Modified.
    */
   public void getCenterOfMassAcceleration(DMatrixRMaj jointAccelerationMatrix, FrameVector3DBasics centerOfMassAccelerationToPack)
   {
      CommonOps_DDRM.mult(getCentroidalMomentumMatrix(), jointAccelerationMatrix, momentumMatrix);
      CommonOps_DDRM.addEquals(momentumMatrix, getBiasSpatialForceMatrix());
      centerOfMassAccelerationToPack.setIncludingFrame(matrixFrame, 3, momentumMatrix);
      centerOfMassAccelerationToPack.scale(1.0 / getTotalMass());
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
    * Gets the rate of change of momentum of the multi-body system.
    *
    * @return the rate of change of momentum.
    */
   public SpatialForceReadOnly getMomentumRate()
   {
      if (!isMomentumRateUpToDate)
      {
         CommonOps_DDRM.mult(getCentroidalMomentumMatrix(), getJointAccelerationMatrix(), momentumMatrix);
         momentumRate.set(momentumMatrix);
         momentumRate.add(getBiasSpatialForce());
         isMomentumRateUpToDate = true;
      }
      return momentumRate;
   }

   /**
    * Computes and packs the rate of change of momentum for the given joint accelerations.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    *
    * @param jointAccelerationMatrix the matrix containing the joint accelerations to use. Not
    *                                modified.
    * @param momentumRateToPack      the vector used to stored the computed rate of change of momentum.
    *                                Modified.
    */
   public void getMomentumRate(DMatrix1Row jointAccelerationMatrix, SpatialForceBasics momentumRateToPack)
   {
      CommonOps_DDRM.mult(getCentroidalMomentumMatrix(), jointAccelerationMatrix, momentumMatrix);
      CommonOps_DDRM.addEquals(momentumMatrix, getBiasSpatialForceMatrix());
      momentumRateToPack.setIncludingFrame(matrixFrame, momentumMatrix);
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
    * @see CentroidalMomentumRateCalculator
    */
   public DMatrixRMaj getCentroidalMomentumMatrix()
   {
      updateCentroidalMomentum();
      return centroidalMomentumMatrix;
   }

   /**
    * Gets the convective term resulting from the Coriolis and centrifugal forces acting on the system.
    *
    * @return the bias spatial force.
    * @see CentroidalMomentumRateCalculator
    */
   public SpatialForceReadOnly getBiasSpatialForce()
   {
      updateCentroidalMomentum();
      return biasSpatialForce;
   }

   /**
    * Gets the convective term resulting from the Coriolis and centrifugal forces acting on the system.
    *
    * @return the bias spatial force.
    * @see CentroidalMomentumRateCalculator
    */
   public DMatrixRMaj getBiasSpatialForceMatrix()
   {
      updateCentroidalMomentum();
      return biasSpatialForceMatrix;
   }

   /**
    * The reference frame in which the centroidal momentum matrix and convective are computed.
    *
    * @return this calculator's reference frame.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return matrixFrame;
   }

   /**
    * Represents a single recursion step with all the intermediate variables needed.
    *
    * @author Sylvain Bertrand
    */
   private class RecursionStep
   {
      /** The rigid-body for which this recursion is. */
      private final RigidBodyReadOnly rigidBody;
      /**
       * Body inertia: usually equal to {@code rigidBody.getInertial()}. However, if at least one child of
       * {@code rigidBody} is ignored, it is equal to this rigid-body inertia and the subtree inertia
       * attached to the ignored joint.
       */
      private final SpatialInertia bodyInertia;
      /**
       * Result of this recursion step: the matrix block of the centroidal momentum matrix for the parent
       * joint.
       */
      private final DMatrixRMaj centroidalMomentumMatrixBlock;
      /** The Coriolis and centrifugal accelerations for this rigid-body. */
      private final SpatialAcceleration coriolisBodyAcceleration;
      /**
       * Intermediate variable to prevent repetitive calculation of a transform between this rigid-body's
       * body-fixed frame and the matrix frame.
       */
      private final RigidBodyTransform matrixFrameToBodyFixedFrameTransform;
      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid-body.
       */
      private final RecursionStep parent;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      private final List<RecursionStep> children = new ArrayList<>();
      /**
       * Joint indices for storing {@code centroidalMomentumMatrixBlock} in the main matrix
       * {@code centroidalMomentumMatrix}.
       */
      private final int[] jointIndices;

      public RecursionStep(RigidBodyReadOnly rigidBody, RecursionStep parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;

         if (isRoot())
         {
            bodyInertia = null;
            coriolisBodyAcceleration = new SpatialAcceleration(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());
            centroidalMomentumMatrixBlock = null;
            matrixFrameToBodyFixedFrameTransform = null;
         }
         else
         {
            parent.children.add(this);
            bodyInertia = new SpatialInertia(rigidBody.getInertia());
            coriolisBodyAcceleration = new SpatialAcceleration();
            centroidalMomentumMatrixBlock = new DMatrixRMaj(6, getJoint().getDegreesOfFreedom());
            matrixFrameToBodyFixedFrameTransform = new RigidBodyTransform();
         }
      }

      public void includeIgnoredSubtreeInertia()
      {
         if (!isRoot() && children.size() != rigidBody.getChildrenJoints().size())
         {
            for (JointReadOnly childJoint : rigidBody.getChildrenJoints())
            {
               if (input.getJointsToIgnore().contains(childJoint))
               {
                  SpatialInertia subtreeIneria = MultiBodySystemTools.computeSubtreeInertia(childJoint);
                  subtreeIneria.changeFrame(getBodyFixedFrame());
                  bodyInertia.add(subtreeIneria);
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).includeIgnoredSubtreeInertia();
      }

      /**
       * First pass going from the root to the leaves.
       * <p>
       * Here the rigid-body Coriolis and centrifugal accelerations are updated to calculate the system's
       * bias force..
       * </p>
       */
      public void passOne()
      {
         if (isRoot())
            return;

         ReferenceFrame inertiaFrame = bodyInertia.getReferenceFrame();
         matrixFrame.getTransformToDesiredFrame(matrixFrameToBodyFixedFrameTransform, inertiaFrame);

         coriolisBodyAcceleration.setIncludingFrame(parent.coriolisBodyAcceleration);
         getJoint().getPredecessorTwist(intermediateTwist);
         coriolisBodyAcceleration.changeFrame(getBodyFixedFrame(), intermediateTwist, parent.getBodyFixedFrame().getTwistOfFrame());
         coriolisBodyAcceleration.setBodyFrame(getBodyFixedFrame());

         bodyInertia.computeDynamicWrench(coriolisBodyAcceleration, getBodyFixedFrame().getTwistOfFrame(), netCoriolisBodyWrench);
         netCoriolisBodyWrench.applyInverseTransform(matrixFrameToBodyFixedFrameTransform);
         netCoriolisBodyWrench.setReferenceFrame(matrixFrame);
         biasSpatialForce.add(netCoriolisBodyWrench);
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

         for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
         {
            unitMomentum.setToZero();
            jointUnitTwist.setIncludingFrame(getJoint().getUnitTwists().get(dofIndex));
            jointUnitTwist.changeFrame(matrixFrame);
            addToUnitMomentumRecursively(jointUnitTwist, unitMomentum);
            unitMomentum.get(0, dofIndex, centroidalMomentumMatrixBlock);

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
         SpatialInertiaReadOnly inertia = rigidBody.getInertia();

         ReferenceFrame inertiaFrame = inertia.getReferenceFrame();

         intermediateTwist.setIncludingFrame(ancestorUnitTwist);
         intermediateTwist.applyTransform(matrixFrameToBodyFixedFrameTransform);
         intermediateTwist.setReferenceFrame(inertiaFrame);

         intermediateMomentum.setReferenceFrame(inertiaFrame);
         intermediateMomentum.compute(inertia, intermediateTwist);
         intermediateMomentum.applyInverseTransform(matrixFrameToBodyFixedFrameTransform);
         intermediateMomentum.setReferenceFrame(matrixFrame);

         unitMomentumToAddTo.add(intermediateMomentum);

         for (int i = 0; i < children.size(); i++)
            children.get(i).addToUnitMomentumRecursively(ancestorUnitTwist, unitMomentumToAddTo);
      }

      public boolean isRoot()
      {
         return parent == null;
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
