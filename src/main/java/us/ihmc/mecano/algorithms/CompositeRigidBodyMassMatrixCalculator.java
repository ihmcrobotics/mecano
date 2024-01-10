package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.*;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * This calculator computes:
 * <ul>
 * <li>the mass matrix of a multi-body system that maps from joint acceleration space to joint
 * effort space.
 * <li>the Coriolis and centrifugal matrix of a multi-body system that maps from joint velocity
 * space to joint effort space.
 * <li>the centroidal momentum matrix that maps from joint velocity space to whole-body momentum
 * space.
 * <li>the centroidal momentum convective term which represents the bias in momentum rate resulting
 * from Coriolis and centrifugal accelerations.
 * </ul>
 * <p>
 * This calculator is based on:
 * <ul>
 * <li>the composite-rigid-body algorithm, as described in Featherstone - Rigid Body Dynamics
 * Algorithms (2008): <a href=
 * "https://books.google.com/books?id=GJRGBQAAQBAJ&lpg=PR5&ots=XoFXvnJZLH&dq=rigid%20body%20dynamics%20algorithms&lr&pg=PR1#v=onepage&q=rigid%20body%20dynamics%20algorithms&f=false">link</a>
 * <li>the paper: <i>Numerical Methods to Compute the Coriolis Matrix and Christoffel Symbols for
 * Rigid-Body Systems</i> by Sebastian Echeandia and Patrick M. Wensing.
 * </ul>
 * </p>
 * <p>
 * The mass matrix can be used in the equations of motion for a multi-body system as follows:
 * </p>
 *
 * <pre>
 * &tau; = H(q) qDDot + C(q, qDot) qDot + G(q) + &sum;<sub>i</sub> J<sub>i</sub> W<sub>i, ext</sub>
 * </pre>
 *
 * where <tt>&tau;</tt>, <tt>q</tt>, <tt>qDot</tt>, and <tt>qDDot</tt> are the joint effort,
 * configuration, velocity, and acceleration vectors, <tt>H</tt> is the joint-space inertia matrix
 * or also called here mass matrix, <tt>C</tt> is the Coriolis and centrifugal matrix, <tt>G</tt>
 * the joint efforts resulting from gravity, and
 * <tt>&sum;<sub>i</sub> J<sub>i</sub> W<sub>i, ext</sub></tt> the joint efforts resulting from
 * external wrenches.
 * <p>
 * In addition to computing the mass matrix of a multi-body system, this calculator can efficiently
 * compute the centroidal momentum matrix and the centroidal momentum convective term. Such that,
 * when both the centroidal momentum and mass matrix are needed, it is more efficient to use this
 * calculator for computing both, when only the centroidal momentum information is need though,
 * prefer using {@link CentroidalMomentumRateCalculator}.
 * </p>
 * <p>
 * Centroidal momentum equations:
 *
 * <pre>
 * h  = A * qDot
 * dh               dA
 * -- = A * qDDot + -- * qDot = A * qDDot + b
 * dt               dt
 * </pre>
 *
 * where <tt>h</tt> is the system's momentum and <tt>A</tt> is the centroidal momentum matrix, the
 * term introduce <tt>b</tt> represents the convective term.
 * </p>
 * <p>
 * Note on kinematic loops: the computed mass matrix will be filled of zeros for the loop closure
 * joints. By externally constraining the configuration and velocity of the joints composing a
 * kinematic loop, the results from this calculator will remain accurate.
 * </p>
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public class CompositeRigidBodyMassMatrixCalculator
{
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;

   /** The root of the internal recursive algorithm. */
   private final CompositeRigidBodyInertia rootCompositeInertia;
   /**
    * Array of all the recursion steps but the root. This allows to efficiently iterate through when
    * computing the centroidal momentum matrix.
    */
   private final CompositeRigidBodyInertia[] compositeInertias;
   /** The mass matrix of the system. */
   private final DMatrixRMaj massMatrix;
   private final DMatrixRMaj coriolisMatrix;
   /** Intermediate variable to store the child inertia. */
   private final SpatialInertia childInertia = new SpatialInertia();
   /** Intermediate variable to store the child factorized inertia. */
   private final FactorizedBodyInertia childFactorizedInertia = new FactorizedBodyInertia();

   /** Intermediate variable for garbage free operations. */
   private final Twist intermediateTwist = new Twist();
   /**
    * Intermediate variable to store the wrench resulting from Coriolis and centrifugal accelerations.
    */
   private final Wrench netCoriolisBodyWrench = new Wrench();
   /**
    * The convective term resulting from the Coriolis and centrifugal forces acting on the system. This
    * is useful in combination of the centroidal momentum matrix to build the mapping from joint
    * acceleration space to rate of change of momentum space.
    */
   private final SpatialForce centroidalConvectiveTerm = new SpatialForce();
   /**
    * The convective term resulting from the Coriolis and centrifugal forces acting on the system. This
    * is useful in combination of the centroidal momentum matrix to build the mapping from joint
    * acceleration space to rate of change of momentum space.
    */
   private final DMatrixRMaj centroidalConvectiveTermMatrix = new DMatrixRMaj(6, 1);

   /** Frame in which the centroidal momentum matrix and its convective term are to be calculated. */
   private ReferenceFrame centroidalMomentumFrame;
   /**
    * The centroidal momentum matrix represents the map from joint velocity space to momentum space.
    * Used in combination with the convective term allows to make the mapping from joint acceleration
    * space to rate of change of momentum space.
    */
   private final DMatrixRMaj centroidalMomentumMatrix;

   /**
    * Whether the mass matrix has been updated since the last call to {@link #reset()}.
    */
   private boolean isMassMatrixUpToDate = false;
   /**
    * Whether the Coriolis matrix should be computed when updating the mass matrix.
    */
   private boolean enableCoriolisMatrixCalculation = false;
   /**
    * Whether the centroidal momentum matrix has been updated since the last call to {@link #reset()}.
    */
   private boolean isCentroidalMomentumMatrixUpToDate = false;
   /**
    * Whether the centroidal momentum convective term has been updated since the last call to
    * {@link #reset()}.
    */
   private boolean isCentroidalConvectiveTermUpToDate = false;

   /**
    * Creates a new calculator for the subtree that starts off the given {@code rootBody}.
    *
    * @param rootBody the start of subtree for which the mass matrix is to be computed. Not modified.
    */
   public CompositeRigidBodyMassMatrixCalculator(RigidBodyReadOnly rootBody)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody));
   }

   /**
    * Creates a new calculator for the subtree that starts off the given {@code rootBody}.
    *
    * @param rootBody                the start of subtree for which the mass matrix is to be computed.
    *                                Not modified.
    * @param centroidalMomentumFrame the reference frame in which the centroidal momentum matrix and
    *                                convective term are to be computed. Only needed when this
    *                                calculator is used to calculate the centroidal momentum matrix and
    *                                convective term.
    */
   public CompositeRigidBodyMassMatrixCalculator(RigidBodyReadOnly rootBody, ReferenceFrame centroidalMomentumFrame)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), centroidalMomentumFrame);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input the definition of the system to be evaluated by this calculator.
    */
   public CompositeRigidBodyMassMatrixCalculator(MultiBodySystemReadOnly input)
   {
      this(input, input.getRootBody().getBodyFixedFrame());
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input                   the definition of the system to be evaluated by this calculator.
    * @param centroidalMomentumFrame the reference frame in which the centroidal momentum matrix and
    *                                convective term are to be computed. Only needed when this
    *                                calculator is used to calculate the centroidal momentum matrix and
    *                                convective term.
    */
   public CompositeRigidBodyMassMatrixCalculator(MultiBodySystemReadOnly input, ReferenceFrame centroidalMomentumFrame)
   {
      this(input, centroidalMomentumFrame, true);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input                          the definition of the system to be evaluated by this
    *                                       calculator.
    * @param centroidalMomentumFrame        the reference frame in which the centroidal momentum matrix
    *                                       and convective term are to be computed. Only needed when
    *                                       this calculator is used to calculate the centroidal
    *                                       momentum matrix and convective term.
    * @param considerIgnoredSubtreesInertia whether the inertia of the ignored part(s) of the given
    *                                       multi-body system should be considered. When {@code true},
    *                                       this provides a more accurate mass matrix, while when
    *                                       {@code false}, this calculator may gain slight performance
    *                                       improvement.
    */
   public CompositeRigidBodyMassMatrixCalculator(MultiBodySystemReadOnly input, ReferenceFrame centroidalMomentumFrame, boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;

      RigidBodyReadOnly rootBody = input.getRootBody();
      rootCompositeInertia = new CompositeRigidBodyInertia(rootBody, null, null);
      compositeInertias = buildMultiBodyTree(rootCompositeInertia, input.getJointsToIgnore()).toArray(new CompositeRigidBodyInertia[0]);

      if (considerIgnoredSubtreesInertia)
         rootCompositeInertia.includeIgnoredSubtreeInertia();

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      massMatrix = new DMatrixRMaj(nDoFs, nDoFs);
      coriolisMatrix = new DMatrixRMaj(nDoFs, nDoFs);

      centroidalMomentumMatrix = new DMatrixRMaj(6, nDoFs);
      setCentroidalMomentumFrame(centroidalMomentumFrame);
   }

   private List<CompositeRigidBodyInertia> buildMultiBodyTree(CompositeRigidBodyInertia parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<CompositeRigidBodyInertia> inertiaList = new ArrayList<>();

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
            CompositeRigidBodyInertia child = new CompositeRigidBodyInertia(childBody, parent, jointIndices);
            inertiaList.add(child);
            inertiaList.addAll(buildMultiBodyTree(child, jointsToIgnore));
         }
      }
      return inertiaList;
   }

   /**
    * Enables/disables the calculation of the Coriolis and centrifugal matrix.
    * <p>
    * Note that enabling the calculation of the Coriolis and centrifugal matrix will increase the
    * computational load when updating the mass matrix.
    * </p>
    *
    * @param enableCoriolisMatrixCalculation whether to enable or disable the calculation of the
    *                                        Coriolis and centrifugal matrix. Disabled by default.
    */
   public void setEnableCoriolisMatrixCalculation(boolean enableCoriolisMatrixCalculation)
   {
      this.enableCoriolisMatrixCalculation = enableCoriolisMatrixCalculation;
   }

   /**
    * Invalidates the internal memory.
    */
   public void reset()
   {
      isMassMatrixUpToDate = false;
      isCentroidalMomentumMatrixUpToDate = false;
      isCentroidalConvectiveTermUpToDate = false;
   }

   private void updateMassMatrix()
   {
      if (isMassMatrixUpToDate)
         return;

      massMatrix.zero();
      if (enableCoriolisMatrixCalculation)
         coriolisMatrix.zero();
      rootCompositeInertia.computeMassMatrix();
      isMassMatrixUpToDate = true;
   }

   private void updateCentroidalMomentumMatrix()
   {
      if (isCentroidalMomentumMatrixUpToDate)
         return;

      updateMassMatrix();

      for (CompositeRigidBodyInertia compositeInertia : compositeInertias)
         compositeInertia.computeCentroidalMomentumMatrix();

      isCentroidalMomentumMatrixUpToDate = true;
   }

   private void updateCentroidalConvectiveTerm()
   {
      if (isCentroidalConvectiveTermUpToDate)
         return;

      centroidalConvectiveTerm.setToZero(centroidalMomentumFrame);
      rootCompositeInertia.computeCentroidalConvectiveTerm();
      centroidalConvectiveTerm.get(centroidalConvectiveTermMatrix);
      isCentroidalConvectiveTermUpToDate = true;
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
    * Gets the mass-matrix for this multi-body system.
    *
    * @return the mass-matrix.
    */
   public DMatrixRMaj getMassMatrix()
   {
      updateMassMatrix();
      return massMatrix;
   }

   /**
    * Gets the Coriolis and centrifugal matrix for this multi-body system.
    *
    * @return the Coriolis and centrifugal matrix.
    * @throws UnsupportedOperationException if the calculation of the Coriolis and centrifugal matrix
    *       was not enabled.
    * @see #setEnableCoriolisMatrixCalculation(boolean)
    */
   public DMatrixRMaj getCoriolisMatrix()
   {
      if (!enableCoriolisMatrixCalculation)
         throw new UnsupportedOperationException("Coriolis matrix calculation is disabled.");

      updateMassMatrix();
      return coriolisMatrix;
   }

   /**
    * Sets the new reference frame in which the centroidal momentum matrix and convective term are to
    * be calculated.
    * <p>
    * The internal memory is invalidated when changing the frame and it will be updated next time the
    * information is being accessed.
    * </p>
    *
    * @param centroidalMomentumFrame the reference frame in which the centroidal momentum matrix and
    *                                convective term are to be computed. Only needed when this
    *                                calculator is used to calculate the centroidal momentum matrix and
    *                                convective term.
    */
   public void setCentroidalMomentumFrame(ReferenceFrame centroidalMomentumFrame)
   {
      if (centroidalMomentumFrame == this.centroidalMomentumFrame)
         return;
      this.centroidalMomentumFrame = centroidalMomentumFrame;
      isCentroidalMomentumMatrixUpToDate = false;
      isCentroidalConvectiveTermUpToDate = false;
   }

   /**
    * Gets the reference frame in which the centroidal momentum matrix and convective are expressed.
    *
    * @return the centroidal momentum frame.
    */
   public ReferenceFrame getCentroidalMomentumFrame()
   {
      return centroidalMomentumFrame;
   }

   /**
    * Gets the N-by-6 centroidal momentum matrix, where N is the number of degrees of freedom of the
    * multi-body system.
    * <p>
    * The centroidal momentum matrix maps from joint velocity space to momentum space and is expressed
    * in the frame {@link #getCentroidalMomentumFrame()}. The latter implies that when multiplied to
    * the joint velocity matrix, the result is the momentum expressed in
    * {@link #getCentroidalMomentumFrame()}.
    * </p>
    *
    * @return the centroidal momentum matrix.
    * @see CompositeRigidBodyMassMatrixCalculator
    */
   public DMatrixRMaj getCentroidalMomentumMatrix()
   {
      updateCentroidalMomentumMatrix();
      return centroidalMomentumMatrix;
   }

   /**
    * Gets the convective term resulting from the Coriolis and centrifugal forces acting on the system.
    *
    * @return the bias spatial force.
    * @see CompositeRigidBodyMassMatrixCalculator
    */
   public SpatialForceReadOnly getCentroidalConvectiveTerm()
   {
      updateCentroidalConvectiveTerm();
      return centroidalConvectiveTerm;
   }

   /**
    * Gets the convective term resulting from the Coriolis and centrifugal forces acting on the system.
    *
    * @return the bias spatial force.
    * @see CompositeRigidBodyMassMatrixCalculator
    */
   public DMatrixRMaj getCentroidalConvectiveTermMatrix()
   {
      updateCentroidalConvectiveTerm();
      return centroidalConvectiveTermMatrix;
   }

   /**
    * Represents a single iteration step with all the intermediate variables needed.
    *
    * @author Sylvain Bertrand
    */
   private class CompositeRigidBodyInertia
   {
      /**
       * The rigid-body for which this recursion is.
       */
      private final RigidBodyReadOnly rigidBody;
      /**
       * Body inertia: usually equal to {@code rigidBody.getInertial()}. However, if at least one child of
       * {@code rigidBody} is ignored, it is equal to this rigid-body inertia and the subtree inertia
       * attached to the ignored joint.
       */
      private final SpatialInertia bodyInertia, bodyInertiaAtJointFrame;
      /**
       * The recursion step holding onto the direct predecessor of this recursion step's rigid-body.
       */
      private final CompositeRigidBodyInertia parent;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      private final List<CompositeRigidBodyInertia> children = new ArrayList<>();
      /**
       * Joint indices for storing this joint's coefficients in the main matrix {@code massMatrix}.
       */
      private final int[] jointIndices;

      /** Spatial inertia representing the subtree starting off this rigid-body. */
      private final SpatialInertia compositeInertia;
      /** Spatial factorized inertia representing the subtree starting off this rigid-body. */
      private final FactorizedBodyInertia compositeFactorizedInertia;

      /**
       * Unit-momentum for each degree of freedom for this parent joint, it is computed from the product
       * of the unit-twist and the subtree composite inertia.
       */
      private final Momentum[] F1, F2, F3;
      private final DMatrixRMaj motionSubspaceDot;
      /** This parent joint unit-twists. */
      private final Twist[] unitTwists;
      /** The time-derivative of this parent joint unit-twists. */
      private final SpatialAcceleration[] unitTwistDots;

      /** The Coriolis and centrifugal accelerations for this rigid-body. */
      private final SpatialAcceleration coriolisBodyAcceleration;
      /**
       * Transform from {@code this.getFrameAfterJoint()} to {@code parent.getFrameAfterJoint()} used to
       * reduce computation load.
       */
      private final RigidBodyTransform transformToParent = new RigidBodyTransform();

      public CompositeRigidBodyInertia(RigidBodyReadOnly rigidBody, CompositeRigidBodyInertia parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;

         if (parent == null)
         {
            bodyInertia = null;
            bodyInertiaAtJointFrame = null;
            compositeInertia = null;
            compositeFactorizedInertia = null;
            motionSubspaceDot = null;
            F1 = null;
            F2 = null;
            F3 = null;
            unitTwists = null;
            unitTwistDots = null;
            coriolisBodyAcceleration = new SpatialAcceleration(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());
         }
         else
         {
            parent.children.add(this);

            int nDoFs = getNumberOfDoFs();
            bodyInertia = new SpatialInertia(rigidBody.getInertia());
            bodyInertiaAtJointFrame = new SpatialInertia(rigidBody.getInertia());
            bodyInertiaAtJointFrame.changeFrame(getFrameAfterJoint());
            compositeInertia = new SpatialInertia();
            compositeFactorizedInertia = new FactorizedBodyInertia();
            motionSubspaceDot = getJoint().isMotionSubspaceVariable() ? new DMatrixRMaj(6, nDoFs) : null;
            F1 = new Momentum[nDoFs];
            F2 = new Momentum[nDoFs];
            F3 = new Momentum[nDoFs];
            unitTwists = new Twist[nDoFs];
            unitTwistDots = new SpatialAcceleration[nDoFs];

            for (int i = 0; i < nDoFs; i++)
            {
               F1[i] = new Momentum();
               F2[i] = new Momentum();
               F3[i] = new Momentum();
               unitTwists[i] = new Twist();
               unitTwistDots[i] = new SpatialAcceleration();

               if (!getJoint().isMotionSubspaceVariable())
               {
                  unitTwists[i].setIncludingFrame(getJoint().getUnitTwists().get(i));
                  unitTwists[i].changeFrame(getFrameAfterJoint());
               }
            }
            coriolisBodyAcceleration = new SpatialAcceleration();
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
                  subtreeIneria.changeFrame(getFrameAfterJoint());
                  bodyInertiaAtJointFrame.add(subtreeIneria);
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).includeIgnoredSubtreeInertia();
      }

      /**
       * Recursion step to compute the mass-matrix. The recursion is from the leaves to the root.
       */
      public void computeMassMatrix()
      {
         if (!isRoot())
         {
            if (!parent.isRoot())
               getFrameAfterJoint().getTransformToDesiredFrame(transformToParent, parent.getFrameAfterJoint());

            if (getJoint().isMotionSubspaceVariable())
            {
               for (int i = 0; i < getNumberOfDoFs(); i++)
               {
                  unitTwists[i].setIncludingFrame(getJoint().getUnitTwists().get(i));
                  unitTwists[i].changeFrame(getFrameAfterJoint());
               }
            }

            if (enableCoriolisMatrixCalculation)
            {
               if (getJoint().isMotionSubspaceVariable())
                  getJoint().getMotionSubspaceDot(motionSubspaceDot);

               for (int i = 0; i < getNumberOfDoFs(); i++)
               {
                  unitTwistDots[i].setReferenceFrame(getFrameAfterJoint());
                  unitTwistDots[i].setBodyFrame(unitTwists[i].getBodyFrame());
                  unitTwistDots[i].setBaseFrame(unitTwists[i].getBaseFrame());
                  TwistReadOnly bodyTwist = getFrameAfterJoint().getTwistOfFrame();

                  if (getJoint().isMotionSubspaceVariable())
                  {
                     unitTwistDots[i].set(0, i, motionSubspaceDot);
                     unitTwistDots[i].addCrossToAngularPart(bodyTwist.getAngularPart(), unitTwists[i].getAngularPart());
                     unitTwistDots[i].addCrossToLinearPart(bodyTwist.getLinearPart(), unitTwists[i].getAngularPart());
                     unitTwistDots[i].addCrossToLinearPart(bodyTwist.getAngularPart(), unitTwists[i].getLinearPart());
                  }
                  else
                  {
                     unitTwistDots[i].getAngularPart().cross(bodyTwist.getAngularPart(), unitTwists[i].getAngularPart());
                     unitTwistDots[i].getLinearPart().cross(bodyTwist.getLinearPart(), unitTwists[i].getAngularPart());
                     unitTwistDots[i].addCrossToLinearPart(bodyTwist.getAngularPart(), unitTwists[i].getLinearPart());
                  }
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).computeMassMatrix();

         if (isRoot())
            return;

         compositeInertia.setIncludingFrame(bodyInertiaAtJointFrame);

         for (int i = 0; i < children.size(); i++)
         {
            CompositeRigidBodyInertia child = children.get(i);
            childInertia.setIncludingFrame(child.compositeInertia);
            // Optimization for "childInertia.changeFrame(getFrameAfterJoint());"
            childInertia.applyTransform(child.transformToParent);
            childInertia.setReferenceFrame(getFrameAfterJoint());
            compositeInertia.add(childInertia);
         }

         for (int i = 0; i < getNumberOfDoFs(); i++)
         {
            F2[i].setReferenceFrame(getFrameAfterJoint());
            F2[i].compute(compositeInertia, unitTwists[i]);
         }

         if (enableCoriolisMatrixCalculation)
         {
            intermediateTwist.setIncludingFrame(getFrameAfterJoint().getTwistOfFrame());
            intermediateTwist.setBodyFrame(getBodyFixedFrame());
            compositeFactorizedInertia.setIncludingFrame(bodyInertiaAtJointFrame, intermediateTwist);

            for (int i = 0; i < children.size(); i++)
            {
               CompositeRigidBodyInertia child = children.get(i);
               childFactorizedInertia.setIncludingFrame(child.compositeFactorizedInertia);
               // Optimization for "childFactorizedInertia.changeFrame(getFrameAfterJoint());"
               childFactorizedInertia.applyTransform(child.transformToParent);
               childFactorizedInertia.setReferenceFrame(getFrameAfterJoint());
               compositeFactorizedInertia.add(childFactorizedInertia);
            }

            for (int i = 0; i < getNumberOfDoFs(); i++)
            {
               F1[i].setReferenceFrame(getFrameAfterJoint());
               compositeInertia.transform(unitTwistDots[i], F1[i]);
               compositeFactorizedInertia.addTransform(unitTwists[i], F1[i]);

               F3[i].setReferenceFrame(getFrameAfterJoint());
               compositeFactorizedInertia.transposeTransform(unitTwists[i], F3[i]);
            }
         }

         /*
          * Here we update the mass-matrix block only corresponding to this joint indices. If the joint is a
          * 1-DoF joint, the block has only 1 element, for 6-DoF joint, the block is a 6-by-6 matrix.
          */
         for (int i = 0; i < getNumberOfDoFs(); i++)
         {
            for (int j = 0; j < getNumberOfDoFs(); j++)
            {
               double massMatrix_ij = unitTwists[i].dot(F2[j]);
               setSymmetricEntry(jointIndices[i], jointIndices[j], massMatrix, massMatrix_ij);
            }
         }

         if (enableCoriolisMatrixCalculation)
         {
            for (int i = 0; i < getNumberOfDoFs(); i++)
            {
               for (int j = 0; j < getNumberOfDoFs(); j++)
               {
                  double coriolis_ij = unitTwists[i].dot(F1[j]);
                  coriolisMatrix.set(jointIndices[i], jointIndices[j], coriolis_ij);

                  if (i != j)
                  {
                     double coriolis_ji = unitTwistDots[i].dot(F2[j]) + unitTwists[i].dot(F3[j]);
                     coriolisMatrix.set(jointIndices[j], jointIndices[i], coriolis_ji);
                  }
               }
            }
         }

         /*
          * Going up to the root to update the cross components between this joint and its ancestors.
          */
         if (enableCoriolisMatrixCalculation)
         {
            CompositeRigidBodyInertia ancestor = parent;
            CompositeRigidBodyInertia previous = this;

            while (!ancestor.isRoot())
            {
               for (int j = 0; j < getNumberOfDoFs(); j++)
               {
                  int index_j = jointIndices[j];
                  Momentum F1j = F1[j];
                  Momentum F2j = F2[j];
                  Momentum F3j = F3[j];

                  // Optimization for "F*j.changeFrame(ancestor.getFrameAfterJoint())"
                  F1j.applyTransform(previous.transformToParent);
                  F2j.applyTransform(previous.transformToParent);
                  F3j.applyTransform(previous.transformToParent);
                  F1j.setReferenceFrame(ancestor.getFrameAfterJoint());
                  F2j.setReferenceFrame(ancestor.getFrameAfterJoint());
                  F3j.setReferenceFrame(ancestor.getFrameAfterJoint());

                  for (int i = 0; i < ancestor.getNumberOfDoFs(); i++)
                  {
                     int index_i = ancestor.jointIndices[i];

                     double massMatrix_ij = ancestor.unitTwists[i].dot(F2j);
                     setSymmetricEntry(index_i, index_j, massMatrix, massMatrix_ij);

                     double coriolis_ij = ancestor.unitTwists[i].dot(F1j);
                     double coriolis_ji = ancestor.unitTwistDots[i].dot(F2j) + ancestor.unitTwists[i].dot(F3j);
                     coriolisMatrix.set(index_i, index_j, coriolis_ij);
                     coriolisMatrix.set(index_j, index_i, coriolis_ji);
                  }
               }

               previous = ancestor;
               ancestor = ancestor.parent;
            }
         }
         else
         {
            CompositeRigidBodyInertia ancestor = parent;
            CompositeRigidBodyInertia previous = this;

            while (!ancestor.isRoot())
            {
               for (int j = 0; j < getNumberOfDoFs(); j++)
               {
                  int index_j = jointIndices[j];
                  Momentum F2j = F2[j];

                  // Optimization for "F2j.changeFrame(ancestor.getFrameAfterJoint())"
                  F2j.applyTransform(previous.transformToParent);
                  F2j.setReferenceFrame(previous.parent.getFrameAfterJoint());

                  for (int i = 0; i < ancestor.getNumberOfDoFs(); i++)
                  {
                     int index_i = ancestor.jointIndices[i];

                     double massMatrix_ij = ancestor.unitTwists[i].dot(F2j);
                     setSymmetricEntry(index_i, index_j, massMatrix, massMatrix_ij);
                  }
               }

               previous = ancestor;
               ancestor = ancestor.parent;
            }
         }
      }

      public void computeCentroidalMomentumMatrix()
      {
         for (int dofIndex = 0; dofIndex < getNumberOfDoFs(); dofIndex++)
         {
            Momentum unitMomentum = F2[dofIndex];
            unitMomentum.changeFrame(centroidalMomentumFrame);
            unitMomentum.get(0, jointIndices[dofIndex], centroidalMomentumMatrix);
         }
      }

      public void computeCentroidalConvectiveTerm()
      {
         if (!isRoot())
         {
            SpatialInertiaReadOnly inertia = bodyInertia;

            coriolisBodyAcceleration.setIncludingFrame(parent.coriolisBodyAcceleration);
            getJoint().getPredecessorTwist(intermediateTwist);
            coriolisBodyAcceleration.changeFrame(getBodyFixedFrame(), intermediateTwist, parent.getBodyFixedFrame().getTwistOfFrame());
            coriolisBodyAcceleration.setBodyFrame(getBodyFixedFrame());

            inertia.computeDynamicWrench(coriolisBodyAcceleration, getBodyFixedFrame().getTwistOfFrame(), netCoriolisBodyWrench);
            netCoriolisBodyWrench.changeFrame(centroidalMomentumFrame);
            centroidalConvectiveTerm.add(netCoriolisBodyWrench);
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).computeCentroidalConvectiveTerm();
      }

      public void setSymmetricEntry(int row, int col, DMatrix symmetricMatrix, double entry)
      {
         symmetricMatrix.set(row, col, entry);
         symmetricMatrix.set(col, row, entry);
      }

      private boolean isRoot()
      {
         return parent == null;
      }

      public int getNumberOfDoFs()
      {
         return getJoint().getDegreesOfFreedom();
      }

      private MovingReferenceFrame getFrameAfterJoint()
      {
         return getJoint().getFrameAfterJoint();
      }

      public MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }

      public JointReadOnly getJoint()
      {
         return rigidBody.getParentJoint();
      }

      @Override
      public String toString()
      {
         return rigidBody.toString();
      }
   }
}
