package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.IntStream;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Computes the mass matrix of a multi-body system that maps from joint acceleration space to joint
 * effort space.
 * <p>
 * This calculator is based on the composite-rigid-body algorithm, as described in Featherstone -
 * Rigid Body Dynamics Algorithms (2008): <a href=
 * "https://books.google.com/books?id=GJRGBQAAQBAJ&lpg=PR5&ots=XoFXvnJZLH&dq=rigid%20body%20dynamics%20algorithms&lr&pg=PR1#v=onepage&q=rigid%20body%20dynamics%20algorithms&f=false">link</a>
 * </p>
 * <p>
 * The mass matrix can be used in the equations of motion for a multi-body system as follows:
 * </p>
 *
 * <pre>
 * &tau; = H(q) qDDot + C(q, qDot, f<sub>ext</sub>)
 * </pre>
 *
 * where <tt>&tau;</tt>, <tt>q</tt>, <tt>qDot</tt>, and <tt>qDDot</tt> are the joint effort,
 * configuration, velocity, and acceleration vectors, <tt>H</tt> is the joint-space inertia matrix
 * or also called here mass matrix, and <tt>C</tt> it the joint-space bias force vector resulting
 * from gravity, Coriolis and centrifugal accelerations, and other external forces if any.
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
   /** Intermediate variable to store the child inertia. */
   private final SpatialInertia childInertia = new SpatialInertia();

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

      centroidalMomentumMatrix = new DMatrixRMaj(6, nDoFs);
      setCentroidalMomentumFrame(centroidalMomentumFrame);
   }

   private List<CompositeRigidBodyInertia> buildMultiBodyTree(CompositeRigidBodyInertia parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<CompositeRigidBodyInertia> inertiaList = new ArrayList<>();

      List<JointReadOnly> childrenJoints = new ArrayList<>(parent.rigidBody.getChildrenJoints());

      if (childrenJoints.size() > 1)
      { // Reorganize the joints in the children to ensure that loop closures are treated last.
         List<JointReadOnly> loopClosureAncestors = new ArrayList<>();

         for (int i = 0; i < childrenJoints.size();)
         {
            if (MultiBodySystemTools.doesSubtreeContainLoopClosure(childrenJoints.get(i).getSuccessor()))
               loopClosureAncestors.add(childrenJoints.remove(i));
            else
               i++;
         }

         childrenJoints.addAll(loopClosureAncestors);
      }

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
      private final SpatialInertia bodyInertia;
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

      /**
       * Unit-momentum for each degree of freedom for this parent joint, it is computed from the product
       * of the unit-twist and the subtree composite inertia.
       */
      private final Momentum[] unitMomenta;
      /** This parent joint unit-twists. */
      private final Twist[] unitTwists;

      /** The Coriolis and centrifugal accelerations for this rigid-body. */
      private final SpatialAcceleration coriolisBodyAcceleration;

      public CompositeRigidBodyInertia(RigidBodyReadOnly rigidBody, CompositeRigidBodyInertia parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;

         if (parent == null)
         {
            bodyInertia = null;
            unitMomenta = null;
            compositeInertia = null;
            unitTwists = null;
            coriolisBodyAcceleration = new SpatialAcceleration(getBodyFixedFrame(), input.getInertialFrame(), getBodyFixedFrame());
         }
         else
         {
            parent.children.add(this);

            int nDoFs = getNumberOfDoFs();
            bodyInertia = new SpatialInertia(rigidBody.getInertia());
            compositeInertia = new SpatialInertia();
            unitMomenta = IntStream.range(0, nDoFs).mapToObj(i -> new Momentum()).toArray(Momentum[]::new);
            unitTwists = new Twist[nDoFs];
            for (int i = 0; i < nDoFs; i++)
            {
               unitTwists[i] = new Twist(getJoint().getUnitTwists().get(i));
               unitTwists[i].changeFrame(getFrameAfterJoint());
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
         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).computeMassMatrix();

         if (isRoot())
            return;

         compositeInertia.setIncludingFrame(bodyInertia);
         compositeInertia.changeFrame(getFrameAfterJoint());

         for (int i = 0; i < children.size(); i++)
         {
            childInertia.setIncludingFrame(children.get(i).compositeInertia);
            childInertia.changeFrame(getFrameAfterJoint());
            compositeInertia.add(childInertia);
         }

         for (int i = 0; i < getNumberOfDoFs(); i++)
         {
            unitMomenta[i].setReferenceFrame(getFrameAfterJoint());
            unitMomenta[i].compute(compositeInertia, unitTwists[i]);
         }

         /*
          * Here we update the mass-matrix block only corresponding to this joint indices. If the joint is a
          * 1-DoF joint, the block has only 1 element, for 6-DoF joint, the block is a 6-by-6 matrix.
          */
         for (int i = 0; i < getNumberOfDoFs(); i++)
         {
            for (int j = 0; j < getNumberOfDoFs(); j++)
            {
               double massMatrixEntry = unitTwists[i].dot(unitMomenta[j]);
               setSymmetricEntry(jointIndices[i], jointIndices[j], massMatrix, massMatrixEntry);
            }
         }

         /*
          * Going up to the root to update the cross components between this joint and its ancestors.
          */
         for (int i = 0; i < getNumberOfDoFs(); i++)
         {
            int dofIndex = jointIndices[i];
            CompositeRigidBodyInertia ancestor = parent;
            Momentum unitMomentum = unitMomenta[i];

            while (ancestor != null && ancestor.getJoint() != null)
            {
               unitMomentum.changeFrame(ancestor.getFrameAfterJoint());

               for (int j = 0; j < ancestor.getNumberOfDoFs(); j++)
               {
                  double offDiagonalCoeff = ancestor.unitTwists[j].dot(unitMomentum);
                  setSymmetricEntry(ancestor.jointIndices[j], dofIndex, massMatrix, offDiagonalCoeff);
               }
               ancestor = ancestor.parent;
            }
         }
      }

      public void computeCentroidalMomentumMatrix()
      {
         for (int dofIndex = 0; dofIndex < getNumberOfDoFs(); dofIndex++)
         {
            Momentum unitMomentum = unitMomenta[dofIndex];
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

      private ReferenceFrame getFrameAfterJoint()
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
   }
}
