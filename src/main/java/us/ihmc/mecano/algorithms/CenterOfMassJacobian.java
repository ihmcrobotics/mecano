package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Computes the center of mass Jacobian that maps from joint velocity space to center of mass
 * Cartesian velocity space.
 * 
 * @author Sylvain Bertrand
 */
public class CenterOfMassJacobian implements ReferenceFrameHolder
{
   /**
    * Whether the Jacobian is to be expressed in the center of mass frame that is updated internally.
    */
   private final boolean isJacobianFrameAtCenterOfMass;
   /**
    * The reference frame in which the internal including the jacobian matrix is to be expressed in.
    * <p>
    * It can either be provided at construction or created internally to be at the center of mass.
    * </p>
    */
   private final ReferenceFrame jacobianFrame;
   /**
    * The root frame to which all the robot frames are attached to. It is used for when the Jacobian is
    * expressed in the center of mass frame.
    */
   private final ReferenceFrame rootFrame;
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;
   /** The root of the internal recursive algorithm. */
   private final RecursionStep initialRecursionStep;
   /**
    * Only the first pass of this algorithm has to be recursive, the two other passes can be iterative
    * which can provide a slight performance improvement.
    */
   private final RecursionStep[] recursionSteps;
   /** Map to quickly retrieve information for each rigid-body. */
   private final Map<RigidBodyReadOnly, RecursionStep> rigidBodyToRecursionStepMap = new LinkedHashMap<>();

   /** The center of mass Jacobian. */
   private final DenseMatrix64F jacobianMatrix;
   /** Matrix containing the velocities of the joints to consider. */
   private final DenseMatrix64F jointVelocityMatrix;
   /** Intermediate variable for garbage free operations. */
   private final DenseMatrix64F centerOfMassVelocityMatrix = new DenseMatrix64F(3, 1);

   /** Intermediate variable to store one column of the Jacobian matrix. */
   private final FixedFrameVector3DBasics jacobianColumn;
   /** Intermediate variable to store the unit-twist of the parent joint. */
   private final Twist jointUnitTwist = new Twist();

   /** The center of mass velocity. */
   private final FixedFrameVector3DBasics centerOfMassVelocity = MecanoFactories.newFixedFrameVector3DBasics(this);

   /** Whether the Jacobian has been updated since the last call to {@link #reset()}. */
   private boolean isJacobianUpToDate = false;
   /**
    * Whether the center of mass velocity has been updated since the last call to {@link #reset()}.
    */
   private boolean isCenterOfMassVelocityUpToDate = false;

   /**
    * Creates a new calculator for the subtree that starts off the given {@code rootBody}.
    * <p>
    * A new reference frame at the center of mass is created, the Jacobian is expressed in that
    * reference frame. The center of mass frame can then be retrieved using
    * {@link #getReferenceFrame()}.
    * </p>
    *
    * @param rootBody the start of subtree for which the center of mass Jacobian is to be computed. Not
    *           modified.
    * @param centerOfMassFrameName the name for the new frame at the center of mass.
    */
   public CenterOfMassJacobian(RigidBodyReadOnly rootBody, String centerOfMassFrameName)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), centerOfMassFrameName);
   }

   /**
    * Creates a new calculator for the subtree that starts off the given {@code rootBody}.
    * 
    * @param rootBody the start of subtree for which the center of mass Jacobian is to be computed. Not
    *           modified.
    * @param jacobianFrame the frame in which the center of mass Jacobian is to be expressed.
    */
   public CenterOfMassJacobian(RigidBodyReadOnly rootBody, ReferenceFrame jacobianFrame)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), jacobianFrame);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    * <p>
    * A new reference frame at the center of mass is created, the Jacobian is expressed in that
    * reference frame. The center of mass frame can then be retrieved using
    * {@link #getReferenceFrame()}.
    * </p>
    * 
    * @param input the definition of the system to be evaluated by this calculator.
    * @param centerOfMassFrameName the name for the new frame at the center of mass.
    */
   public CenterOfMassJacobian(MultiBodySystemReadOnly input, String centerOfMassFrameName)
   {
      this(input, centerOfMassFrameName, true);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    * <p>
    * A new reference frame at the center of mass is created, the Jacobian is expressed in that
    * reference frame. The center of mass frame can then be retrieved using
    * {@link #getReferenceFrame()}.
    * </p>
    * 
    * @param input the definition of the system to be evaluated by this calculator.
    * @param centerOfMassFrameName the name for the new frame at the center of mass.
    * @param considerIgnoredSubtreesInertia whether the inertia of the ignored part(s) of the given
    *           multi-body system should be considered. When {@code true}, this provides more accurate
    *           Jacobian as it account for the ignored rigid-bodies, i.e. bodies which have an ancestor
    *           joint that is ignored as specified in the given {@code input}. When {@code false}, the
    *           resulting Jacobian may be less accurate and this calculator may gain slight performance
    *           improvement.
    */
   public CenterOfMassJacobian(MultiBodySystemReadOnly input, String centerOfMassFrameName, boolean considerIgnoredSubtreesInertia)
   {
      this(input, null, centerOfMassFrameName, considerIgnoredSubtreesInertia);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input the definition of the system to be evaluated by this calculator.
    * @param jacobianFrame the frame in which the center of mass Jacobian is to be expressed.
    */
   public CenterOfMassJacobian(MultiBodySystemReadOnly input, ReferenceFrame jacobianFrame)
   {
      this(input, jacobianFrame, true);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input the definition of the system to be evaluated by this calculator.
    * @param jacobianFrame the frame in which the center of mass Jacobian is to be expressed.
    * @param considerIgnoredSubtreesInertia whether the inertia of the ignored part(s) of the given
    *           multi-body system should be considered. When {@code true}, this provides more accurate
    *           Jacobian as it account for the ignored rigid-bodies, i.e. bodies which have an ancestor
    *           joint that is ignored as specified in the given {@code input}. When {@code false}, the
    *           resulting Jacobian may be less accurate and this calculator may gain slight performance
    *           improvement.
    */
   public CenterOfMassJacobian(MultiBodySystemReadOnly input, ReferenceFrame jacobianFrame, boolean considerIgnoredSubtreesInertia)
   {
      this(input, jacobianFrame, null, considerIgnoredSubtreesInertia);
   }

   private CenterOfMassJacobian(MultiBodySystemReadOnly input, ReferenceFrame jacobianFrame, String centerOfMassFrameName,
                                boolean considerIgnoredSubtreesInertia)
   {
      this.input = input;
      isJacobianFrameAtCenterOfMass = jacobianFrame == null;
      RigidBodyReadOnly rootBody = input.getRootBody();
      rootFrame = rootBody.getBodyFixedFrame().getRootFrame();
      if (isJacobianFrameAtCenterOfMass)
      {
         this.jacobianFrame = new ReferenceFrame(centerOfMassFrameName, rootFrame)
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.setTranslationAndIdentityRotation(initialRecursionStep.centerOfMass);
            }
         };
      }
      else
      {
         this.jacobianFrame = jacobianFrame;
      }

      initialRecursionStep = new RecursionStep(rootBody, null);
      rigidBodyToRecursionStepMap.put(rootBody, initialRecursionStep);
      buildMultiBodyTree(initialRecursionStep, input.getJointsToIgnore());
      if (considerIgnoredSubtreesInertia)
         initialRecursionStep.includeIgnoredSubtreeInertia();
      recursionSteps = rigidBodyToRecursionStepMap.values().toArray(new RecursionStep[0]);

      jacobianColumn = new FrameVector3D(this.jacobianFrame);

      int nDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider());
      jacobianMatrix = new DenseMatrix64F(3, nDegreesOfFreedom);
      jointVelocityMatrix = new DenseMatrix64F(nDegreesOfFreedom, 1);
   }

   private void buildMultiBodyTree(RecursionStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      for (JointReadOnly childJoint : parent.rigidBody.getChildrenJoints())
      {
         if (jointsToIgnore.contains(childJoint))
            continue;

         RigidBodyReadOnly childBody = childJoint.getSuccessor();
         if (childBody != null)
         {
            int[] jointIndices = input.getJointMatrixIndexProvider().getJointDoFIndices(childJoint);
            RecursionStep child = new RecursionStep(childBody, jointIndices);
            rigidBodyToRecursionStepMap.put(childBody, child);
            parent.children.add(child);
            buildMultiBodyTree(child, jointsToIgnore);
         }
      }
   }

   /**
    * Invalidates the internal memory.
    */
   public void reset()
   {
      isJacobianUpToDate = false;
      isCenterOfMassVelocityUpToDate = false;
   }

   private void updateJacobian()
   {
      if (isJacobianUpToDate)
         return;

      passOne(initialRecursionStep);
      passTwo();
      passThree(1.0 / initialRecursionStep.subTreeMass);
      isJacobianUpToDate = true;
   }

   /**
    * Recursion method that calls {@link RecursionStep#passOne()} starting with the system's leaves.
    * 
    * @param current the current recursion step.
    * @see RecursionStep#passOne()
    */
   private void passOne(RecursionStep current)
   {
      for (int childIndex = 0; childIndex < current.children.size(); childIndex++)
      {
         RecursionStep child = current.children.get(childIndex);
         passOne(child);
      }

      current.passOne();
   }

   /**
    * Iterative method that calls {@link RecursionStep#passTwo()} only if
    * {@link #isJacobianFrameAtCenterOfMass} is {@code true}.
    * 
    * @see RecursionStep#passTwo()
    */
   private void passTwo()
   {
      if (!isJacobianFrameAtCenterOfMass)
         return;

      jacobianFrame.update();

      for (RecursionStep recursionStep : recursionSteps)
         recursionStep.passTwo();
   }

   /**
    * Iterative method that calls {@link RecursionStep#passThree(double)}.
    * 
    * @see RecursionStep#passThree(double)
    */
   private void passThree(double inverseOfTotalMass)
   {
      for (RecursionStep recursionStep : recursionSteps)
         recursionStep.passThree(inverseOfTotalMass);
   }

   private void updateCenterOfMassVelocity()
   {
      if (isCenterOfMassVelocityUpToDate)
         return;

      List<? extends JointReadOnly> joints = input.getJointMatrixIndexProvider().getIndexedJointsInOrder();
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocityMatrix);
      CommonOps.mult(getJacobianMatrix(), jointVelocityMatrix, centerOfMassVelocityMatrix);
      centerOfMassVelocity.set(centerOfMassVelocityMatrix);
      isCenterOfMassVelocityUpToDate = true;
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
      updateJacobian();
      return initialRecursionStep.subTreeMass;
   }

   /**
    * Gets the center of mass position of the multi-body system.
    *
    * @return the center of mass position.
    */
   public FramePoint3DReadOnly getCenterOfMass()
   {
      updateJacobian();
      return initialRecursionStep.centerOfMass;
   }

   /**
    * Gets the center of mass velocity of the multi-body system.
    *
    * @return the center of mass velocity.
    */
   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      updateCenterOfMassVelocity();
      return centerOfMassVelocity;
   }

   /**
    * Computes and packs the center of mass velocity for the given joint velocities.
    * <p>
    * The given matrix is expected to have been configured using the same
    * {@link JointMatrixIndexProvider} that was used to configure this calculator.
    * </p>
    * 
    * @param jointVelocityMatrix the matrix containing the joint velocities to use. Not modified.
    * @param centerOfMassVelocityToPack the vector used to stored the computed center of mass velocity.
    *           Modified.
    */
   public void getCenterOfMassVelocity(DenseMatrix64F jointVelocityMatrix, FrameVector3DBasics centerOfMassVelocityToPack)
   {
      CommonOps.mult(getJacobianMatrix(), jointVelocityMatrix, centerOfMassVelocityMatrix);
      centerOfMassVelocityToPack.setIncludingFrame(jacobianFrame, centerOfMassVelocityMatrix);
   }

   /**
    * Gets the N-by-3 center of mass Jacobian, where N is the number of degrees of freedom of the
    * multi-body system.
    * <p>
    * The center of mass Jacobian maps from joint velocity space to center of mass Cartesian velocity
    * space and is expressed in the frame {@link #getReferenceFrame()}. The latter implies that when
    * multiplied to the joint velocity matrix, the result is the center of mass velocity expressed in
    * {@link #getReferenceFrame()}.
    * </p>
    * 
    * @return the center of mass Jacobian.
    */
   public DenseMatrix64F getJacobianMatrix()
   {
      updateJacobian();
      return jacobianMatrix;
   }

   /**
    * The reference frame in which the center of mass Jacobian is computed.
    *
    * @return this calculator's reference frame.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return jacobianFrame;
   }

   /**
    * Represents a single recursion step with all the intermediate variables needed.
    * 
    * @author Sylvain Bertrand
    */
   private class RecursionStep
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
       * The total mass of this rigid-body and all its descendant.
       */
      private double subTreeMass;
      /**
       * Represents the sum of the center of mass of each rigid-body in the current sub-tree scaled by
       * their respective mass.
       */
      private final FramePoint3D centerOfMassTimesMass = new FramePoint3D();
      /**
       * Represents the the center of mass of the current sub-tree.
       */
      private final FramePoint3D centerOfMass = new FramePoint3D();
      /**
       * Result of this recursion step: the matrix block of the Jacobian for the parent joint.
       */
      private final DenseMatrix64F jacobianJointBlock;
      /**
       * The recursion steps holding onto the direct successor of this recursion step's rigid-body.
       */
      private final List<RecursionStep> children = new ArrayList<>();
      /**
       * Joint indices for storing {@code jacobianJointBlock} in the main matrix {@code jacobianMatrix}.
       */
      private final int[] jointIndices;

      public RecursionStep(RigidBodyReadOnly rigidBody, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.jointIndices = jointIndices;

         if (isRoot())
         {
            bodyInertia = null;
            jacobianJointBlock = null;
         }
         else
         {
            bodyInertia = new SpatialInertia(rigidBody.getInertia());
            jacobianJointBlock = new DenseMatrix64F(3, getJoint().getDegreesOfFreedom());
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
                  subtreeIneria.changeFrame(rigidBody.getBodyFixedFrame());
                  bodyInertia.add(subtreeIneria);
               }
            }
         }

         for (int childIndex = 0; childIndex < children.size(); childIndex++)
            children.get(childIndex).includeIgnoredSubtreeInertia();
      }

      /**
       * First pass going from the leaves to the root.
       * <p>
       * Here the subtree mass and center of mass is computed for each rigid-body.
       * </p>
       */
      public void passOne()
      {
         ReferenceFrame frameToUse = isJacobianFrameAtCenterOfMass ? rootFrame : jacobianFrame;

         if (isRoot())
         {
            // Update the total mass
            subTreeMass = bodyInertia == null ? 0.0 : bodyInertia.getMass();
            for (int i = 0; i < children.size(); i++)
               subTreeMass += children.get(i).subTreeMass;

            // The centerOfMassTimesMass can be used to obtain the overall center of mass position.
            if (bodyInertia == null)
            {
               centerOfMassTimesMass.setToZero(frameToUse);
            }
            else
            {
               centerOfMassTimesMass.setIncludingFrame(bodyInertia.getCenterOfMassOffset());
               centerOfMassTimesMass.changeFrame(frameToUse);
               centerOfMassTimesMass.scale(bodyInertia.getMass());
            }

            for (int i = 0; i < children.size(); i++)
            {
               centerOfMassTimesMass.add(children.get(i).centerOfMassTimesMass);
            }
         }
         else
         {
            // Update the sub-tree mass
            subTreeMass = bodyInertia.getMass();
            for (int i = 0; i < children.size(); i++)
               subTreeMass += children.get(i).subTreeMass;

            // Update the sub-tree center of mass
            centerOfMassTimesMass.setIncludingFrame(bodyInertia.getCenterOfMassOffset());
            centerOfMassTimesMass.changeFrame(frameToUse);
            centerOfMassTimesMass.scale(bodyInertia.getMass());

            for (int i = 0; i < children.size(); i++)
            {
               centerOfMassTimesMass.add(children.get(i).centerOfMassTimesMass);
            }
         }

         centerOfMass.setIncludingFrame(centerOfMassTimesMass);
         centerOfMass.scale(1.0 / subTreeMass);
      }

      /**
       * Second pass that can be done iteratively and each iteration is independent.
       * <p>
       * This pass is only needed when the {@code jacobianFrame} is at the center of mass and that this
       * calculator has to update. It recomputes the intermediate variable {@link #centerOfMassTimesMass}
       * at the center of mass frame so it can be used in the next and last pass.
       * </p>
       */
      public void passTwo()
      {
         centerOfMassTimesMass.setIncludingFrame(centerOfMass);
         centerOfMassTimesMass.sub(jacobianFrame.getTransformToRoot().getTranslationVector());
         centerOfMassTimesMass.setReferenceFrame(jacobianFrame);
         centerOfMassTimesMass.scale(subTreeMass);
      }

      /**
       * Third and last pass that can be done iteratively and each iteration is independent.
       * <p>
       * Computes the Jacobian block corresponding to this joint.
       * </p>
       * 
       * @param inverseOfTotalMass the inverse of the total system mass.
       */
      public void passThree(double inverseOfTotalMass)
      {
         if (isRoot())
            return;

         // Compute the Jacobian matrix block corresponding to the current joint.
         JointReadOnly joint = getJoint();

         for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
         {
            jointUnitTwist.setIncludingFrame(joint.getUnitTwists().get(i));
            jointUnitTwist.changeFrame(jacobianFrame);
            jacobianColumn.cross(jointUnitTwist.getAngularPart(), centerOfMassTimesMass);
            jacobianColumn.scaleAdd(subTreeMass, jointUnitTwist.getLinearPart(), jacobianColumn);
            jacobianColumn.get(0, i, jacobianJointBlock);
         }

         CommonOps.scale(inverseOfTotalMass, jacobianJointBlock);

         for (int dofIndex = 0; dofIndex < getJoint().getDegreesOfFreedom(); dofIndex++)
         {
            int column = jointIndices[dofIndex];
            CommonOps.extract(jacobianJointBlock, 0, 3, dofIndex, dofIndex + 1, jacobianMatrix, 0, column);
         }
      }

      public boolean isRoot()
      {
         return jointIndices == null;
      }

      public JointReadOnly getJoint()
      {
         return rigidBody.getParentJoint();
      }
   }
}
