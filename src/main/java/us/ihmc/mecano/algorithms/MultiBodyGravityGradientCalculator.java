package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class MultiBodyGravityGradientCalculator
{
   private final MultiBodySystemReadOnly input;
   private final DMatrixRMaj gradientMatrix;
   private final AlgorithmStep initialStep;
   private final FrameVector3D gravitationalAcceleration = new FrameVector3D();

   public MultiBodyGravityGradientCalculator(MultiBodySystemReadOnly input)
   {
      this.input = input;

      gradientMatrix = new DMatrixRMaj(input.getNumberOfDoFs(), input.getNumberOfDoFs());

      initialStep = new AlgorithmStep(input.getRootBody(), null, null);
      buildMultiBodyTree(initialStep, input.getJointsToIgnore());
      initialStep.includeIgnoredSubtreeInertia();

      gravitationalAcceleration.setToZero(input.getInertialFrame());
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration, it is usually equal to
    *                {@code (0, 0, -9.81)}.
    */
   public void setGravitionalAcceleration(FrameTuple3DReadOnly gravity)
   {
      gravitationalAcceleration.set(gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration, it is usually equal to
    *                {@code (0, 0, -9.81)}.
    */
   public void setGravitionalAcceleration(Tuple3DReadOnly gravity)
   {
      gravitationalAcceleration.setIncludingFrame(input.getInertialFrame(), gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration along the z-axis, it is usually equal to
    *                {@code -9.81}.
    */
   public void setGravitionalAcceleration(double gravity)
   {
      setGravitionalAcceleration(0.0, 0.0, gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravityX the gravitational linear acceleration along the x-axis, it is usually equal to
    *                 {@code 0}.
    * @param gravityY the gravitational linear acceleration along the y-axis, it is usually equal to
    *                 {@code 0}.
    * @param gravityZ the gravitational linear acceleration along the z-axis, it is usually equal to
    *                 {@code -9.81}.
    */
   public void setGravitionalAcceleration(double gravityX, double gravityY, double gravityZ)
   {
      gravitationalAcceleration.setIncludingFrame(input.getInertialFrame(), gravityX, gravityY, gravityZ);
   }

   private List<AlgorithmStep> buildMultiBodyTree(AlgorithmStep parent, Collection<? extends JointReadOnly> jointsToIgnore)
   {
      List<AlgorithmStep> algorithmSteps = new ArrayList<>();
      algorithmSteps.add(parent);

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
            AlgorithmStep child = new AlgorithmStep(childBody, parent, jointIndices);
            algorithmSteps.addAll(buildMultiBodyTree(child, jointsToIgnore));
         }
      }

      return algorithmSteps;
   }

   public void compute()
   {
      initialStep.passOne();
      initialStep.passTwo();
   }

   public DMatrixRMaj getGradientMatrix()
   {
      return gradientMatrix;
   }

   private class AlgorithmStep
   {
      private final RigidBodyReadOnly rigidBody;
      private final SpatialInertia bodyInertia;
      private final AlgorithmStep parent;
      private final int[] jointIndices;
      private final List<AlgorithmStep> children = new ArrayList<>();

      private double subTreeMass;
      private final FramePoint3D centerOfMassTimesMass = new FramePoint3D();
      private final FramePoint3D centerOfMass = new FramePoint3D();

      private final FramePoint3D childCoM = new FramePoint3D();
      private final FrameVector3D gravity = new FrameVector3D();

      private final RigidBodyTransform transformToInertial = new RigidBodyTransform();

      public AlgorithmStep(RigidBodyReadOnly rigidBody, AlgorithmStep parent, int[] jointIndices)
      {
         this.rigidBody = rigidBody;
         this.parent = parent;
         this.jointIndices = jointIndices;

         if (isRoot())
         {
            bodyInertia = null;
         }
         else
         {
            parent.children.add(this);
            bodyInertia = new SpatialInertia(rigidBody.getInertia());
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

      private final Twist unitTwist_i = new Twist();

      public void passOne()
      {
         for (int i = 0; i < children.size(); i++)
            children.get(i).passOne();

         // Update the subtree mass
         subTreeMass = bodyInertia == null ? 0.0 : bodyInertia.getMass();
         for (int i = 0; i < children.size(); i++)
            subTreeMass += children.get(i).subTreeMass;

         ReferenceFrame frameToUse = isRoot() ? getBodyFixedFrame() : getFrameAfterJoint();
         transformToInertial.set(frameToUse.getTransformToRoot());
         gravity.setIncludingFrame(gravitationalAcceleration);
         transformToInertial.inverseTransform(gravity);
         gravity.setReferenceFrame(frameToUse);

         // The centerOfMassTimesMass can be used to obtain the overall center of mass position.
         if (bodyInertia == null)
         {
            centerOfMassTimesMass.setToZero(getBodyFixedFrame());
         }
         else
         {
            centerOfMassTimesMass.setIncludingFrame(bodyInertia.getCenterOfMassOffset());
            centerOfMassTimesMass.changeFrame(frameToUse);
            centerOfMassTimesMass.scale(bodyInertia.getMass());
         }

         for (int i = 0; i < children.size(); i++)
         {
            childCoM.setIncludingFrame(children.get(i).centerOfMass);
            childCoM.changeFrame(frameToUse);
            centerOfMassTimesMass.scaleAdd(children.get(i).subTreeMass, childCoM, centerOfMassTimesMass);
         }

         centerOfMass.setIncludingFrame(centerOfMassTimesMass);
         centerOfMass.scale(1.0 / subTreeMass);
      }

      private final Vector3D tempCross = new Vector3D();

      /** From leaves to root, compute the elements of the gradient matrix. */
      public void passTwo()
      {
         for (int i = 0; i < children.size(); i++)
            children.get(i).passTwo();

         if (isRoot())
            return;

         for (int i = 0; i < getNumberOfDoFs(); i++)
         {
            TwistReadOnly unitTwist_i = getUnitTwist(i);
            tempCross.cross(gravity, unitTwist_i.getAngularPart());
            double gradient_ii = -tempCross.dot(unitTwist_i.getLinearPart());
            tempCross.cross(centerOfMass);
            gradient_ii += tempCross.dot(unitTwist_i.getAngularPart());
            gradient_ii *= subTreeMass;
            gradientMatrix.set(jointIndices[i], jointIndices[i], gradient_ii);

            for (int j = 0; j < i; j++)
            {
               TwistReadOnly unitTwist_j = getUnitTwist(j);
               tempCross.cross(gravity, unitTwist_j.getAngularPart());
               double gradient_ji = -tempCross.dot(unitTwist_i.getLinearPart());
               tempCross.cross(gravity, unitTwist_j.getAngularPart());
               tempCross.cross(centerOfMass);
               gradient_ji += tempCross.dot(unitTwist_i.getAngularPart());
               gradient_ji *= subTreeMass;
               gradientMatrix.set(jointIndices[j], jointIndices[i], gradient_ji);

//               tempCross.cross(gravity, unitTwist_i.getAngularPart());
//               double gradient_ij = -subTreeMass * tempCross.dot(unitTwist_j.getLinearPart());
//               tempCross.cross(centerOfMass);
//               gradient_ij += tempCross.dot(unitTwist_j.getAngularPart());
//               gradient_ij *= subTreeMass;
//               gradientMatrix.set(jointIndices[i], jointIndices[j], gradient_ij);
            }
         }

         AlgorithmStep ancestor = parent;

         while (!ancestor.isRoot())
         {
            for (int j = 0; j < getNumberOfDoFs(); j++)
            {
               int index_j = jointIndices[j];
               TwistReadOnly unitTwist_j = getUnitTwist(j);

               for (int i = 0; i < ancestor.getNumberOfDoFs(); i++)
               {
                  int index_i = ancestor.jointIndices[i];
                  getUnitTwistInLocalFrame(ancestor, i, unitTwist_i);

                  tempCross.cross(gravity, unitTwist_i.getAngularPart());
                  double gradient_ij = -subTreeMass * tempCross.dot(unitTwist_j.getLinearPart());
                  tempCross.cross(centerOfMassTimesMass);
                  gradient_ij += tempCross.dot(unitTwist_j.getAngularPart());
                  gradientMatrix.set(index_i, index_j, gradient_ij);
                  gradientMatrix.set(index_j, index_i, gradient_ij);
               }
            }

            ancestor = ancestor.parent;
         }
      }

      private void getUnitTwistInLocalFrame(AlgorithmStep other, int dofIndex, TwistBasics unitTwistToPack)
      {
         unitTwistToPack.setIncludingFrame(other.getUnitTwist(dofIndex));
         unitTwistToPack.applyTransform(other.transformToInertial);
         unitTwistToPack.applyInverseTransform(transformToInertial);
      }

      public boolean isRoot()
      {
         return parent == null;
      }

      public JointReadOnly getJoint()
      {
         return rigidBody.getParentJoint();
      }

      public int getNumberOfDoFs()
      {
         return getJoint().getDegreesOfFreedom();
      }

      public MovingReferenceFrame getFrameAfterJoint()
      {
         return getJoint().getFrameAfterJoint();
      }

      public TwistReadOnly getUnitTwist(int dofIndex)
      {
         return getJoint().getUnitTwists().get(dofIndex);
      }

      public MovingReferenceFrame getBodyFixedFrame()
      {
         return rigidBody.getBodyFixedFrame();
      }
   }
}