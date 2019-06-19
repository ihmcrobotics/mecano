package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator.ArticulatedBodyRecursionStep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

/**
 * Experimental calculator that implements part of the impulse framework introduced in Impulse-based
 * Dynamic Simulation of Rigid Body System by Brian V. Mirtich
 * <a href="https://people.eecs.berkeley.edu/~jfc/mirtich/thesis/mirtichThesis.pdf">link</a>.
 * <p>
 * <b> This calculator is supposed to be feature complete but has not yet been properly tested nor
 * integrated in a simulation. It is not recommended for use yet. </b>
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class MultiBodyImpulseCalculator
{
   private final ImpulseRecursionStep rootRecursionStep;
   private final Map<RigidBodyReadOnly, ImpulseRecursionStep> rigidBodyToRecursionStepMap = new HashMap<>();

   public MultiBodyImpulseCalculator(ArticulatedBodyRecursionStep rootRecursionStep)
   {
      this.rootRecursionStep = new ImpulseRecursionStep(rootRecursionStep, null);
      buildMultiBodyTree(this.rootRecursionStep);
   }

   private void buildMultiBodyTree(ImpulseRecursionStep recursionStep)
   {
      for (ArticulatedBodyRecursionStep childInertia : recursionStep.articulatedBodyRecursionStep.children)
      {
         ImpulseRecursionStep child = new ImpulseRecursionStep(childInertia, recursionStep);
         rigidBodyToRecursionStepMap.put(childInertia.rigidBody, child);
         buildMultiBodyTree(child);
      }
   }

   public void reset()
   {
      rootRecursionStep.reset();
   }

   public boolean calculateImpulseResponse(RigidBodyReadOnly rigidBody, FrameVector3DReadOnly impulse, FramePoint3DReadOnly pointOfApplication)
   {
      ImpulseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(rigidBody);

      if (recursionStep == null)
         return false;

      recursionStep.setImpulse(impulse, pointOfApplication);
      recursionStep.propagateImpulse(null);

      return true;
   }

   private final FrameVector3D testImpulse = new FrameVector3D();
   private final FramePoint3D testCollisionLocation = new FramePoint3D();

   public boolean calculateCollisionMatrix(RigidBodyReadOnly rigidBody, FramePoint3DReadOnly collisionLocation, Matrix3DBasics collisionMatrixToPack)
   {
      ImpulseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(rigidBody);

      if (recursionStep == null)
         return false;

      testCollisionLocation.setIncludingFrame(collisionLocation);
      testCollisionLocation.changeFrame(recursionStep.getFrameAfterJoint());

      testImpulse.setIncludingFrame(collisionLocation.getReferenceFrame(), Axis.X);
      recursionStep.setImpulse(testImpulse, collisionLocation);
      recursionStep.propagateImpulse(null);
      recursionStep.instantaneousTwistChange.getLinearVelocityAt(testCollisionLocation, testImpulse);
      testImpulse.changeFrame(collisionLocation.getReferenceFrame());
      double collisionMatrix00 = testImpulse.getX();
      double collisionMatrix01 = testImpulse.getY();
      double collisionMatrix02 = testImpulse.getZ();

      testImpulse.setIncludingFrame(collisionLocation.getReferenceFrame(), Axis.Y);
      recursionStep.setImpulse(testImpulse, collisionLocation);
      recursionStep.propagateImpulse(null);
      recursionStep.instantaneousTwistChange.getLinearVelocityAt(testCollisionLocation, testImpulse);
      testImpulse.changeFrame(collisionLocation.getReferenceFrame());
      double collisionMatrix10 = testImpulse.getX();
      double collisionMatrix11 = testImpulse.getY();
      double collisionMatrix12 = testImpulse.getZ();

      testImpulse.setIncludingFrame(collisionLocation.getReferenceFrame(), Axis.Z);
      recursionStep.setImpulse(testImpulse, collisionLocation);
      recursionStep.propagateImpulse(null);
      recursionStep.instantaneousTwistChange.getLinearVelocityAt(testCollisionLocation, testImpulse);
      testImpulse.changeFrame(collisionLocation.getReferenceFrame());
      double collisionMatrix20 = testImpulse.getX();
      double collisionMatrix21 = testImpulse.getY();
      double collisionMatrix22 = testImpulse.getZ();

      collisionMatrixToPack.set(collisionMatrix00,
                                collisionMatrix01,
                                collisionMatrix02,
                                collisionMatrix10,
                                collisionMatrix11,
                                collisionMatrix12,
                                collisionMatrix20,
                                collisionMatrix21,
                                collisionMatrix22);
      return true;
   }

   private final DenseMatrix64F jointVelocityMatrix = new DenseMatrix64F(JointReadOnly.MAX_NUMBER_OF_DOFS, 1);

   public boolean writeComputedJointInstanteneousVelocityChange(JointBasics joint)
   {
      ImpulseRecursionStep recursionStep = rigidBodyToRecursionStepMap.get(joint.getSuccessor());

      if (recursionStep == null)
         return false;

      jointVelocityMatrix.reshape(joint.getDegreesOfFreedom(), 1);
      joint.getJointVelocity(0, jointVelocityMatrix);
      CommonOps.addEquals(jointVelocityMatrix, recursionStep.delta_qd);

      return true;
   }

   static class ImpulseRecursionStep
   {
      private final SpatialForce impulse;
      private final SpatialForce impulseForParent;

      private final Twist instantaneousTwistChange = new Twist();

      private final DenseMatrix64F yA;
      private final DenseMatrix64F ya;

      private final DenseMatrix64F U_Dinv_ST;
      private final DenseMatrix64F one_minus_U_Dinv_ST;
      private final DenseMatrix64F Dinv_UT;
      private final DenseMatrix64F Dinv_ST;
      private final DenseMatrix64F delta_qd;
      private final DenseMatrix64F delta_v;

      private final ArticulatedBodyRecursionStep articulatedBodyRecursionStep;
      private final ImpulseRecursionStep parent;
      private final List<ImpulseRecursionStep> children = new ArrayList<>();

      private boolean isUpToDate = false;

      public ImpulseRecursionStep(ArticulatedBodyRecursionStep articulatedBodyRecursionStep, ImpulseRecursionStep parent)
      {
         this.articulatedBodyRecursionStep = articulatedBodyRecursionStep;
         this.parent = parent;

         if (parent == null)
         {
            impulse = null;
            impulseForParent = null;

            yA = null;
            ya = null;

            U_Dinv_ST = null;
            one_minus_U_Dinv_ST = null;
            Dinv_UT = null;
            Dinv_ST = null;
            delta_qd = null;
            delta_v = null;
         }
         else
         {
            parent.children.add(this);

            impulse = new SpatialForce();
            impulseForParent = new SpatialForce();

            yA = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
            ya = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);

            int nDoFs = articulatedBodyRecursionStep.getJoint().getDegreesOfFreedom();
            U_Dinv_ST = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, SpatialVectorReadOnly.SIZE);
            one_minus_U_Dinv_ST = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, SpatialVectorReadOnly.SIZE);
            Dinv_UT = new DenseMatrix64F(nDoFs, SpatialVectorReadOnly.SIZE);
            Dinv_ST = new DenseMatrix64F(nDoFs, SpatialVectorReadOnly.SIZE);
            delta_qd = new DenseMatrix64F(nDoFs, 1);
            delta_v = new DenseMatrix64F(SpatialVectorReadOnly.SIZE, 1);
         }
      }

      public void reset()
      {
         isUpToDate = false;

         for (int i = 0; i < children.size(); i++)
            children.get(i).reset();
      }

      public void updateIntermediateVariables()
      {
         if (articulatedBodyRecursionStep.isRoot())
            return;

         if (isUpToDate)
            return;

         CommonOps.multTransB(articulatedBodyRecursionStep.U_Dinv, articulatedBodyRecursionStep.S, U_Dinv_ST);
         for (int index = 0; index < U_Dinv_ST.getNumElements(); index++)
            one_minus_U_Dinv_ST.set(index, -U_Dinv_ST.get(index));
         for (int diagIndex = 0; diagIndex < SpatialVectorReadOnly.SIZE; diagIndex++)
            one_minus_U_Dinv_ST.add(diagIndex, diagIndex, 1.0);

         CommonOps.multTransB(articulatedBodyRecursionStep.Dinv, articulatedBodyRecursionStep.U, Dinv_UT);
         CommonOps.multTransB(articulatedBodyRecursionStep.Dinv, articulatedBodyRecursionStep.S, Dinv_ST);

         isUpToDate = true;
      }

      public void setImpulse(FrameVector3DReadOnly impulse, FramePoint3DReadOnly pointOfApplication)
      {
         this.impulse.setIncludingFrame(null, impulse, pointOfApplication);
      }

      public void propagateImpulse(ImpulseRecursionStep impulseSourceChild)
      {
         if (impulseSourceChild != null)
         {
            impulse.setIncludingFrame(impulseSourceChild.impulseForParent);
         }

         updateIntermediateVariables();

         impulse.changeFrame(getFrameAfterJoint());
         impulse.get(yA);
         CommonOps.mult(one_minus_U_Dinv_ST, yA, ya);
         impulseForParent.setIncludingFrame(impulse.getReferenceFrame(), ya);

         if (parent != null)
         {
            parent.propagateImpulse(this);
         }

         instantaneousTwistChange.setIncludingFrame(parent.instantaneousTwistChange);
         instantaneousTwistChange.changeFrame(getFrameAfterJoint());
         instantaneousTwistChange.get(delta_v);

         CommonOps.mult(Dinv_UT, delta_v, delta_qd);
         CommonOps.multAdd(Dinv_ST, yA, delta_qd);

         CommonOps.multAdd(articulatedBodyRecursionStep.S, delta_qd, delta_qd);

         instantaneousTwistChange.set(delta_v);
      }

      public MovingReferenceFrame getFrameAfterJoint()
      {
         return articulatedBodyRecursionStep.getFrameAfterJoint();
      }
   }
}
