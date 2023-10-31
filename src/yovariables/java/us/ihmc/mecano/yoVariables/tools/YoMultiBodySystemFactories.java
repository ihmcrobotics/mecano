package us.ihmc.mecano.yoVariables.tools;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories.JointBuilder;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;
import us.ihmc.mecano.yoVariables.multiBodySystem.*;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This class gathers factories useful for creating multi-body system backed by {@code YoVariable}s.
 *
 * @author Sylvain Bertrand
 */
public class YoMultiBodySystemFactories
{

   /**
    * Creates a default joint builder that creates joints backed by {@code YoVariable}s.
    * <p>
    * Used with
    * {@link MultiBodySystemFactories#cloneMultiBodySystem(RigidBodyReadOnly, ReferenceFrame, String, RigidBodyBuilder, JointBuilder)},
    * the returned builder can be used to clone any multi-body system and upgrade the cloned joints to
    * be backed by {@code YoVariable}s.
    * </p>
    *
    * @param registry the registry to register child variables to.
    * @return the new joint builder.
    */
   public static JointBuilder newYoJointBuilder(YoRegistry registry)
   {
      return new JointBuilder()
      {
         @Override
         public YoSixDoFJoint buildSixDoFJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
         {
            return new YoSixDoFJoint(name, predecessor, transformToParent, registry);
         }

         @Override
         public YoPlanarJoint buildPlanarJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
         {
            return new YoPlanarJoint(name, predecessor, transformToParent, registry);
         }

         @Override
         public YoSphericalJoint buildSphericalJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent)
         {
            return new YoSphericalJoint(name, predecessor, transformToParent, registry);
         }

         @Override
         public YoRevoluteJoint buildRevoluteJoint(String name,
                                                   RigidBodyBasics predecessor,
                                                   RigidBodyTransformReadOnly transformToParent,
                                                   Vector3DReadOnly jointAxis)
         {
            return new YoRevoluteJoint(name, predecessor, transformToParent, jointAxis, registry);
         }

         @Override
         public YoPrismaticJoint buildPrismaticJoint(String name,
                                                     RigidBodyBasics predecessor,
                                                     RigidBodyTransformReadOnly transformToParent,
                                                     Vector3DReadOnly jointAxis)
         {
            return new YoPrismaticJoint(name, predecessor, transformToParent, jointAxis, registry);
         }

         @Override
         public YoCrossFourBarJoint cloneCrossFourBarJoint(CrossFourBarJointReadOnly original, String cloneSuffix, RigidBodyBasics clonePredecessor)
         {
            RevoluteJointReadOnly originalJointA = original.getJointA();
            RevoluteJointReadOnly originalJointB = original.getJointB();
            RevoluteJointReadOnly originalJointC = original.getJointC();
            RevoluteJointReadOnly originalJointD = original.getJointD();
            RigidBodyReadOnly originalBodyDA = originalJointA.getSuccessor();
            RigidBodyReadOnly originalBodyBC = originalJointB.getSuccessor();
            int loopClosureIndex;
            if (originalJointA.isLoopClosure())
               loopClosureIndex = 0;
            else if (originalJointB.isLoopClosure())
               loopClosureIndex = 1;
            else if (originalJointC.isLoopClosure())
               loopClosureIndex = 2;
            else
               loopClosureIndex = 3;

            return new YoCrossFourBarJoint(original.getName() + cloneSuffix,
                                           clonePredecessor,
                                           originalJointA.getName() + cloneSuffix,
                                           originalJointB.getName() + cloneSuffix,
                                           originalJointC.getName() + cloneSuffix,
                                           originalJointD.getName() + cloneSuffix,
                                           originalBodyDA.getName() + cloneSuffix,
                                           originalBodyBC.getName() + cloneSuffix,
                                           originalJointA.getFrameBeforeJoint().getTransformToParent(),
                                           originalJointB.getFrameBeforeJoint().getTransformToParent(),
                                           originalJointD.getFrameBeforeJoint().getTransformToParent(),
                                           originalJointC.getFrameBeforeJoint().getTransformToParent(),
                                           originalBodyDA.getInertia().getMomentOfInertia(),
                                           originalBodyBC.getInertia().getMomentOfInertia(),
                                           originalBodyDA.getInertia().getMass(),
                                           originalBodyBC.getInertia().getMass(),
                                           originalBodyDA.getBodyFixedFrame().getTransformToParent(),
                                           originalBodyBC.getBodyFixedFrame().getTransformToParent(),
                                           original.getActuatedJointIndex(),
                                           loopClosureIndex,
                                           original.getJointAxis(),
                                           registry);
         }

         @Override
         public YoRevoluteTwinsJoint cloneRevoluteTwinsJoint(RevoluteTwinsJointReadOnly original, String cloneSuffix, RigidBodyBasics clonePredecessor)
         {
            RevoluteJointReadOnly originalJointA = original.getJointA();
            RevoluteJointReadOnly originalJointB = original.getJointB();
            RigidBodyReadOnly originalBodyAB = originalJointA.getSuccessor();

            return new YoRevoluteTwinsJoint(original.getName() + cloneSuffix,
                                            clonePredecessor,
                                            originalJointA.getName() + cloneSuffix,
                                            originalJointB.getName() + cloneSuffix,
                                            originalBodyAB.getName() + cloneSuffix,
                                            originalJointA.getFrameBeforeJoint().getTransformToParent(),
                                            originalJointB.getFrameBeforeJoint().getTransformToParent(),
                                            originalBodyAB.getInertia().getMomentOfInertia(),
                                            originalBodyAB.getInertia().getMass(),
                                            originalBodyAB.getBodyFixedFrame().getTransformToParent(),
                                            original.getActuatedJointIndex(),
                                            original.getConstraintRatio(),
                                            original.getConstraintOffset(),
                                            original.getJointAxis(),
                                            registry);
         }
      };
   }

   public static RigidBodyBuilder newYoRigidBodyBuilder(YoRegistry registry)
   {
      return new RigidBodyBuilder()
      {
         @Override
         public YoRigidBody buildRoot(String bodyName, RigidBodyTransformReadOnly transformToParent, ReferenceFrame parentStationaryFrame)
         {
            return new YoRigidBody(bodyName, transformToParent, parentStationaryFrame);
         }

         @Override
         public YoRigidBody build(String bodyName,
                                  JointBasics parentJoint,
                                  Matrix3DReadOnly momentOfInertia,
                                  double mass,
                                  RigidBodyTransformReadOnly inertiaPose)
         {
            return new YoRigidBody(bodyName, parentJoint, momentOfInertia, mass, inertiaPose, registry);
         }

         @Override
         public YoRigidBody cloneRigidBody(RigidBodyReadOnly original, ReferenceFrame cloneStationaryFrame, String cloneSuffix, JointBasics parentJointOfClone)
         {
            if (original.isRootBody() && parentJointOfClone != null)
               throw new IllegalArgumentException("Inconsistent set of arguments. If the original body is the root body, the parent joint should be null.");

            String nameOriginal = original.getName();

            if (parentJointOfClone == null)
            {
               /*
                * Regardless of whether the original body is the root body or not, we create the clone as the root
                * body of its own multi-body system. This allows to keep the clone subtree detached from the
                * original one but still ensure that the clone follows in space the pose of the original body.
                */
               MovingReferenceFrame originalBodyFixedFrame = original.getBodyFixedFrame();
               if (cloneStationaryFrame == null)
                  cloneStationaryFrame = originalBodyFixedFrame;
               return buildRoot(nameOriginal + cloneSuffix, originalBodyFixedFrame.getTransformToParent(), cloneStationaryFrame);
            }
            else
            {
               SpatialInertiaReadOnly originalInertia = original.getInertia();

               double mass = originalInertia.getMass();
               Matrix3DReadOnly momentOfInertia = originalInertia.getMomentOfInertia();
               RigidBodyTransform inertiaPose = new RigidBodyTransform(original.getBodyFixedFrame().getTransformToParent());
               YoRigidBody clone = build(nameOriginal + cloneSuffix, parentJointOfClone, momentOfInertia, mass, inertiaPose);
               clone.getInertia().getCenterOfMassOffset().set((Tuple3DReadOnly) originalInertia.getCenterOfMassOffset());
               return clone;
            }
         }
      };
   }
}
