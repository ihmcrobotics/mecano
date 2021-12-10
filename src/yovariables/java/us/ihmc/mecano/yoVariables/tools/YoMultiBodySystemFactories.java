package us.ihmc.mecano.yoVariables.tools;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories.JointBuilder;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoCrossFourBarJoint;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoPlanarJoint;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoPrismaticJoint;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoRevoluteJoint;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoSixDoFJoint;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoSphericalJoint;
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
         public YoRevoluteJoint buildRevoluteJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent, Vector3DReadOnly jointAxis)
         {
            return new YoRevoluteJoint(name, predecessor, transformToParent, jointAxis, registry);
         }

         @Override
         public YoPrismaticJoint buildPrismaticJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent, Vector3DReadOnly jointAxis)
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
      };
   }
}
