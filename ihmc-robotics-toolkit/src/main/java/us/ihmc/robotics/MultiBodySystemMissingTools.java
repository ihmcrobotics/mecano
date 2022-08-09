package us.ihmc.robotics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.ArrayList;
import java.util.List;

public class MultiBodySystemMissingTools
{
   public static void copyOneDoFJointsConfiguration(JointBasics[] source, JointBasics[] destination)
   {
      OneDoFJointBasics[] oneDofJoints1 = MultiBodySystemTools.filterJoints(source, OneDoFJointBasics.class);
      OneDoFJointBasics[] oneDofJoints2 = MultiBodySystemTools.filterJoints(destination, OneDoFJointBasics.class);

      if (oneDofJoints1.length != oneDofJoints2.length)
      {
         throw new IllegalArgumentException("The lists of joints must be the same length. %d != %d".formatted(oneDofJoints1.length,
                                                                                                              oneDofJoints2.length));
      }

      for (int i = 0; i < oneDofJoints1.length; i++)
      {
         oneDofJoints2[i].setJointConfiguration(oneDofJoints1[i]);
      }
   }

   public static List<JointBasics> getSubtreeJointsIncludingFourBars(RigidBodyBasics rootBody)
   {
      List<JointBasics> joints = new ArrayList<>();

      for (JointBasics joint : rootBody.childrenSubtreeIterable())
      {
         if (joint instanceof CrossFourBarJoint)
         {
            joints.addAll(((CrossFourBarJoint) joint).getFourBarFunction().getLoopJoints());
         }
         else
         {
            joints.add(joint);
         }
      }

      return joints;
   }

   public static MultiBodySystemBasics createSingleBodySystem(RigidBodyBasics singleBody)
   {
      List<? extends JointBasics> allJoints = new ArrayList<>();
      List<? extends JointBasics> jointsToConsider = new ArrayList<>();
      List<? extends JointBasics> jointsToIgnore = new ArrayList<>();
      JointMatrixIndexProvider jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(jointsToConsider);

      return new MultiBodySystemBasics()
      {
         @Override
         public RigidBodyBasics getRootBody()
         {
            return singleBody;
         }

         @Override
         public List<? extends JointBasics> getAllJoints()
         {
            return allJoints;
         }

         @Override
         public List<? extends JointBasics> getJointsToConsider()
         {
            return jointsToConsider;
         }

         @Override
         public List<? extends JointBasics> getJointsToIgnore()
         {
            return jointsToIgnore;
         }

         @Override
         public JointMatrixIndexProvider getJointMatrixIndexProvider()
         {
            return jointMatrixIndexProvider;
         }
      };
   }
}
