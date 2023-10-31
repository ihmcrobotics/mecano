package us.ihmc.mecano.yoVariables;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.fourBar.RevoluteTwinsJointBasicsTest;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoRevoluteTwinsJoint;

import java.util.Random;

public class YoRevoluteTwinsJointTest extends RevoluteTwinsJointBasicsTest<YoRevoluteTwinsJoint>
{
   @Override
   public YoRevoluteTwinsJoint nextRevoluteTwinsJoint(Random random, String name)
   {
      RigidBody rootBody = new RigidBody(name + "RootBody", ReferenceFrame.getWorldFrame());
      return nextRevoluteTwinsJoint(random, name, EuclidCoreRandomTools.nextUnitVector3D(random), rootBody);
   }

   public static YoRevoluteTwinsJoint nextRevoluteTwinsJoint(Random random, String name, Vector3DReadOnly jointAxis, RigidBodyBasics predecessor)
   {
      RigidBodyTransform transformAToPredecessor = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform transformBToA = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      int actuatedJointIndex = random.nextInt(2);
      double constraintRatio = EuclidCoreRandomTools.nextDouble(random, 0.25, 1.5);
      double constraintOffset = EuclidCoreRandomTools.nextDouble(random, -1.0, 1.0);
      YoRevoluteTwinsJoint joint = new YoRevoluteTwinsJoint(name,
                                                            predecessor,
                                                            null,
                                                            null,
                                                            null,
                                                            transformAToPredecessor,
                                                            transformBToA,
                                                            null,
                                                            0.0,
                                                            null,
                                                            actuatedJointIndex,
                                                            constraintRatio,
                                                            constraintOffset,
                                                            jointAxis,
                                                            null);
      MultiBodySystemRandomTools.nextRigidBody(random, "endEffector", joint);
      return joint;
   }
}
