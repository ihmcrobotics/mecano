package us.ihmc.mecano.fourBar;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RevoluteTwinsJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;

public class RevoluteTwinsJointTest extends RevoluteTwinsJointBasicsTest<RevoluteTwinsJoint>
{

   @Override
   public RevoluteTwinsJoint nextRevoluteTwinsJoint(Random random, String name)
   {
      RigidBody rootBody = new RigidBody(name + "RootBody", ReferenceFrame.getWorldFrame());
      return nextRevoluteTwinsJoint(random, name, EuclidCoreRandomTools.nextUnitVector3D(random), rootBody);
   }

   public static RevoluteTwinsJoint nextRevoluteTwinsJoint(Random random, String name, Vector3DReadOnly jointAxis, RigidBodyBasics predecessor)
   {
      RigidBodyTransform transformAToPredecessor = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      RigidBodyTransform transformBToA = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      int actuatedJointIndex = random.nextInt(2);
      double constraintRatio = EuclidCoreRandomTools.nextDouble(random, 0.25, 1.5);
      double constraintOffset = EuclidCoreRandomTools.nextDouble(random, -1.0, 1.0);
      RevoluteTwinsJoint joint = new RevoluteTwinsJoint(name,
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
                                                        jointAxis);
      MultiBodySystemRandomTools.nextRigidBody(random, "endEffector", joint);
      return joint;
   }
}
