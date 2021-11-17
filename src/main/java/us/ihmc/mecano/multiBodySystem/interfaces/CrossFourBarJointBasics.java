package us.ihmc.mecano.multiBodySystem.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface CrossFourBarJointBasics extends CrossFourBarJointReadOnly, OneDoFJointBasics
{
   @Override
   RevoluteJointBasics getMasterJoint();

   @Override
   RevoluteJointBasics getJointA();

   @Override
   RevoluteJointBasics getJointB();

   @Override
   RevoluteJointBasics getJointC();

   @Override
   RevoluteJointBasics getJointD();

   @Override
   void updateFrame();

   @Override
   void updateMotionSubspace();

   @Override
   default void setQ(double q)
   {
      getMasterJoint().setQ(computeMasterJointQ(q));
   }

   @Override
   default void setQd(double qd)
   {
      getMasterJoint().setQd(computeMasterJointQd(qd));
   }

   @Override
   default void setQdd(double qdd)
   {
      getMasterJoint().setQdd(computeMasterJointQdd(qdd));
   }

   @Override
   default void setTau(double tau)
   {
      getJointA().setJointTauToZero();
      getJointB().setJointTauToZero();
      getJointC().setJointTauToZero();
      getJointD().setJointTauToZero();
      getMasterJoint().setTau(computeMasterJointTau(tau));
   }

   @Override
   default void setJointPosition(Tuple3DReadOnly jointPosition)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   default void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
      setQd(jointAngularVelocity.dot(getJointAxis()));
   }

   @Override
   default void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   default void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
      setQdd(jointAngularAcceleration.dot(getJointAxis()));
   }

   @Override
   default void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   default void setJointTorque(Vector3DReadOnly jointTorque)
   {
      setTau(jointTorque.dot(getJointAxis()));
   }

   @Override
   default void setJointForce(Vector3DReadOnly jointForce)
   {
      // This joint type behaves more like a revolute joint.
   }

   @Override
   default void setJointLimitLower(double jointLimitLower)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   default void setJointLimitUpper(double jointLimitUpper)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   default void setVelocityLimitLower(double velocityLimitLower)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   default void setVelocityLimitUpper(double velocityLimitUpper)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   default void setEffortLimitLower(double effortLimitLower)
   {
      getMasterJoint().setEffortLimitLower(effortLimitLower);
   }

   @Override
   default void setEffortLimitUpper(double effortLimitUpper)
   {
      getMasterJoint().setEffortLimitUpper(effortLimitUpper);
   }
}
