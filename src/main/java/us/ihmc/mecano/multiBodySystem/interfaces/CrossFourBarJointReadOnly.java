package us.ihmc.mecano.multiBodySystem.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface CrossFourBarJointReadOnly extends OneDoFJointReadOnly
{
   RevoluteJointReadOnly getMasterJoint();

   RevoluteJointReadOnly getJointA();

   RevoluteJointReadOnly getJointB();

   RevoluteJointReadOnly getJointC();

   RevoluteJointReadOnly getJointD();

   double computeMasterJointQ(double q);

   double computeMasterJointQd(double qd);

   double computeMasterJointQdd(double qdd);

   double computeMasterJointTau(double tau);

   DMatrix getLoopJacobian();

   DMatrix getLoopConvectiveTerm();

   @Override
   default boolean isMotionSubspaceVariable()
   {
      return true;
   }

   @Override
   default FrameVector3DReadOnly getJointAxis()
   {
      return getMasterJoint().getJointAxis();
   }

   @Override
   default double getQ()
   {
      return getJointA().getQ() + getJointD().getQ();
   }

   @Override
   default double getQd()
   {
      return getJointA().getQd() + getJointD().getQd();
   }

   @Override
   default double getQdd()
   {
      return getJointA().getQdd() + getJointD().getQdd();
   }

   @Override
   default double getJointLimitLower()
   {
      return getJointA().getJointLimitLower() + getJointD().getJointLimitLower();
   }

   @Override
   default double getJointLimitUpper()
   {
      return getJointA().getJointLimitUpper() + getJointD().getJointLimitUpper();
   }

   @Override
   default double getVelocityLimitLower()
   {
      return getJointA().getVelocityLimitLower() + getJointD().getVelocityLimitLower();
   }

   @Override
   default double getVelocityLimitUpper()
   {
      return getJointA().getVelocityLimitUpper() + getJointD().getVelocityLimitUpper();
   }

   @Override
   default double getEffortLimitLower()
   {
      return getMasterJoint().getEffortLimitLower();
   }

   @Override
   default double getEffortLimitUpper()
   {
      return getMasterJoint().getEffortLimitUpper();
   }
}
