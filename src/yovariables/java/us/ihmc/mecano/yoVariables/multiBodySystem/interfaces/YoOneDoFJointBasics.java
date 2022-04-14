package us.ihmc.mecano.yoVariables.multiBodySystem.interfaces;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.variable.YoDouble;

public interface YoOneDoFJointBasics extends OneDoFJointBasics
{
   /** {@inheritDoc} */
   @Override
   default void setQ(double q)
   {
      getYoQ().set(q);
   }

   /** {@inheritDoc} */
   @Override
   default void setQd(double qd)
   {
      getYoQd().set(qd);
   }

   /** {@inheritDoc} */
   @Override
   default void setQdd(double qdd)
   {
      getYoQdd().set(qdd);
   }

   /** {@inheritDoc} */
   @Override
   default void setTau(double tau)
   {
      getYoTau().set(tau);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLimitLower(double jointLimitLower)
   {
      getYoJointLimitLower().set(jointLimitLower);
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLimitUpper(double jointLimitUpper)
   {
      getYoJointLimitUpper().set(jointLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   default void setVelocityLimitLower(double velocityLimitLower)
   {
      getYoVelocityLimitLower().set(velocityLimitLower);
   }

   /** {@inheritDoc} */
   @Override
   default void setVelocityLimitUpper(double velocityLimitUpper)
   {
      getYoVelocityLimitUpper().set(velocityLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   default void setEffortLimitLower(double effortLimitLower)
   {
      getYoEffortLimitLower().set(effortLimitLower);
   }

   /** {@inheritDoc} */
   @Override
   default void setEffortLimitUpper(double effortLimitUpper)
   {
      getYoEffortLimitUpper().set(effortLimitUpper);
   }

   /** {@inheritDoc} */
   @Override
   default double getQ()
   {
      return getYoQ().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getQd()
   {
      return getYoQd().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getQdd()
   {
      return getYoQdd().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getTau()
   {
      return getYoTau().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getJointLimitLower()
   {
      return getYoJointLimitLower().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getJointLimitUpper()
   {
      return getYoJointLimitUpper().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getVelocityLimitLower()
   {
      return getYoVelocityLimitLower().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getVelocityLimitUpper()
   {
      return getYoVelocityLimitUpper().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getEffortLimitLower()
   {
      return getYoEffortLimitLower().getValue();
   }

   /** {@inheritDoc} */
   @Override
   default double getEffortLimitUpper()
   {
      return getYoEffortLimitUpper().getValue();
   }

   /**
    * Gets the yoVariable used to store this joint position.
    *
    * @return the position's yoVariable.
    */
   YoDouble getYoQ();

   /**
    * Gets the yoVariable used to store this joint velocity.
    *
    * @return the velocity's yoVariable.
    */
   YoDouble getYoQd();

   /**
    * Gets the yoVariable used to store this joint acceleration.
    *
    * @return the acceleration's yoVariable.
    */
   YoDouble getYoQdd();

   /**
    * Gets the yoVariable used to store this joint force/torque.
    *
    * @return the force/torque's yoVariable.
    */
   YoDouble getYoTau();

   /**
    * Gets the yoVariable used to store this joint lower limit.
    *
    * @return the lower limit's yoVariable.
    */
   YoDouble getYoJointLimitLower();

   /**
    * Gets the yoVariable used to store this joint upper limit.
    *
    * @return the upper limit's yoVariable.
    */
   YoDouble getYoJointLimitUpper();

   /**
    * Gets the yoVariable used to store this joint's velocity lower limit.
    *
    * @return the velocity lower limit's yoVariable.
    */
   YoDouble getYoVelocityLimitLower();

   /**
    * Gets the yoVariable used to store this joint's velocity upper limit.
    *
    * @return the velocity upper limit's yoVariable.
    */
   YoDouble getYoVelocityLimitUpper();

   /**
    * Gets the yoVariable used to store this joint's force/torque lower limit.
    *
    * @return the force/torque lower limit's yoVariable.
    */
   YoDouble getYoEffortLimitLower();

   /**
    * Gets the yoVariable used to store this joint's force/torque upper limit.
    *
    * @return the force/torque upper limit's yoVariable.
    */
   YoDouble getYoEffortLimitUpper();
}
