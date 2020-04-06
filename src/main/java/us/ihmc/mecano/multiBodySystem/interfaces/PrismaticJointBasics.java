package us.ihmc.mecano.multiBodySystem.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a prismatic joint.
 * <p>
 * A prismatic joint is a joint has 1 degree of freedom of translation.
 * </p>
 * 
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface PrismaticJointBasics extends PrismaticJointReadOnly, OneDoFJointBasics
{
   /**
    * This method is ineffective for prismatic joints.
    */
   @Override
   default void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointPosition(Tuple3DReadOnly jointPosition)
   {
      setQ(TupleTools.dot(getJointAxis(), jointPosition));
   }

   /**
    * This method is ineffective for prismatic joints.
    */
   @Override
   default void setJointAngularVelocity(Vector3DReadOnly jointAngularVelocity)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLinearVelocity(Vector3DReadOnly jointLinearVelocity)
   {
      setQd(getJointAxis().dot(jointLinearVelocity));
   }

   /**
    * This method is ineffective for prismatic joints.
    */
   @Override
   default void setJointAngularAcceleration(Vector3DReadOnly jointAngularAcceleration)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointLinearAcceleration(Vector3DReadOnly jointLinearAcceleration)
   {
      setQdd(getJointAxis().dot(jointLinearAcceleration));
   }

   /**
    * This method is ineffective for prismatic joints.
    */
   @Override
   default void setJointTorque(Vector3DReadOnly jointTorque)
   {
   }

   /** {@inheritDoc} */
   @Override
   default void setJointForce(Vector3DReadOnly jointForce)
   {
      setTau(getJointAxis().dot(jointForce));
   }
}
