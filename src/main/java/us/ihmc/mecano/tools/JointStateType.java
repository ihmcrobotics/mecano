package us.ihmc.mecano.tools;

/**
 * Enumeration of the different possible states that a joint has.
 * <p>
 * Mostly used to specify the state target for methods reading or modifying the state of joints.
 * </p>
 */
public enum JointStateType
{
   /**
    * Refers to the position and/or orientation of a joint.
    */
   CONFIGURATION,
   /**
    * Refers to the angular and/or linear velocity of a joint.
    */
   VELOCITY,
   /**
    * Refers to the angular and/or linear acceleration of a joint.
    */
   ACCELERATION,
   /**
    * Refers to the moment and/or force of a joint.
    */
   EFFORT
}