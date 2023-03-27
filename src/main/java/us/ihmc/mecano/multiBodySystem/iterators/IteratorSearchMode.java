package us.ihmc.mecano.multiBodySystem.iterators;

/**
 * This enum can be used to tweak the behavior of the iterators {@link JointIterator} and
 * {@link RigidBodyIterator}.
 * 
 * @author Sylvain Bertrand
 */
public enum IteratorSearchMode
{
   /**
    * In this mode, the iterator will prioritize iterating through the full depth of a tree branch
    * before going the the next.
    */
   DEPTH_FIRST_SEARCH,
   /**
    * In this mode, the iterator will prioritize iterating through a depth level of the kinematics
    * before going deeper into the branches.
    */
   BREADTH_FIRST_SEARCH;
}
