package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Provides a unique interface to configure multi-body algorithms.
 * <p>
 * Requiring only a {@code MultiBodySystemReadOnly} indicates that the consuming class or algorithm
 * will <b>not</b> modify the state of the given system internally. In occasion where the consumer
 * needs to update the state internally, {@link MultiBodySystemBasics} is required instead.
 * </p>
 * <p>
 * It provides the necessary information about what part of the multi-body system is to be
 * considered, and which to be ignored.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface MultiBodySystemReadOnly
{
   /**
    * Gets the inertial frame to use with this multi-body system.
    * <p>
    * The inertial frame is used as the principal referential when measuring the motion of an object in
    * space. It is usually set to the root frame of the reference frame tree this system is attached
    * to.
    * </p>
    *
    * @return the inertial frame.
    */
   default ReferenceFrame getInertialFrame()
   {
      return getRootBody().getBodyFixedFrame().getRootFrame();
   }

   /**
    * Gets the root body of a multi-body system or the start body of a subtree that the algorithms
    * consider.
    * 
    * @return the body support subtree or multi-body system to consider.
    */
   RigidBodyReadOnly getRootBody();

   /**
    * Gets all the joints composing the subtree starting at {@link #getRootBody()} including both the
    * joints to consider and to ignore.
    * 
    * @return the list containing all the joints.
    */
   default List<? extends JointReadOnly> getAllJoints()
   {
      return SubtreeStreams.fromChildren(getRootBody()).collect(Collectors.toList());
   }

   /**
    * Gets all the joints to be considered in the algorithm.
    * <p>
    * The ordering of returned list should be consistent with
    * {@link JointMatrixIndexProvider#getIndexedJointsInOrder()}.
    * </p>
    * 
    * @return the list of all the joints to be considered.
    */
   default List<? extends JointReadOnly> getJointsToConsider()
   {
      return getAllJoints();
   }

   /**
    * Gets all the joints to be ignored in the algorithm.
    * 
    * @return the list of all the joints to be ignored.
    */
   default List<? extends JointReadOnly> getJointsToIgnore()
   {
      return getAllJoints().stream().filter(joint -> !getJointsToConsider().contains(joint)).collect(Collectors.toList());
   }

   /**
    * Gets the {@code JointMatrixIndexProvider} to use with this input.
    * 
    * @return the matrix index provider for the considered joints.
    */
   default JointMatrixIndexProvider getJointMatrixIndexProvider()
   {
      return JointMatrixIndexProvider.toIndexProvider(getJointsToConsider());
   }

   /**
    * Creates a new input from the given {@code rootBody}. The resulting input will consider all the
    * joints composing the subtree starting off the given body.
    * 
    * @param rootBody the support body to the subtree to consider. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemReadOnly toMultiBodySystemInput(RigidBodyReadOnly rootBody)
   {
      return toMultiBodySystemInput(rootBody, Collections.emptyList());
   }

   /**
    * Creates a new input from the given {@code rootBody}. The resulting input will consider all the
    * joints composing the subtree starting off the given body less the given joints to ignore and
    * their respective descendants.
    * 
    * @param rootBody the support body to the subtree to consider. Not modified.
    * @param jointsToIgnore the array of joints to ignore. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemReadOnly toMultiBodySystemInput(RigidBodyReadOnly rootBody, JointReadOnly[] jointsToIgnore)
   {
      return toMultiBodySystemInput(rootBody, Arrays.asList(jointsToIgnore));
   }

   /**
    * Creates a new input from the given {@code rootBody}. The resulting input will consider all the
    * joints composing the subtree starting off the given body less the given joints to ignore and
    * their respective descendants.
    * 
    * @param rootBody the support body to the subtree to consider. Not modified.
    * @param jointsToIgnore the array of joints to ignore. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemReadOnly toMultiBodySystemInput(RigidBodyReadOnly rootBody, List<? extends JointReadOnly> jointsToIgnore)
   {
      List<? extends JointReadOnly> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      List<? extends JointReadOnly> jointsToConsider = extractJointsToConsider(rootBody, jointsToIgnore);
      JointMatrixIndexProvider jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(jointsToConsider);

      return new MultiBodySystemReadOnly()
      {
         @Override
         public RigidBodyReadOnly getRootBody()
         {
            return rootBody;
         }

         @Override
         public List<? extends JointReadOnly> getAllJoints()
         {
            return allJoints;
         }

         @Override
         public List<? extends JointReadOnly> getJointsToConsider()
         {
            return jointsToConsider;
         }

         @Override
         public List<? extends JointReadOnly> getJointsToIgnore()
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

   /**
    * Creates a new input from the given joints to consider.
    * <p>
    * The resulting root body and joints to ignore are automatically evaluated.
    * </p>
    * 
    * @param jointsToConsider the joints to consider. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemReadOnly toMultiBodySystemInput(JointReadOnly[] jointsToConsider)
   {
      return toMultiBodySystemInput(Arrays.asList(jointsToConsider));
   }

   /**
    * Creates a new input from the given joints to consider.
    * <p>
    * The resulting root body and joints to ignore are automatically evaluated.
    * </p>
    * 
    * @param jointsToConsider the joints to consider. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemReadOnly toMultiBodySystemInput(List<? extends JointReadOnly> jointsToConsider)
   {
      RigidBodyReadOnly rootBody = getClosestJointToRoot(jointsToConsider).getPredecessor();
      List<? extends JointReadOnly> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      List<? extends JointReadOnly> jointsToIgnore = SubtreeStreams.fromChildren(rootBody).filter(joint -> !jointsToConsider.contains(joint))
                                                                   .collect(Collectors.toList());
      JointMatrixIndexProvider jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(jointsToConsider);

      return new MultiBodySystemReadOnly()
      {
         @Override
         public RigidBodyReadOnly getRootBody()
         {
            return rootBody;
         }

         @Override
         public List<? extends JointReadOnly> getAllJoints()
         {
            return allJoints;
         }

         @Override
         public List<? extends JointReadOnly> getJointsToConsider()
         {
            return jointsToConsider;
         }

         @Override
         public List<? extends JointReadOnly> getJointsToIgnore()
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

   /**
    * Navigates through the subtree starting off of {@code rootBody} and collects all the joints that
    * are to be considered.
    * <p>
    * A joint is ignored if it is in the given list {@code jointsToIgnore} or it is a descendant of
    * another joint to ignore.
    * </p>
    * 
    * @param rootBody the supporting body of the subtree to collect joints from. Not modified.
    * @param jointsToIgnore the list of joints to ignore. Not modified.
    * @return the list of joints to consider.
    */
   public static List<? extends JointReadOnly> extractJointsToConsider(RigidBodyReadOnly rootBody, List<? extends JointReadOnly> jointsToIgnore)
   {
      return SubtreeStreams.fromChildren(rootBody).filter(candidate -> !isJointToBeIgnored(candidate, jointsToIgnore)).collect(Collectors.toList());
   }

   /**
    * Tests whether the given {@code query} is to be ignored.
    * <p>
    * A joint is ignored if it is in the given list {@code jointsToIgnore} or it is a descendant of
    * another joint to ignore.
    * </p>
    * 
    * @param query the joint to test. Not modified.
    * @param jointsToIgnore the list of joints to ignore. Not modified.
    * @return {@code true} if the query is to be ignored, {@code false} otherwise.
    */
   public static boolean isJointToBeIgnored(JointReadOnly query, List<? extends JointReadOnly> jointsToIgnore)
   {
      for (int i = 0; i < jointsToIgnore.size(); i++)
      {
         JointReadOnly jointToIgnore = jointsToIgnore.get(i);
         if (MultiBodySystemTools.isAncestor(query.getSuccessor(), jointToIgnore.getSuccessor()))
            return true;
      }
      return false;
   }

   /**
    * Finds amongst the given {@code joints} which is the closest to the root body.
    * 
    * @param joints the list of joints to search. Not modified.
    * @return the closest joint to the root body.
    */
   public static JointReadOnly getClosestJointToRoot(List<? extends JointReadOnly> joints)
   {
      JointReadOnly closest = joints.get(0);
      RigidBodyReadOnly rootBody = MultiBodySystemTools.getRootBody(closest.getPredecessor());
      int distanceToRoot = MultiBodySystemTools.computeDistanceToAncestor(closest.getPredecessor(), rootBody);

      for (int i = 1; i < joints.size(); i++)
      {
         JointReadOnly candidate = joints.get(i);
         int candidateDistanceToRoot = MultiBodySystemTools.computeDistanceToAncestor(candidate.getPredecessor(), rootBody);
         if (candidateDistanceToRoot < distanceToRoot)
         {
            distanceToRoot = candidateDistanceToRoot;
            closest = candidate;
         }
      }
      return closest;
   }
}
