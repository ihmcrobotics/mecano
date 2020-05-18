package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemFactories;

/**
 * Provides a unique interface to configure multi-body algorithms.
 * <p>
 * Requiring a {@code MultiBodySystemBasics} as the input of a class or algorithm indicates that the
 * state of the given system will be updated internally. Classes and algorithms that do <b>not</b>
 * modify the system state internally only require {@link MultiBodySystemReadOnly}.
 * </p>
 * <p>
 * It provides the necessary information about what part of the multi-body system is to be
 * considered, and which to be ignored.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface MultiBodySystemBasics extends MultiBodySystemReadOnly
{
   /** {@inheritDoc} */
   @Override
   RigidBodyBasics getRootBody();

   /** {@inheritDoc} */
   @Override
   default List<? extends JointBasics> getAllJoints()
   {
      return SubtreeStreams.fromChildren(getRootBody()).collect(Collectors.toList());
   }

   /** {@inheritDoc} */
   @Override
   default List<? extends JointBasics> getJointsToConsider()
   {
      return getAllJoints();
   }

   /** {@inheritDoc} */
   @Override
   default List<? extends JointBasics> getJointsToIgnore()
   {
      return getAllJoints().stream().filter(joint -> !getJointsToConsider().contains(joint)).collect(Collectors.toList());
   }

   /**
    * Creates a new input from the given {@code rootBody}. The resulting input will consider all the
    * joints composing the subtree starting off the given body.
    *
    * @param rootBody the support body to the subtree to consider. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemBasics toMultiBodySystemBasics(RigidBodyBasics rootBody)
   {
      return toMultiBodySystemBasics(rootBody, Collections.emptyList());
   }

   /**
    * Creates a new input from the given {@code rootBody}. The resulting input will consider all the
    * joints composing the subtree starting off the given body less the given joints to ignore and
    * their respective descendants.
    *
    * @param rootBody       the support body to the subtree to consider. Not modified.
    * @param jointsToIgnore the array of joints to ignore. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemBasics toMultiBodySystemBasics(RigidBodyBasics rootBody, JointBasics[] jointsToIgnore)
   {
      return toMultiBodySystemBasics(rootBody, Arrays.asList(jointsToIgnore));
   }

   /**
    * Creates a new input from the given {@code rootBody}. The resulting input will consider all the
    * joints composing the subtree starting off the given body less the given joints to ignore and
    * their respective descendants.
    *
    * @param rootBody       the support body to the subtree to consider. Not modified.
    * @param jointsToIgnore the array of joints to ignore. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemBasics toMultiBodySystemBasics(RigidBodyBasics rootBody, List<? extends JointBasics> jointsToIgnore)
   {
      List<? extends JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      List<? extends JointBasics> jointsToConsider = extractJointsToConsider(rootBody, jointsToIgnore);
      JointMatrixIndexProvider jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(jointsToConsider);

      return new MultiBodySystemBasics()
      {
         @Override
         public RigidBodyBasics getRootBody()
         {
            return rootBody;
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

   /**
    * Creates a new input from the given joints to consider.
    * <p>
    * The resulting root body and joints to ignore are automatically evaluated.
    * </p>
    *
    * @param jointsToConsider the joints to consider. Not modified.
    * @return the new input.
    */
   public static MultiBodySystemBasics toMultiBodySystemBasics(JointBasics[] jointsToConsider)
   {
      return toMultiBodySystemBasics(Arrays.asList(jointsToConsider));
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
   public static MultiBodySystemBasics toMultiBodySystemBasics(List<? extends JointBasics> jointsToConsider)
   {
      RigidBodyBasics rootBody = (RigidBodyBasics) MultiBodySystemReadOnly.getClosestJointToRoot(jointsToConsider).getPredecessor();
      List<? extends JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      List<? extends JointBasics> jointsToIgnore = SubtreeStreams.fromChildren(rootBody).filter(joint -> !jointsToConsider.contains(joint))
                                                                 .collect(Collectors.toList());
      JointMatrixIndexProvider jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(jointsToConsider);

      return new MultiBodySystemBasics()
      {
         @Override
         public RigidBodyBasics getRootBody()
         {
            return rootBody;
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

   /**
    * Navigates through the subtree starting off of {@code rootBody} and collects all the joints that
    * are to be considered.
    * <p>
    * A joint is ignored if it is in the given list {@code jointsToIgnore} or it is a descendant of
    * another joint to ignore.
    * </p>
    *
    * @param rootBody       the supporting body of the subtree to collect joints from. Not modified.
    * @param jointsToIgnore the list of joints to ignore. Not modified.
    * @return the list of joints to consider.
    */
   public static List<? extends JointBasics> extractJointsToConsider(RigidBodyBasics rootBody, List<? extends JointBasics> jointsToIgnore)
   {
      return SubtreeStreams.fromChildren(rootBody).filter(candidate -> !MultiBodySystemReadOnly.isJointToBeIgnored(candidate, jointsToIgnore))
                           .collect(Collectors.toList());
   }

   /**
    * Performs a deep copy of {@code original}, preserving naming, root body, and the joints to ignore.
    * The clone is attached to the given {@code clonerootFrame}.
    *
    * @param original       the multi-body system to clone. Not modified.
    * @param cloneRootFrame the root frame to which the clone system is attached.
    * @return the clone.
    */
   public static MultiBodySystemBasics clone(MultiBodySystemReadOnly original, ReferenceFrame cloneRootFrame)
   {
      RigidBodyBasics cloneRootBody = MultiBodySystemFactories.cloneMultiBodySystem(original.getRootBody(), cloneRootFrame, "");
      Set<String> namesOfJointsToConsider = SubtreeStreams.fromChildren(original.getRootBody()).map(JointReadOnly::getName).collect(Collectors.toSet());
      List<? extends JointBasics> jointsToConsider = SubtreeStreams.fromChildren(cloneRootBody)
                                                                   .filter(joint -> namesOfJointsToConsider.contains(joint.getName()))
                                                                   .collect(Collectors.toList());
      return toMultiBodySystemBasics(jointsToConsider);
   }
}
