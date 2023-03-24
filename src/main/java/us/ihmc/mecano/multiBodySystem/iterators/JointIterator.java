package us.ihmc.mecano.multiBodySystem.iterators;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Deque;
import java.util.Iterator;
import java.util.List;
import java.util.function.Predicate;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * {@code JointIterator} is a generic iterator that can be used on any implementation of
 * {@code JointReadOnly}.
 * <p>
 * This iterator can be used to iterate through the all the joints of the subtree that starts at
 * {@code root}.
 * </p>
 *
 * @author Sylvain Bertrand
 * @param <J> the type of the {@code Iterable}.
 */
@SuppressWarnings("unchecked")
public class JointIterator<J extends JointReadOnly> implements Iterator<J>
{
   private final Deque<JointReadOnly> stack = new ArrayDeque<>();
   private final Predicate<JointReadOnly> selectionRule;
   private final List<JointReadOnly> roots = new ArrayList<>();
   private final IteratorSearchMode mode;

   /**
    * Creates a new iterator for multiple subtrees.
    *
    * @param filteringClass the class of the type of joint to iterate through. If a joint is not an
    *                       instance of the {@code filteringClass}, then it will not be part of the
    *                       iteration.
    * @param selectionRule  rule to filter the joints to iterate through. Joints for which
    *                       {@code selectionRule.test(joint)} returns {@code false} are ignored and
    *                       will not be part of the iteration. Can be {@code null}.
    * @param mode           how the search should be conducted, either depth-first search, or
    *                       breadth-first search. Can be {@code null}.
    * @param root           joint from which the subtree starts. Not modified.
    */
   public JointIterator(Class<J> filteringClass, Predicate<J> selectionRule, IteratorSearchMode mode, JointReadOnly root)
   {
      this(filteringClass, selectionRule, mode, Collections.singleton(root));
   }

   /**
    * Creates a new iterator for multiple subtrees.
    *
    * @param filteringClass the class of the type of joint to iterate through. If a joint is not an
    *                       instance of the {@code filteringClass}, then it will not be part of the
    *                       iteration.
    * @param selectionRule  rule to filter the joints to iterate through. Joints for which
    *                       {@code selectionRule.test(joint)} returns {@code false} are ignored and
    *                       will not be part of the iteration. Can be {@code null}.
    * @param mode           how the search should be conducted, either depth-first search, or
    *                       breadth-first search. Can be {@code null}.
    * @param roots          joints from which each subtree starts. Not modified.
    */
   public JointIterator(Class<J> filteringClass, Predicate<J> selectionRule, IteratorSearchMode mode, Collection<? extends JointReadOnly> roots)
   {
      if (selectionRule == null)
         this.selectionRule = joint -> filteringClass.isInstance(joint);
      else
         this.selectionRule = joint -> filteringClass.isInstance(joint) && selectionRule.test((J) joint);

      if (mode == null)
         this.mode = IteratorSearchMode.DEPTH_FIRST_SEARCH;
      else
         this.mode = mode;

      if (roots != null)
      {
         stack.addAll(roots);
         this.roots.addAll(roots);
      }
   }

   private J next = null;
   private boolean hasNextHasBeenCalled = false;

   @Override
   public boolean hasNext()
   {
      next = null;

      if (stack.isEmpty())
         return false;

      if (!hasNextHasBeenCalled)
      {
         next = searchNextJointPassingRule();
         hasNextHasBeenCalled = true;
      }
      return next != null;
   }

   @Override
   public J next()
   {
      if (!hasNextHasBeenCalled)
      {
         if (!hasNext())
            throw new NullPointerException();
      }

      hasNextHasBeenCalled = false;
      J ret = next;
      next = null;
      return ret;
   }

   private J searchNextJointPassingRule()
   {
      while (!stack.isEmpty())
      {
         JointReadOnly currentJoint = searchNextJoint();
         if (currentJoint != null && selectionRule.test(currentJoint))
            return (J) currentJoint;
      }
      return null;
   }

   private JointReadOnly searchNextJoint()
   {
      JointReadOnly currentJoint = stack.poll();

      RigidBodyReadOnly successor = currentJoint.getSuccessor();

      if (currentJoint.isLoopClosure())
      { // Determine if the primary branch of the loop is included in the iteration.
         for (int rootIndex = 0; rootIndex < roots.size(); rootIndex++)
         {
            if (MultiBodySystemTools.isAncestor(successor, roots.get(rootIndex).getSuccessor()))
            { // The primary branch is included in the iteration, no need to recurse further down.
               return currentJoint;
            }
         }
      }

      if (successor != null)
      {
         List<? extends JointReadOnly> childrenJoints = successor.getChildrenJoints();

         if (childrenJoints != null)
         {
            switch (mode)
            {
               case DEPTH_FIRST_SEARCH:
               {
                  for (int i = childrenJoints.size() - 1; i >= 0; i--)
                  {
                     stack.offerFirst(childrenJoints.get(i));
                  }
                  break;
               }
               case BREADTH_FIRST_SEARCH:
               {
                  stack.addAll(childrenJoints);
                  break;
               }
               default:
               {
                  throw new IllegalArgumentException("Unexpected value: " + mode);
               }
            }
         }
      }

      return currentJoint;
   }
}