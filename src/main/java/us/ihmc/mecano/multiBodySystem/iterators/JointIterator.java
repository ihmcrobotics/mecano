package us.ihmc.mecano.multiBodySystem.iterators;

import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Deque;
import java.util.Iterator;
import java.util.List;
import java.util.function.Predicate;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

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

   /**
    * Creates a new iterator for multiple subtrees.
    * 
    * @param filteringClass the class of the type of joint to iterate through. If a joint is not an
    *                       instance of the {@code filteringClass}, then it will not be part of the
    *                       iteration.
    * @param selectionRule  rule to filter the joints to iterate through. Joints for which
    *                       {@code selectionRule.test(joint)} returns {@code false} are ignored and
    *                       will not be part of the iteration. Can be {@code null}.
    * @param root           joint from which the subtree starts. Not modified.
    */
   public JointIterator(Class<J> filteringClass, Predicate<J> selectionRule, JointReadOnly root)
   {
      if (selectionRule == null)
         this.selectionRule = joint -> filteringClass.isInstance(joint);
      else
         this.selectionRule = joint -> filteringClass.isInstance(joint) && selectionRule.test((J) joint);

      if (root != null)
         stack.add(root);
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
    * @param roots          joints from which each subtree starts. Not modified.
    */
   public JointIterator(Class<J> filteringClass, Predicate<J> selectionRule, Collection<? extends JointReadOnly> roots)
   {
      if (selectionRule == null)
         this.selectionRule = joint -> filteringClass.isInstance(joint);
      else
         this.selectionRule = joint -> filteringClass.isInstance(joint) && selectionRule.test((J) joint);

      if (roots != null)
         stack.addAll(roots);
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
         if (currentJoint == null || selectionRule.test(currentJoint))
            return (J) currentJoint;
      }
      return null;
   }

   private JointReadOnly searchNextJoint()
   {
      if (stack.isEmpty())
         return null;

      JointReadOnly currentJoint = stack.poll();

      RigidBodyReadOnly successor = currentJoint.getSuccessor();

      if (successor != null)
      {
         List<? extends JointReadOnly> childrenJoints = successor.getChildrenJoints();

         if (childrenJoints != null)
            stack.addAll(childrenJoints);
      }

      return currentJoint;
   }
}