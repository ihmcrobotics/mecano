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
 * {@code RigidBodyIterator} is a generic iterator that can be used on any implementation of
 * {@code RigidBodyReadOnly}.
 * <p>
 * This iterator can be used to iterate through the all the rigid-bodies of the subtree that starts
 * at {@code root}.
 * </p>
 * 
 * @author Sylvain Bertrand
 * @param <B> the type of the {@code Iterable}.
 */
@SuppressWarnings("unchecked")
public class RigidBodyIterator<B extends RigidBodyReadOnly> implements Iterator<B>
{
   private final Deque<RigidBodyReadOnly> stack = new ArrayDeque<>();
   private final Predicate<RigidBodyReadOnly> selectionRule;

   /**
    * Creates a new iterable for a single subtree.
    * 
    * @param filteringClass the class of the type of rigid-body to iterate through. If a rigid-body is
    *                       not an instance of the {@code filteringClass}, then it will not be part of
    *                       the iteration.
    * @param selectionRule  rule to filter the rigid-bodies to iterate through. Rigid-bodies for which
    *                       {@code selectionRule.test(body)} returns {@code false} are ignored and will
    *                       not be part of the iteration.
    * @param root           rigid-body from which the subtree starts. Not modified.
    */
   public RigidBodyIterator(Class<B> filteringClass, Predicate<B> selectionRule, RigidBodyReadOnly root)
   {
      if (selectionRule == null)
         this.selectionRule = body -> filteringClass.isInstance(body);
      else
         this.selectionRule = body -> filteringClass.isInstance(body) && selectionRule.test((B) body);

      if (root != null)
         stack.add(root);
   }

   /**
    * Creates a new iterator for multiple subtrees.
    * 
    * @param filteringClass the class of the type of rigid-body to iterate through. If a rigid-body is
    *                       not an instance of the {@code filteringClass}, then it will not be part of
    *                       the iteration.
    * @param selectionRule  rule to filter the rigid-bodies to iterate through. Rigid-bodies for which
    *                       {@code selectionRule.test(body)} returns {@code false} are ignored and will
    *                       not be part of the iteration.
    * @param roots          rigid-bodies from which each subtree starts. Not modified.
    */
   public RigidBodyIterator(Class<B> filteringClass, Predicate<B> selectionRule, Collection<? extends RigidBodyReadOnly> roots)
   {
      if (selectionRule == null)
         this.selectionRule = body -> filteringClass.isInstance(body);
      else
         this.selectionRule = body -> filteringClass.isInstance(body) && selectionRule.test((B) body);

      if (roots != null)
         stack.addAll(roots);
   }

   private B next = null;
   private boolean hasNextHasBeenCalled = false;

   @Override
   public boolean hasNext()
   {
      next = null;

      if (stack.isEmpty())
         return false;

      if (!hasNextHasBeenCalled)
      {
         next = searchNextRigidBodyPassingRule();
         hasNextHasBeenCalled = true;
      }
      return next != null;
   }

   @Override
   public B next()
   {
      if (!hasNextHasBeenCalled)
      {
         if (!hasNext())
            throw new NullPointerException();
      }

      hasNextHasBeenCalled = false;
      B ret = next;
      next = null;
      return ret;
   }

   private B searchNextRigidBodyPassingRule()
   {
      while (!stack.isEmpty())
      {
         RigidBodyReadOnly currentBody = searchNextRigidBody();
         if (currentBody == null || selectionRule.test(currentBody))
            return (B) currentBody;
      }
      return null;
   }

   private RigidBodyReadOnly searchNextRigidBody()
   {
      if (stack.isEmpty())
         return null;

      RigidBodyReadOnly currentBody = stack.poll();

      List<? extends JointReadOnly> childrenJoints = currentBody.getChildrenJoints();

      if (childrenJoints != null)
      {
         for (JointReadOnly childJoint : childrenJoints)
         {
            RigidBodyReadOnly childBody = childJoint.getSuccessor();
            stack.add(childBody);
         }
      }

      return currentBody;
   }
}