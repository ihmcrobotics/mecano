package us.ihmc.mecano.multiBodySystem.iterators;

import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.function.Predicate;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

/**
 * {@code JointIterable} is a generic iterable that can be used on any implementation of
 * {@code JointReadOnly}.
 * <p>
 * This iterable can be used to iterate through the all the joints of the subtree that starts at
 * {@code root}.
 * </p>
 *
 * @author Sylvain Bertrand
 * @param <J> the type of the {@code Iterable}.
 */
public class JointIterable<J extends JointReadOnly> implements Iterable<J>
{
   private final Collection<? extends JointReadOnly> roots;
   private final Predicate<J> selectionRule;
   private final Class<J> filteringClass;
   private final IteratorSearchMode mode;

   /**
    * Creates a new iterable for a single subtree.
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
   public JointIterable(Class<J> filteringClass, Predicate<J> selectionRule, IteratorSearchMode mode, JointReadOnly root)
   {
      this(filteringClass, selectionRule, mode, Collections.singletonList(root));
   }

   /**
    * Creates a new iterable for multiple subtrees.
    *
    * @param filteringClass the class of the type of joint to iterate through. If a joint is not an
    *                       instance of the {@code filteringClass}, then it will not be part of the
    *                       iteration.
    * @param selectionRule  rule to filter the joints to iterate through. Joints for which
    *                       {@code selectionRule.test(joint)} returns {@code false} are ignored and
    *                       will not be part of the iteration. Can be {@code null}.
    * @param mode           how the search should be conducted, either depth-first search, or
    *                       breadth-first search. Can be {@code null}.
    * @param rootBody       rigid-body from which the subtree starts. Not modified.
    */
   public JointIterable(Class<J> filteringClass, Predicate<J> selectionRule, IteratorSearchMode mode, RigidBodyReadOnly rootBody)
   {
      this(filteringClass, selectionRule, mode, rootBody.getChildrenJoints());
   }

   /**
    * Creates a new iterable for multiple subtrees.
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
   public JointIterable(Class<J> filteringClass, Predicate<J> selectionRule, IteratorSearchMode mode, Collection<? extends JointReadOnly> roots)
   {
      this.filteringClass = filteringClass;
      this.selectionRule = selectionRule;
      this.mode = mode;
      this.roots = roots;
   }

   /** {@inheritDoc} */
   @Override
   public Iterator<J> iterator()
   {
      return new JointIterator<>(filteringClass, selectionRule, mode, roots);
   }

   /**
    * Creates a {@code Stream} representative of this {@code JointIterable}.
    *
    * @return the subtree stream.
    */
   public Stream<J> toStream()
   {
      return StreamSupport.stream(spliterator(), false);
   }
}
