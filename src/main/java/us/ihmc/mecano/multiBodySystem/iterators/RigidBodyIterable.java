package us.ihmc.mecano.multiBodySystem.iterators;

import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.function.Predicate;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

/**
 * {@code RigidBodyIterable} is a generic iterable that can be used on any implementation of
 * {@code RigidBodyReadOnly}.
 * <p>
 * This iterable can be used to iterate through the all the bodies of the subtree that starts at
 * {@code root}.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <B> the type of the {@code Iterable}.
 */
public class RigidBodyIterable<B extends RigidBodyReadOnly> implements Iterable<B>
{
   private final Collection<? extends RigidBodyReadOnly> roots;
   private final Predicate<B> selectionRule;
   private final Class<B> filteringClass;

   /**
    * Creates a new iterable for a single subtree.
    * 
    * @param filteringClass the class of the type of rigid-body to iterate through. If a rigid-body is
    *           not an instance of the {@code filteringClass}, then it will not be part of the
    *           iteration.
    * @param selectionRule rule to filter the rigid-bodies to iterate through. Rigid-bodies for which
    *           {@code selectionRule.test(body)} returns {@code false} are ignored and will not be part
    *           of the iteration.
    * @param root rigid-body from which the subtree starts. Not modified.
    */
   public RigidBodyIterable(Class<B> filteringClass, Predicate<B> selectionRule, B root)
   {
      this.selectionRule = selectionRule;
      this.roots = Collections.singleton(root);
      this.filteringClass = filteringClass;
   }

   /**
    * Creates a new iterable for multiple subtrees.
    * 
    * @param filteringClass the class of the type of rigid-body to iterate through. If a rigid-body is
    *           not an instance of the {@code filteringClass}, then it will not be part of the
    *           iteration.
    * @param selectionRule rule to filter the rigid-bodies to iterate through. Rigid-bodies for which
    *           {@code selectionRule.test(body)} returns {@code false} are ignored and will not be part
    *           of the iteration.
    * @param roots rigid-bodies from which each subtree starts. Not modified.
    */
   public RigidBodyIterable(Class<B> filteringClass, Predicate<B> selectionRule, Collection<? extends RigidBodyReadOnly> roots)
   {
      this.filteringClass = filteringClass;
      this.selectionRule = selectionRule;
      this.roots = roots;
   }

   /** {@inheritDoc} */
   @Override
   public Iterator<B> iterator()
   {
      return new RigidBodyIterator<>(filteringClass, selectionRule, roots);
   }

   /**
    * Creates a {@code Stream} representative of this {@code RigidBodyIterable}.
    * 
    * @return the subtree stream.
    */
   public Stream<B> toStream()
   {
      return StreamSupport.stream(spliterator(), false);
   }
}
