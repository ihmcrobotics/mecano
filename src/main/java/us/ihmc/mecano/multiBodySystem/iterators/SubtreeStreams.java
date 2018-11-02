package us.ihmc.mecano.multiBodySystem.iterators;

import java.util.Collection;
import java.util.Collections;
import java.util.stream.Stream;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

/**
 * {@code SubtreeStreams} provides {@code Stream} support for traversing joints and rigid-bodies of
 * a multi-body system.
 * 
 * @author Sylvain Bertrand
 */
public class SubtreeStreams
{
   /**
    * Creates a new {@code Stream} that consists of the entire subtree starting from the given
    * {@code root}.
    * 
    * @param root the start of the subtree. Not modified.
    * @return the new subtree stream.
    */
   public static Stream<JointReadOnly> from(JointReadOnly root)
   {
      return from(JointReadOnly.class, root);
   }

   /**
    * Creates a new {@code Stream} for multiple subtrees.
    * 
    * @param roots the start of each subtree. Not modified.
    * @return the new subtree stream.
    */
   public static Stream<JointReadOnly> from(Collection<? extends JointReadOnly> roots)
   {
      return from(JointReadOnly.class, roots);
   }

   /**
    * Creates a new {@code Stream} that consists of the entire subtree starting from the given
    * {@code root}.
    * 
    * @param root the start of the subtree. Not modified.
    * @return the new subtree stream.
    */
   public static Stream<JointBasics> from(JointBasics root)
   {
      return from(JointBasics.class, root);
   }

   /**
    * Creates a new {@code Stream} that consists of the entire subtree starting from the given
    * {@code root}.
    * 
    * @param filteringClass the class of the type of joint to stream. If a joint is not an instance of
    *           the {@code filteringClass}, then it will not be part of the stream.
    * @param root the start of the subtree. Not modified.
    * @return the new subtree stream.
    */
   public static <J extends JointReadOnly> Stream<J> from(Class<J> filteringClass, J root)
   {
      return from(filteringClass, Collections.singleton(root));
   }

   /**
    * Creates a new {@code Stream} for multiple subtrees.
    * 
    * @param filteringClass the class of the type of joint to stream. If a joint is not an instance of
    *           the {@code filteringClass}, then it will not be part of the stream.
    * @param roots the start of each subtree. Not modified.
    * @return the new subtree stream.
    */
   public static <J extends JointReadOnly> Stream<J> from(Class<J> filteringClass, Collection<? extends JointReadOnly> roots)
   {
      return new JointIterable<>(filteringClass, null, roots).toStream();
   }

   /**
    * Creates a new {@code Stream} that consists of the entire subtree starting from the given
    * {@code root}.
    * 
    * @param root the start of the subtree. Not modified.
    * @return the new subtree stream.
    */
   public static Stream<RigidBodyReadOnly> from(RigidBodyReadOnly root)
   {
      return from(RigidBodyReadOnly.class, root);
   }

   /**
    * Creates a new {@code Stream} that consists of all the joint subtrees for each of the {@code root}
    * children.
    * 
    * @param root the root of the joint subtree.
    * @return the new joint subtree stream.
    */
   public static Stream<JointReadOnly> fromChildren(RigidBodyReadOnly root)
   {
      return from(JointReadOnly.class, root.getChildrenJoints());
   }

   /**
    * Creates a new {@code Stream} that consists of the entire subtree starting from the given
    * {@code root}.
    * 
    * @param root the start of the subtree. Not modified.
    * @return the new subtree stream.
    */
   public static Stream<RigidBodyBasics> from(RigidBodyBasics root)
   {
      return from(RigidBodyBasics.class, root);
   }

   /**
    * Creates a new {@code Stream} that consists of all the joint subtrees for each of the {@code root}
    * children.
    * 
    * @param root the root of the joint subtree.
    * @return the new joint subtree stream.
    */
   public static Stream<JointBasics> fromChildren(RigidBodyBasics root)
   {
      return from(JointBasics.class, root.getChildrenJoints());
   }

   /**
    * Creates a new {@code Stream} that consists of all the joint subtrees for each of the {@code root}
    * children.
    * 
    * @param filteringClass the class of the type of joint to stream. If a joint is not an instance of
    *           the {@code filteringClass}, then it will not be part of the stream.
    * @param root the root of the joint subtree.
    * @return the new joint subtree stream.
    */
   public static <J extends JointReadOnly> Stream<J> fromChildren(Class<J> filteringClass, RigidBodyReadOnly root)
   {
      return from(filteringClass, root.getChildrenJoints());
   }

   /**
    * Creates a new {@code Stream} that consists of the entire subtree starting from the given
    * {@code root}.
    * 
    * @param filteringClass the class of the type of joint to stream. If a joint is not an instance of
    *           the {@code filteringClass}, then it will not be part of the stream.
    * @param root the start of the subtree. Not modified.
    * @return the new subtree stream.
    */
   public static <B extends RigidBodyReadOnly> Stream<B> from(Class<B> filteringClass, B root)
   {
      return new RigidBodyIterable<>(filteringClass, null, root).toStream();
   }
}
