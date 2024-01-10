package us.ihmc.mecano.tools;

import org.ejml.data.DMatrix;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialInertia;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * This class provides a variety of tools to facilitate operations that need to navigate through a
 * multi-body system.
 */
public class MultiBodySystemTools
{
   /**
    * Sums the inertia of all the rigid-bodies composing the subtree that originates at {@code joint}
    * including {@code joint.getSuccessor()}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joint the root of the subtree.
    * @return the subtree total inertia.
    */
   public static SpatialInertia computeSubtreeInertia(JointReadOnly joint)
   {
      return computeSubtreeInertia(joint.getSuccessor());
   }

   /**
    * Sums the inertia of all the rigid-bodies composing the subtree that originates at
    * {@code rootBody} including {@code rootBody}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rootBody the root of the subtree.
    * @return the subtree total inertia.
    */
   public static SpatialInertia computeSubtreeInertia(RigidBodyReadOnly rootBody)
   {
      MovingReferenceFrame bodyFixedFrame = rootBody.getBodyFixedFrame();

      SpatialInertia subtreeInertia = new SpatialInertia(bodyFixedFrame, bodyFixedFrame);
      SpatialInertia bodyInertia = new SpatialInertia();

      for (RigidBodyReadOnly subtreeBody : rootBody.subtreeList())
      {
         bodyInertia.setIncludingFrame(subtreeBody.getInertia());
         bodyInertia.changeFrame(bodyFixedFrame);
         subtreeInertia.add(bodyInertia);
      }

      return subtreeInertia;
   }

   /**
    * Retrieves and gets the root body of the multi-body system the given {@code body} belongs to.
    *
    * @param body an arbitrary body that belongs to the multi-body system that this method is to find
    *             the root.
    * @return the root body.
    */
   public static RigidBodyReadOnly getRootBody(RigidBodyReadOnly body)
   {
      RigidBodyReadOnly root = body;

      while (root.getParentJoint() != null)
      {
         root = root.getParentJoint().getPredecessor();
      }

      return root;
   }

   /**
    * Retrieves and gets the root body of the multi-body system the given {@code body} belongs to.
    *
    * @param body an arbitrary body that belongs to the multi-body system that this method is to find
    *             the root.
    * @return the root body.
    */
   public static RigidBodyBasics getRootBody(RigidBodyBasics body)
   {
      RigidBodyBasics root = body;

      while (root.getParentJoint() != null)
      {
         root = root.getParentJoint().getPredecessor();
      }

      return root;
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that implement {@code OneDoFJointBasics} and that are in between and return them as an array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param start the rigid-body from where to begin the collection of joints.
    * @param end   the rigid-body where to stop the collection of joints.
    * @return the array of joints representing the path from {@code start} to {@code end}.
    */
   public static OneDoFJointBasics[] createOneDoFJointPath(RigidBodyBasics start, RigidBodyBasics end)
   {
      return filterJoints(createJointPath(start, end), OneDoFJointBasics.class);
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that are in between and return them as an array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param start the rigid-body from where to begin the collection of joints.
    * @param end   the rigid-body where to stop the collection of joints.
    * @return the array of joints representing the path from {@code start} to {@code end}, or
    *       {@code null} if the given rigid-bodies are not part of the same multi-body system.
    */
   public static JointBasics[] createJointPath(RigidBodyBasics start, RigidBodyBasics end)
   {
      List<JointBasics> jointPath = new ArrayList<>();
      collectJointPath(start, end, jointPath);
      return jointPath.toArray(new JointBasics[jointPath.size()]);
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that are in between and return them as an array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param start the rigid-body from where to begin the collection of joints.
    * @param end   the rigid-body where to stop the collection of joints.
    * @return the array of joints representing the path from {@code start} to {@code end}, or
    *       {@code null} if the given rigid-bodies are not part of the same multi-body system.
    */
   public static JointReadOnly[] createJointPath(RigidBodyReadOnly start, RigidBodyReadOnly end)
   {
      List<JointReadOnly> jointPath = new ArrayList<>();
      collectJointPath(start, end, jointPath);
      return jointPath.toArray(new JointReadOnly[jointPath.size()]);
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that are in between in the given {@code jointPathToPack}.
    * <p>
    * The resulting joint path represent the shortest path connecting {@code start} and {@code end}. No
    * assumption is made on the relative position of the two rigid-bodies in the multi-body system.
    * </p>
    *
    * @param start           the rigid-body where to begin collecting the joints.
    * @param end             the rigid-body where to stop collecting the joints.
    * @param jointPathToPack the list in which the joint path is stored. Note that the list is first
    *                        cleared before storing the joint path.
    * @return the nearest common ancestor of {@code start} and {@code end}.
    */
   public static RigidBodyReadOnly collectJointPath(RigidBodyReadOnly start, RigidBodyReadOnly end, List<JointReadOnly> jointPathToPack)
   {
      jointPathToPack.clear();

      RigidBodyReadOnly ancestor = computeNearestCommonAncestor(start, end);
      RigidBodyReadOnly currentBody;

      currentBody = start;

      while (currentBody != ancestor)
      {
         JointReadOnly parentJoint = currentBody.getParentJoint();
         jointPathToPack.add(parentJoint);
         currentBody = parentJoint.getPredecessor();
      }

      int distance = jointPathToPack.size();
      currentBody = end;

      while (currentBody != ancestor)
      {
         currentBody = currentBody.getParentJoint().getPredecessor();
         distance++;
      }

      while (jointPathToPack.size() < distance)
         jointPathToPack.add(null);

      currentBody = end;

      for (int i = distance - 1; currentBody != ancestor; i--)
      {
         JointReadOnly parentJoint = currentBody.getParentJoint();
         jointPathToPack.set(i, parentJoint);
         currentBody = parentJoint.getPredecessor();
      }
      return ancestor;
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the joints
    * that are in between in the given {@code jointPathToPack}.
    * <p>
    * The resulting joint path represent the shortest path connecting {@code start} and {@code end}. No
    * assumption is made on the relative position of the two rigid-bodies in the multi-body system.
    * </p>
    *
    * @param start           the rigid-body where to begin collecting the joints.
    * @param end             the rigid-body where to stop collecting the joints.
    * @param jointPathToPack the list in which the joint path is stored. Note that the list is first
    *                        cleared before storing the joint path.
    * @return the nearest common ancestor of {@code start} and {@code end}.
    */
   public static RigidBodyBasics collectJointPath(RigidBodyBasics start, RigidBodyBasics end, List<JointBasics> jointPathToPack)
   {
      jointPathToPack.clear();

      RigidBodyBasics ancestor = computeNearestCommonAncestor(start, end);
      RigidBodyBasics currentBody;

      currentBody = start;

      while (currentBody != ancestor)
      {
         JointBasics parentJoint = currentBody.getParentJoint();
         jointPathToPack.add(parentJoint);
         currentBody = parentJoint.getPredecessor();
      }

      int distance = jointPathToPack.size();
      currentBody = end;

      while (currentBody != ancestor)
      {
         currentBody = currentBody.getParentJoint().getPredecessor();
         distance++;
      }

      while (jointPathToPack.size() < distance)
         jointPathToPack.add(null);

      currentBody = end;

      for (int i = distance - 1; currentBody != ancestor; i--)
      {
         JointBasics parentJoint = currentBody.getParentJoint();
         jointPathToPack.set(i, parentJoint);
         currentBody = parentJoint.getPredecessor();
      }
      return ancestor;
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the
    * rigid-bodies that connect {@code start} to {@code end} in the given {@code rigidBodyPathToPack}.
    * <p>
    * The resulting resulting path includes both {@code start} and {@code end} and represent the
    * shortest path connecting the two rigid-bodies. No assumption is made on the relative position of
    * the two rigid-bodies in the multi-body system.
    * </p>
    *
    * @param start               the rigid-body where to begin collecting the rigid-bodies.
    * @param end                 the rigid-body where to stop collecting the rigid-bodies.
    * @param rigidBodyPathToPack the list in which the rigid-body path is stored. Note that the list is
    *                            first cleared before storing the rigid-body path.
    * @return the nearest common ancestor of {@code start} and {@code end}.
    */
   public static RigidBodyReadOnly collectRigidBodyPath(RigidBodyReadOnly start, RigidBodyReadOnly end, List<RigidBodyReadOnly> rigidBodyPathToPack)
   {
      rigidBodyPathToPack.clear();

      if (start == end)
      {
         rigidBodyPathToPack.add(end);
         return end;
      }

      RigidBodyReadOnly ancestor = computeNearestCommonAncestor(start, end);
      RigidBodyReadOnly currentBody;

      currentBody = start;

      if (start == ancestor)
         rigidBodyPathToPack.add(start);

      while (currentBody != ancestor)
      {
         rigidBodyPathToPack.add(currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      int distance = rigidBodyPathToPack.size();
      currentBody = end;

      while (currentBody != ancestor)
      {
         currentBody = currentBody.getParentJoint().getPredecessor();
         distance++;
      }

      while (rigidBodyPathToPack.size() < distance)
         rigidBodyPathToPack.add(null);

      currentBody = end;

      if (end == ancestor)
         rigidBodyPathToPack.add(end);

      for (int i = distance - 1; currentBody != ancestor; i--)
      {
         rigidBodyPathToPack.set(i, currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }
      return ancestor;
   }

   /**
    * Travels the multi-body system from {@code start} to {@code end} and stores in order the
    * rigid-bodies that connect {@code start} to {@code end} in the given {@code rigidBodyPathToPack}.
    * <p>
    * The resulting resulting path includes both {@code start} and {@code end} and represent the
    * shortest path connecting the two rigid-bodies. No assumption is made on the relative position of
    * the two rigid-bodies in the multi-body system.
    * </p>
    *
    * @param start               the rigid-body where to begin collecting the rigid-bodies.
    * @param end                 the rigid-body where to stop collecting the rigid-bodies.
    * @param rigidBodyPathToPack the list in which the rigid-body path is stored. Note that the list is
    *                            first cleared before storing the rigid-body path.
    * @return the nearest common ancestor of {@code start} and {@code end}.
    */
   public static RigidBodyBasics collectRigidBodyPath(RigidBodyBasics start, RigidBodyBasics end, List<RigidBodyBasics> rigidBodyPathToPack)
   {
      rigidBodyPathToPack.clear();

      if (start == end)
      {
         rigidBodyPathToPack.add(end);
         return end;
      }

      RigidBodyBasics ancestor = computeNearestCommonAncestor(start, end);
      RigidBodyBasics currentBody;

      currentBody = start;

      if (start == ancestor)
         rigidBodyPathToPack.add(start);

      while (currentBody != ancestor)
      {
         rigidBodyPathToPack.add(currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      int distance = rigidBodyPathToPack.size();
      currentBody = end;

      while (currentBody != ancestor)
      {
         currentBody = currentBody.getParentJoint().getPredecessor();
         distance++;
      }

      while (rigidBodyPathToPack.size() < distance)
         rigidBodyPathToPack.add(null);

      currentBody = end;

      if (end == ancestor)
         rigidBodyPathToPack.add(end);

      for (int i = distance - 1; currentBody != ancestor; i--)
      {
         rigidBodyPathToPack.set(i, currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }
      return ancestor;
   }

   /**
    * Traverses up the kinematic chain from the candidate descendant towards the root body, checking to
    * see if each parent body is the ancestor in question.
    *
    * @param candidateDescendant the query for the descendant. A rigid-body is the descendant of
    *                            another rigid-body if it is between the other rigid-body and an
    *                            end-effector.
    * @param ancestor            the query for the ancestor. A rigid-body is the ancestor of another
    *                            rigid-body if it is between the other rigid-body and the root body.
    * @return {@code true} if {@code candidateDescendant} is a descendant of {@code ancestor},
    *       {@code false} otherwise.
    */
   public static boolean isAncestor(RigidBodyReadOnly candidateDescendant, RigidBodyReadOnly ancestor)
   {
      RigidBodyReadOnly currentBody = candidateDescendant;
      while (!currentBody.isRootBody())
      {
         if (currentBody == ancestor)
         {
            return true;
         }
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      return currentBody == ancestor;
   }

   /**
    * Computes the number of joints that separates the {@code rigidBody} from its root body.
    *
    * @param rigidBody the query.
    * @return the distance in number of joints between the {@code rigidBody} and the root body. This
    *       method returns {@code 0} if the {@code rigidBody} is the root body.
    */
   public static int computeDistanceToRoot(RigidBodyReadOnly rigidBody)
   {
      int distance = 0;
      RigidBodyReadOnly currentBody = rigidBody;

      while (!currentBody.isRootBody())
      {
         distance++;
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      return distance;
   }

   /**
    * Computes the number of joints that separates the {@code descendant} from its {@code ancestor}.
    * <p>
    * The {@code ancestor} is expected to be located between the root body and the {@code descendant},
    * if not this method returns {@code -1}.
    * </p>
    *
    * @param descendant the descendant, often it is the end-effector.
    * @param ancestor   the ancestor of the descendant, often it is the root body.
    * @return the distance in number of joints between the {@code descendant} and the {@code ancestor}.
    *       This method returns {@code 0} if the two rigid-bodies are the same. This method returns
    *       {@code -1} if the given {@code ancestor} is not located between the {@code descendant}
    *       and the root body.
    */
   public static int computeDistanceToAncestor(RigidBodyReadOnly descendant, RigidBodyReadOnly ancestor)
   {
      int distance = 0;
      RigidBodyReadOnly currentBody = descendant;

      while (!currentBody.isRootBody() && currentBody != ancestor)
      {
         distance++;
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      if (currentBody != ancestor)
         distance = -1;

      return distance;
   }

   /**
    * Computes the number of joints that separates the two rigid-bodies {@code firstBody} and
    * {@code secondBody}.
    * <p>
    * Unlike {@link #computeDistanceToAncestor(RigidBodyReadOnly, RigidBodyReadOnly)}, no assumption is
    * made regarding the relative position of the two bodies within the multi-body system.
    * </p>
    *
    * @param firstBody  the first end of the kinematic chain to compute the distance of.
    * @param secondBody the second end of the kinematic chain to compute the distance of.
    * @return the distance in number of joints between the {@code firstBody} and the
    *       {@code secondBody}. This method returns {@code 0} if the two rigid-bodies are the same.
    * @throws IllegalArgumentException if the two rigid-bodies do not belong to the same multi-body
    *       system.
    */
   public static int computeDistance(RigidBodyReadOnly firstBody, RigidBodyReadOnly secondBody)
   {
      RigidBodyReadOnly ancestor = computeNearestCommonAncestor(firstBody, secondBody);
      return computeDistanceToAncestor(firstBody, ancestor) + computeDistanceToAncestor(secondBody, ancestor);
   }

   /**
    * Calculates the number of degrees of freedom of the kinematic chain that connects
    * {@code firstBody} and {@code secondBody}.
    *
    * @param firstBody  the first end of the kinematic chain.
    * @param secondBody the second end of the kinematic chain.
    * @return the number of degrees of freedom.
    * @throws IllegalArgumentException if the two rigid-bodies do not belong to the same multi-body
    *       system.
    */
   public static int computeDegreesOfFreedom(RigidBodyReadOnly firstBody, RigidBodyReadOnly secondBody)
   {
      int nDoFs = 0;

      RigidBodyReadOnly ancestor = computeNearestCommonAncestor(firstBody, secondBody);

      RigidBodyReadOnly currentBody = firstBody;

      while (currentBody != ancestor)
      {
         JointReadOnly parentJoint = currentBody.getParentJoint();
         nDoFs += parentJoint.getDegreesOfFreedom();
         currentBody = parentJoint.getPredecessor();
      }

      currentBody = secondBody;

      while (currentBody != ancestor)
      {
         JointReadOnly parentJoint = currentBody.getParentJoint();
         nDoFs += parentJoint.getDegreesOfFreedom();
         currentBody = parentJoint.getPredecessor();
      }

      return nDoFs;
   }

   /**
    * Iterates through the given joints and sums their number degrees of freedom.
    *
    * @param joints the kinematic chain to compute the total number of degrees of freedom of.
    * @return the total number of degrees of freedom.
    */
   public static int computeDegreesOfFreedom(List<? extends JointReadOnly> joints)
   {
      int numberOfDegreesOfFreedom = 0;

      for (int i = 0; i < joints.size(); i++)
      {
         numberOfDegreesOfFreedom += joints.get(i).getDegreesOfFreedom();
      }

      return numberOfDegreesOfFreedom;
   }

   /**
    * Iterates through the given joints and sums their number degrees of freedom.
    *
    * @param joints the kinematic chain to compute the total number of degrees of freedom of.
    * @return the total number of degrees of freedom.
    */
   public static int computeDegreesOfFreedom(JointReadOnly[] joints)
   {
      int numberOfDegreesOfFreedom = 0;

      for (int i = 0; i < joints.length; i++)
      {
         numberOfDegreesOfFreedom += joints[i].getDegreesOfFreedom();
      }

      return numberOfDegreesOfFreedom;
   }

   /**
    * Finds the common ancestor of {@code firstBody} and {@code secondBody} that minimizes the distance
    * {@code d}:
    *
    * <pre>
    * d = <i>computeDistanceToAncestor</i>(firstBody, ancestor) + <i>computeDistanceToAncestor</i>(secondBody, ancestor)
    * </pre>
    *
    * @param firstBody  the first rigid-body of the query.
    * @param secondBody the second rigid-body of the query.
    * @return the nearest common ancestor.
    * @throws IllegalArgumentException if the two rigid-bodies do not belong to the same multi-body
    *       system.
    */
   public static RigidBodyBasics computeNearestCommonAncestor(RigidBodyBasics firstBody, RigidBodyBasics secondBody)
   {
      return (RigidBodyBasics) computeNearestCommonAncestor((RigidBodyReadOnly) firstBody, (RigidBodyReadOnly) secondBody);
   }

   /**
    * Finds the common ancestor of {@code firstBody} and {@code secondBody} that minimizes the distance
    * {@code d}:
    *
    * <pre>
    * d = <i>computeDistanceToAncestor</i>(firstBody, ancestor) + <i>computeDistanceToAncestor</i>(secondBody, ancestor)
    * </pre>
    *
    * @param firstBody  the first rigid-body of the query.
    * @param secondBody the second rigid-body of the query.
    * @return the nearest common ancestor.
    * @throws IllegalArgumentException if the two rigid-bodies do not belong to the same multi-body
    *       system.
    */
   public static RigidBodyReadOnly computeNearestCommonAncestor(RigidBodyReadOnly firstBody, RigidBodyReadOnly secondBody)
   {
      if (firstBody == secondBody)
         return firstBody;

      RigidBodyReadOnly firstAncestor = firstBody;
      int firstDistanceToRoot = computeDistanceToRoot(firstBody);
      RigidBodyReadOnly secondAncestor = secondBody;
      int secondDistanceToRoot = computeDistanceToRoot(secondBody);

      int distanceToRoot;

      if (firstDistanceToRoot > secondDistanceToRoot)
      {
         distanceToRoot = firstDistanceToRoot;
         while (distanceToRoot > secondDistanceToRoot)
         {
            firstAncestor = firstAncestor.getParentJoint().getPredecessor();
            distanceToRoot--;
         }
      }
      else if (secondDistanceToRoot > firstDistanceToRoot)
      {
         distanceToRoot = secondDistanceToRoot;
         while (distanceToRoot > firstDistanceToRoot)
         {
            secondAncestor = secondAncestor.getParentJoint().getPredecessor();
            distanceToRoot--;
         }
      }
      else
      {
         distanceToRoot = firstDistanceToRoot;
      }

      if (firstAncestor == secondAncestor)
         return firstAncestor;

      // The multi-body system has a tree structure and the 2 bodies are on 2 distinct branches of that tree.
      // Knowing that both firstAncestor and secondAncestor are at the same distance from the root, we go up the tree until they match.

      while (distanceToRoot > 0)
      {
         firstAncestor = firstAncestor.getParentJoint().getPredecessor();
         secondAncestor = secondAncestor.getParentJoint().getPredecessor();

         if (firstAncestor == secondAncestor)
            return firstAncestor;

         distanceToRoot--;
      }

      // We are at the root and the ancestors still do not match => we are dealing with 2 distinct multi-body systems.
      throw new IllegalArgumentException(
            "The two rigid-bodies are not part of the same multi-body system: first root: " + firstAncestor.getName() + ", second root: "
            + secondAncestor.getName());
   }

   /**
    * Collects in order the successor of each joint, i.e. {@link JointReadOnly#getSuccessor()}.
    * <p>
    * Note on kinematic loops: if {@code joints} contains the two joints terminating a kinematic loop,
    * then the common successor will occur twice in the result.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints to collect the successors of.
    * @return the array containing in order the successor of each joint.
    */
   // TODO Consider explicitly handling successor of kinematic loops
   public static RigidBodyReadOnly[] collectSuccessors(JointReadOnly... joints)
   {
      return Stream.of(joints).map(JointReadOnly::getSuccessor).toArray(RigidBodyReadOnly[]::new);
   }

   /**
    * Collects in order the successor of each joint, i.e. {@link JointReadOnly#getSuccessor()}.
    * <p>
    * Note on kinematic loops: if {@code joints} contains the two joints terminating a kinematic loop,
    * then the common successor will occur twice in the result.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints to collect the successors of.
    * @return the array containing in order the successor of each joint.
    */
   // TODO Consider explicitly handling successor of kinematic loops
   public static RigidBodyBasics[] collectSuccessors(JointBasics... joints)
   {
      return Stream.of(joints).map(JointBasics::getSuccessor).toArray(RigidBodyBasics[]::new);
   }

   /**
    * Collects any rigid-body that composes any of the subtrees originating at the given
    * {@code joints}.
    * <p>
    * Note on kinematic loops: if {@code joints} contains the two joints terminating a kinematic loop,
    * then the subtree of the common successor will occur twice in the result.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints indicating the start of each subtree to collect.
    * @return the array containing all the rigid-bodies composing the subtrees.
    */
   // TODO Consider explicitly handling successor of kinematic loops
   public static RigidBodyReadOnly[] collectSubtreeSuccessors(JointReadOnly... joints)
   {
      return Stream.of(joints).map(JointReadOnly::getSuccessor).flatMap(RigidBodyReadOnly::subtreeStream).distinct().toArray(RigidBodyReadOnly[]::new);
   }

   /**
    * Collects any rigid-body that composes any of the subtrees originating at the given
    * {@code joints}.
    * <p>
    * Note on kinematic loops: if {@code joints} contains the two joints terminating a kinematic loop,
    * then the subtree of the common successor will occur twice in the result.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints indicating the start of each subtree to collect.
    * @return the array containing all the rigid-bodies composing the subtrees.
    */
   // TODO Consider explicitly handling successor of kinematic loops
   public static RigidBodyBasics[] collectSubtreeSuccessors(JointBasics... joints)
   {
      return Stream.of(joints).map(JointBasics::getSuccessor).flatMap(RigidBodyBasics::subtreeStream).distinct().toArray(RigidBodyBasics[]::new);
   }

   /**
    * Collects and returns all the joints located between the given {@code rigidBody} and the root
    * body.
    * <p>
    * Note on kinematic loops: this method does not collect the joints on the secondary branch of a
    * kinematic loop, i.e. the branch that starts off the primary branch and ends with the loop closure
    * joint.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect the support joints of.
    * @return the array containing the support joints of the given rigid-body.
    */
   public static JointReadOnly[] collectSupportJoints(RigidBodyReadOnly rigidBody)
   {
      return createJointPath(getRootBody(rigidBody), rigidBody);
   }

   /**
    * Collects and returns all the joints located between the given {@code rigidBody} and the root
    * body.
    * <p>
    * Note on kinematic loops: this method does not collect the joints on the secondary branch of a
    * kinematic loop, i.e. the branch that starts off the primary branch and ends with the loop closure
    * joint.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect the support joints of.
    * @return the array containing the support joints of the given rigid-body.
    */
   public static JointBasics[] collectSupportJoints(RigidBodyBasics rigidBody)
   {
      return createJointPath(getRootBody(rigidBody), rigidBody);
   }

   /**
    * Collects for each rigid-body all their support joints, i.e. the joints that are between the
    * rigid-body and the root body, and returns an array containing no duplicate elements.
    * <p>
    * Note on kinematic loops: this method does not collect the joints on the secondary branch of a
    * kinematic loop, i.e. the branch that starts off the primary branch and ends with the loop closure
    * joint.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBodies the rigid-bodies to collect the support joints of.
    * @return the array containing the support joints of all the given rigid-bodies.
    */
   public static JointReadOnly[] collectSupportJoints(RigidBodyReadOnly... rigidBodies)
   {
      return Stream.of(rigidBodies).map(MultiBodySystemTools::collectSupportJoints).flatMap(Stream::of).distinct().toArray(JointReadOnly[]::new);
   }

   /**
    * Collects for each rigid-body all their support joints, i.e. the joints that are between the
    * rigid-body and the root body, and returns an array containing no duplicate elements.
    * <p>
    * Note on kinematic loops: this method does not collect the joints on the secondary branch of a
    * kinematic loop, i.e. the branch that starts off the primary branch and ends with the loop closure
    * joint.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBodies the rigid-bodies to collect the support joints of.
    * @return the array containing the support joints of all the given rigid-bodies.
    */
   public static JointBasics[] collectSupportJoints(RigidBodyBasics... rigidBodies)
   {
      return Stream.of(rigidBodies).map(MultiBodySystemTools::collectSupportJoints).flatMap(Stream::of).distinct().toArray(JointBasics[]::new);
   }

   /**
    * Collects all the joints that are part of any of the subtrees originating from the given
    * {@code rootBodies}, and returns an array containing no duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rootBodies the rigid-bodies from which the subtree to collect start off.
    * @return the array containing all the joint composing the subtrees.
    */
   public static JointReadOnly[] collectSubtreeJoints(RigidBodyReadOnly... rootBodies)
   {
      return Stream.of(rootBodies).flatMap(SubtreeStreams::fromChildren).distinct().toArray(JointReadOnly[]::new);
   }

   /**
    * Collects all the joints that are part of any of the subtrees originating from the given
    * {@code rootBodies}, and returns an array containing no duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rootBodies the rigid-bodies from which the subtree to collect start off.
    * @return the array containing all the joint composing the subtrees.
    */
   public static JointBasics[] collectSubtreeJoints(RigidBodyBasics... rootBodies)
   {
      return Stream.of(rootBodies).flatMap(SubtreeStreams::fromChildren).distinct().toArray(JointBasics[]::new);
   }

   /**
    * Collects all the joints that are part of any of the subtrees originating from the given
    * {@code rootBodies}, and returns an array containing no duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rootBodies the rigid-bodies from which the subtree to collect start off.
    * @return the array containing all the joint composing the subtrees.
    */
   public static JointReadOnly[] collectSubtreeJoints(List<? extends RigidBodyReadOnly> rootBodies)
   {
      return rootBodies.stream().flatMap(SubtreeStreams::fromChildren).distinct().toArray(JointReadOnly[]::new);
   }

   /**
    * Combines {@link #collectSupportJoints(RigidBodyReadOnly)} with
    * {@link #collectSubtreeJoints(RigidBodyReadOnly...)}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect the support and subtree joints of.
    * @return the array containing the support and subtree joints.
    * @see #collectSupportJoints(RigidBodyReadOnly)
    * @see #collectSubtreeJoints(RigidBodyReadOnly...)
    */
   public static JointReadOnly[] collectSupportAndSubtreeJoints(RigidBodyReadOnly rigidBody)
   {
      List<JointReadOnly> supportAndSubtreeJoints = SubtreeStreams.fromChildren(rigidBody).collect(Collectors.toList());
      supportAndSubtreeJoints.addAll(Arrays.asList(collectSupportJoints(rigidBody)));
      return supportAndSubtreeJoints.toArray(new JointReadOnly[supportAndSubtreeJoints.size()]);
   }

   /**
    * Combines {@link #collectSupportJoints(RigidBodyBasics)} with
    * {@link #collectSubtreeJoints(RigidBodyBasics...)}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect the support and subtree joints of.
    * @return the array containing the support and subtree joints.
    * @see #collectSupportJoints(RigidBodyBasics)
    * @see #collectSubtreeJoints(RigidBodyBasics...)
    */
   public static JointBasics[] collectSupportAndSubtreeJoints(RigidBodyBasics rigidBody)
   {
      List<JointBasics> supportAndSubtreeJoints = new ArrayList<>();
      Stream.of(collectSupportJoints(rigidBody)).forEach(supportAndSubtreeJoints::add);
      rigidBody.childrenSubtreeIterable().forEach(supportAndSubtreeJoints::add);
      return supportAndSubtreeJoints.toArray(new JointBasics[supportAndSubtreeJoints.size()]);
   }

   /**
    * Combines {@link #collectSupportJoints(RigidBodyReadOnly...)} with
    * {@link #collectSubtreeJoints(RigidBodyReadOnly...)}, and returns an array containing no duplicate
    * elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBodies the rigid-bodies to collect the support and subtree joints of.
    * @return the array containing the support and subtree joints.
    * @see #collectSupportJoints(RigidBodyReadOnly)
    * @see #collectSubtreeJoints(RigidBodyReadOnly...)
    */
   public static JointReadOnly[] collectSupportAndSubtreeJoints(RigidBodyReadOnly... rigidBodies)
   {
      return Stream.of(rigidBodies).map(MultiBodySystemTools::collectSupportAndSubtreeJoints).flatMap(Stream::of).distinct().toArray(JointReadOnly[]::new);
   }

   /**
    * Combines {@link #collectSupportJoints(RigidBodyBasics...)} with
    * {@link #collectSubtreeJoints(RigidBodyBasics...)}, and returns an array containing no duplicate
    * elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBodies the rigid-bodies to collect the support and subtree joints of.
    * @return the array containing the support and subtree joints.
    * @see #collectSupportJoints(RigidBodyBasics)
    * @see #collectSubtreeJoints(RigidBodyBasics...)
    */
   public static JointBasics[] collectSupportAndSubtreeJoints(RigidBodyBasics... rigidBodies)
   {
      return Stream.of(rigidBodies).map(MultiBodySystemTools::collectSupportAndSubtreeJoints).flatMap(Stream::of).distinct().toArray(JointBasics[]::new);
   }

   /**
    * Collects starting from the given {@code rigidBody} all descendant that has no children, i.e. all
    * end-effector.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect of descendant end-effectors of.
    * @return the array containing the end-effectors.
    */
   public static RigidBodyBasics[] collectSubtreeEndEffectors(RigidBodyBasics rigidBody)
   {
      return rigidBody.subtreeStream().filter(body -> body.getChildrenJoints().isEmpty()).toArray(RigidBodyBasics[]::new);
   }

   /**
    * Collects starting from the given {@code rigidBody} all descendant that has no children, i.e. all
    * end-effector.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rigidBody the rigid-body to collect of descendant end-effectors of.
    * @return the array containing the end-effectors.
    */
   public static RigidBodyReadOnly[] collectSubtreeEndEffectors(RigidBodyReadOnly rigidBody)
   {
      return rigidBody.subtreeStream().filter(body -> body.getChildrenJoints().isEmpty()).toArray(RigidBodyReadOnly[]::new);
   }

   /**
    * Collects only the joints from {@code source} that are instances of the given {@code clazz} and
    * stores them in {@code destination}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param <T>    the joint type to be filtered.
    * @param source the original collection of joints to filter. Not modified.
    * @param clazz  the class that the filtered joints have to implement.
    * @return the filtered joints.
    */
   public static <T extends JointReadOnly> List<T> filterJoints(List<? extends JointReadOnly> source, Class<T> clazz)
   {
      List<T> filteredJoints = new ArrayList<>();
      filterJoints(source, filteredJoints, clazz);
      return filteredJoints;
   }

   /**
    * Collects only the joints from {@code source} that are instances of the given {@code clazz} and
    * stores them in {@code destination}.
    * <p>
    * The filtered joints are added to the end of {@code destination} using {@link List#add(Object)}.
    * </p>
    *
    * @param <T>         the joint type to be filtered.
    * @param source      the original collection of joints to filter. Not modified.
    * @param destination the collection where to store the filtered joints. Modified.
    * @param clazz       the class that the filtered joints have to implement.
    * @return the number of joints that implement the given class.
    */
   @SuppressWarnings("unchecked")
   public static <T extends JointReadOnly> int filterJoints(List<? extends JointReadOnly> source, List<T> destination, Class<T> clazz)
   {
      int numberOfFilteredJoints = 0;

      for (int i = 0; i < source.size(); i++)
      {
         JointReadOnly joint = source.get(i);

         if (clazz.isAssignableFrom(joint.getClass()))
         {
            destination.add((T) joint);
            numberOfFilteredJoints++;
         }
      }

      return numberOfFilteredJoints;
   }

   /**
    * Collects and returns only the joints from {@code source} that are instances of the given
    * {@code clazz}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param <T>    the joint type to be filtered.
    * @param source the original array of joints to filter. Not modified.
    * @param clazz  the class that the filtered joints have to implement.
    * @return the array containing the filtered joints.
    */
   public static <T extends JointReadOnly> T[] filterJoints(JointReadOnly[] source, Class<T> clazz)
   {
      @SuppressWarnings("unchecked") T[] retArray = (T[]) Array.newInstance(clazz, computeNumberOfJointsOfType(clazz, source));
      filterJoints(source, retArray, clazz);
      return retArray;
   }

   /**
    * Collects and returns only the joints from {@code source} that are instances of the given
    * {@code clazz}.
    * <p>
    * This method writes the filtered joints in {@code destination} starting at index {@code 0}
    * overwriting any element previously stored in the array.
    * </p>
    *
    * @param <T>         the joint type to be filtered.
    * @param source      the original array of joints to filter. Not modified.
    * @param destination the array where to store the filtered joints. Modified.
    * @param clazz       the class that the filtered joints have to implement.
    * @return the number of joints that implement the given class.
    */
   @SuppressWarnings("unchecked")
   public static <T extends JointReadOnly> int filterJoints(JointReadOnly[] source, T[] destination, Class<T> clazz)
   {
      int numberOfFilteredJoints = 0;

      for (int i = 0; i < source.length; i++)
      {
         JointReadOnly joint = source[i];

         if (clazz.isAssignableFrom(joint.getClass()))
            destination[numberOfFilteredJoints++] = (T) joint;
      }

      return numberOfFilteredJoints;
   }

   /**
    * Finds and returns the first joint contained in the subtree starting at the given rigid-body which
    * name matches the given name.
    *
    * @param start     where to start the search, if unsure, the root body should be given. Not
    *                  modified.
    * @param jointName the name of the joint to be found.
    * @return the joint matching the given name, or {@code null} if no joint with such name could be
    *       found.
    */
   public static JointReadOnly findJoint(RigidBodyReadOnly start, String jointName)
   {
      return findJoint(start, jointName, false);
   }

   /**
    * Finds and returns the first joint contained in the subtree starting at the given rigid-body which
    * name matches the given name.
    *
    * @param start      where to start the search, if unsure, the root body should be given. Not
    *                   modified.
    * @param jointName  the name of the joint to be found.
    * @param ignoreCase whether to ignore the case when comparing the name, see
    *                   {@link String#equalsIgnoreCase(String)}.
    * @return the joint matching the given name, or {@code null} if no joint with such name could be
    *       found.
    */
   public static JointReadOnly findJoint(RigidBodyReadOnly start, String jointName, boolean ignoreCase)
   {
      List<? extends JointReadOnly> childrenJoints = start.getChildrenJoints();

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         JointReadOnly childJoint = childrenJoints.get(i);

         if (ignoreCase)
         {
            if (childJoint.getName().equalsIgnoreCase(jointName))
               return childJoint;
         }
         else
         {
            if (childJoint.getName().equals(jointName))
               return childJoint;
         }
      }

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         JointReadOnly result = findJoint(childrenJoints.get(i).getSuccessor(), jointName, ignoreCase);
         if (result != null)
            return result;
      }

      return null;
   }

   /**
    * Finds and returns the first joint contained in the subtree starting at the given rigid-body which
    * name matches the given name.
    *
    * @param start     where to start the search, if unsure, the root body should be given. Not
    *                  modified.
    * @param jointName the name of the joint to be found.
    * @return the joint matching the given name, or {@code null} if no joint with such name could be
    *       found.
    */
   public static JointBasics findJoint(RigidBodyBasics start, String jointName)
   {
      return findJoint(start, jointName, false);
   }

   /**
    * Finds and returns the first joint contained in the subtree starting at the given rigid-body which
    * name matches the given name.
    *
    * @param start      where to start the search, if unsure, the root body should be given. Not
    *                   modified.
    * @param jointName  the name of the joint to be found.
    * @param ignoreCase whether to ignore the case when comparing the name, see
    *                   {@link String#equalsIgnoreCase(String)}.
    * @return the joint matching the given name, or {@code null} if no joint with such name could be
    *       found.
    */
   public static JointBasics findJoint(RigidBodyBasics start, String jointName, boolean ignoreCase)
   {
      return (JointBasics) findJoint((RigidBodyReadOnly) start, jointName, ignoreCase);
   }

   /**
    * Finds and returns the first rigid-body contained in the subtree (including {@code start})
    * starting at the given rigid-body which name matches the given name.
    *
    * @param start         where to start the search, if unsure, the root body should be given. Not
    *                      modified.
    * @param rigidBodyName the name of the rigid-body to be found.
    * @return the rigid-body matching the given name, or {@code null} if no rigid-body with such name
    *       could be found.
    */
   public static RigidBodyReadOnly findRigidBody(RigidBodyReadOnly start, String rigidBodyName)
   {
      return findRigidBody(start, rigidBodyName, false);
   }

   /**
    * Finds and returns the first rigid-body contained in the subtree (including {@code start})
    * starting at the given rigid-body which name matches the given name.
    *
    * @param start         where to start the search, if unsure, the root body should be given. Not
    *                      modified.
    * @param rigidBodyName the name of the rigid-body to be found.
    * @param ignoreCase    whether to ignore the case when comparing the name, see
    *                      {@link String#equalsIgnoreCase(String)}.
    * @return the rigid-body matching the given name, or {@code null} if no rigid-body with such name
    *       could be found.
    */
   public static RigidBodyReadOnly findRigidBody(RigidBodyReadOnly start, String rigidBodyName, boolean ignoreCase)
   {
      if (start == null)
         return null;

      if (ignoreCase)
      {
         if (start.getName().equalsIgnoreCase(rigidBodyName))
            return start;
      }
      else
      {
         if (start.getName().equals(rigidBodyName))
            return start;
      }

      List<? extends JointReadOnly> childrenJoints = start.getChildrenJoints();

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         RigidBodyReadOnly result = findRigidBody(childrenJoints.get(i).getSuccessor(), rigidBodyName, ignoreCase);
         if (result != null)
            return result;
      }

      return null;
   }

   /**
    * Finds and returns the first rigid-body contained in the subtree (including {@code start})
    * starting at the given rigid-body which name matches the given name.
    *
    * @param start         where to start the search, if unsure, the root body should be given. Not
    *                      modified.
    * @param rigidBodyName the name of the rigid-body to be found.
    * @return the rigid-body matching the given name, or {@code null} if no rigid-body with such name
    *       could be found.
    */
   public static RigidBodyBasics findRigidBody(RigidBodyBasics start, String rigidBodyName)
   {
      return findRigidBody(start, rigidBodyName, false);
   }

   /**
    * Finds and returns the first rigid-body contained in the subtree (including {@code start})
    * starting at the given rigid-body which name matches the given name.
    *
    * @param start         where to start the search, if unsure, the root body should be given. Not
    *                      modified.
    * @param rigidBodyName the name of the rigid-body to be found.
    * @param ignoreCase    whether to ignore the case when comparing the name, see
    *                      {@link String#equalsIgnoreCase(String)}.
    * @return the rigid-body matching the given name, or {@code null} if no rigid-body with such name
    *       could be found.
    */
   public static RigidBodyBasics findRigidBody(RigidBodyBasics start, String rigidBodyName, boolean ignoreCase)
   {
      return (RigidBodyBasics) findRigidBody((RigidBodyReadOnly) start, rigidBodyName, ignoreCase);
   }

   /**
    * Tests that the given {@code joints} are stored in order from root to leaf and represent a
    * continuous kinematic chain.
    * <p>
    * More precisely:
    * <ul>
    * <li>the first joint of the array should be the closest or equal to the root body of the
    * multi-body system they belong.
    * <li>the last joint of the array should be the closest or equal to one of the leaves or
    * end-effectors of the multi-body system they belong.
    * <li>the continuity test asserts that each pair of successive joints in the given array are also
    * successive in the kinematic chain:
    * {@code joints[i] == joints[i+1].getPredecessor().getParentJoint()} &forall; i &in; [0;
    * {@code joints.length - 1}].
    * </ul>
    * </p>
    *
    * @param joints the query. Not modified.
    * @return {@code true} if the joints are stored in a continuous manner from root to leaf,
    *       {@code false} otherwise.
    */
   public static boolean areJointsInContinuousOrder(JointReadOnly[] joints)
   {
      for (int index = 0; index < joints.length - 1; index++)
      {
         if (joints[index] != joints[index + 1].getPredecessor().getParentJoint())
            return false;
      }

      return true;
   }

   /**
    * Tests that the given {@code joints} are stored in order from root to leaf and represent a
    * continuous kinematic chain.
    * <p>
    * More precisely:
    * <ul>
    * <li>the first joint of the list should be the closest or equal to the root body of the multi-body
    * system they belong.
    * <li>the last joint of the list should be the closest or equal to one of the leaves or
    * end-effectors of the multi-body system they belong.
    * <li>the continuity test asserts that each pair of successive joints in the given list are also
    * successive in the kinematic chain:
    * {@code joints.get(i) == joints.get(i+1).getPredecessor().getParentJoint()} &forall; i &in; [0;
    * {@code joints.size() - 1}].
    * </ul>
    * </p>
    *
    * @param joints the query. Not modified.
    * @return {@code true} if the joints are stored in a continuous manner from root to leaf,
    *       {@code false} otherwise.
    */
   public static boolean areJointsInContinuousOrder(List<? extends JointReadOnly> joints)
   {
      for (int index = 0; index < joints.size() - 1; index++)
      {
         if (joints.get(index) != joints.get(index + 1).getPredecessor().getParentJoint())
            return false;
      }

      return true;
   }

   /**
    * Tests whether the subtree starting at the given rigid-body contains at least one loop closure or
    * not.
    *
    * @param start the rigid-body from which the subtree to test starts.
    * @return {@code true} if the subtree contains one or more loop closure, {@code false} if there is
    *       no loop closure.
    * @see JointReadOnly#isLoopClosure()
    */
   public static boolean doesSubtreeContainLoopClosure(RigidBodyReadOnly start)
   {
      for (int i = 0; i < start.getChildrenJoints().size(); i++)
      {
         if (start.getChildrenJoints().get(i).isLoopClosure())
            return true;
      }

      for (int i = 0; i < start.getChildrenJoints().size(); i++)
      {
         if (doesSubtreeContainLoopClosure(start.getChildrenJoints().get(i).getSuccessor()))
            return true;
      }

      return false;
   }

   /**
    * Iterates through the given array and compute how many do implement the given {@code clazz}.
    *
    * @param <T>    the joint type to be filtered.
    * @param clazz  the query for the joint type.
    * @param joints the array containing the joints to be tested.
    * @return the number of joints in the array that implement the given class.
    */
   public static <T extends JointReadOnly> int computeNumberOfJointsOfType(Class<T> clazz, JointReadOnly[] joints)
   {
      int number = 0;
      for (JointReadOnly joint : joints)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
            number++;
      }

      return number;
   }

   /**
    * Copies the requested state from the {@code source} joints to the {@code destination} joints.
    * <p>
    * The two lists should be of same length and each pair (source, destination) joint for any given
    * index should be of the same type.
    * </p>
    *
    * @param source         the joints holding the state to copy over. Not modified.
    * @param destination    the joints which state is to be be updated. Modified.
    * @param stateSelection the state that is to be copied over.
    */
   public static void copyJointsState(List<? extends JointReadOnly> source, List<? extends JointBasics> destination, JointStateType stateSelection)
   {
      if (source.size() != destination.size())
         throw new IllegalArgumentException("Inconsistent argument size: source = " + source.size() + ", destination = " + destination.size() + ".");

      switch (stateSelection)
      {
         case CONFIGURATION:
            copyJointsConfiguration(source, destination);
            return;
         case VELOCITY:
            copyJointsVelocity(source, destination);
            return;
         case ACCELERATION:
            copyJointsAcceleration(source, destination);
            return;
         case EFFORT:
            copyJointsTau(source, destination);
            return;
         default:
            throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static void copyJointsConfiguration(List<? extends JointReadOnly> source, List<? extends JointBasics> destination)
   {
      for (int jointIndex = 0; jointIndex < source.size(); jointIndex++)
      {
         JointReadOnly sourceJoint = source.get(jointIndex);
         JointBasics destinationJoint = destination.get(jointIndex);
         destinationJoint.setJointConfiguration(sourceJoint);
      }
   }

   private static void copyJointsVelocity(List<? extends JointReadOnly> source, List<? extends JointBasics> destination)
   {
      for (int jointIndex = 0; jointIndex < source.size(); jointIndex++)
      {
         JointReadOnly sourceJoint = source.get(jointIndex);
         JointBasics destinationJoint = destination.get(jointIndex);
         destinationJoint.setJointTwist(sourceJoint);
      }
   }

   private static void copyJointsAcceleration(List<? extends JointReadOnly> source, List<? extends JointBasics> destination)
   {
      for (int jointIndex = 0; jointIndex < source.size(); jointIndex++)
      {
         JointReadOnly sourceJoint = source.get(jointIndex);
         JointBasics destinationJoint = destination.get(jointIndex);
         destinationJoint.setJointAcceleration(sourceJoint);
      }
   }

   private static void copyJointsTau(List<? extends JointReadOnly> source, List<? extends JointBasics> destination)
   {
      for (int jointIndex = 0; jointIndex < source.size(); jointIndex++)
      {
         JointReadOnly sourceJoint = source.get(jointIndex);
         JointBasics destinationJoint = destination.get(jointIndex);
         destinationJoint.setJointWrench(sourceJoint);
      }
   }

   /**
    * Iterates through the given {@code joints}, extract the requested state {@code stateSelection} for
    * each joint, and finally stores the states in order in the given matrix {@code matrixToPack}.
    *
    * @param joints         the joints to extract the state of. Not modified.
    * @param stateSelection indicates what state is to be extract, i.e. it can be either configuration,
    *                       velocity, acceleration, or tau (or effort).
    * @param matrixToPack   the matrix in which the state of the joints is to be stored. Modified.
    * @return the number of rows used to store the information in the matrix.
    */
   public static int extractJointsState(List<? extends JointReadOnly> joints, JointStateType stateSelection, DMatrix matrixToPack)
   {
      switch (stateSelection)
      {
         case CONFIGURATION:
            return extractJointsConfiguration(joints, 0, matrixToPack);
         case VELOCITY:
            return extractJointsVelocity(joints, 0, matrixToPack);
         case ACCELERATION:
            return extractJointsAcceleration(joints, 0, matrixToPack);
         case EFFORT:
            return extractJointsTau(joints, 0, matrixToPack);
         default:
            throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static int extractJointsConfiguration(List<? extends JointReadOnly> joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointConfiguration(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsVelocity(List<? extends JointReadOnly> joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointVelocity(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsAcceleration(List<? extends JointReadOnly> joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointAcceleration(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsTau(List<? extends JointReadOnly> joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointTau(startIndex, matrixToPack);
      }

      return startIndex;
   }

   /**
    * Iterates through the given {@code joints}, extract the requested state {@code stateSelection} for
    * each joint, and finally stores the states in order in the given matrix {@code matrixToPack}.
    *
    * @param joints         the joints to extract the state of. Not modified.
    * @param stateSelection indicates what state is to be extract, i.e. it can be either configuration,
    *                       velocity, acceleration, or tau (or effort).
    * @param matrixToPack   the matrix in which the state of the joints is to be stored. Modified.
    * @return the number of rows used to store the information in the matrix.
    */
   public static int extractJointsState(JointReadOnly[] joints, JointStateType stateSelection, DMatrix matrixToPack)
   {
      switch (stateSelection)
      {
         case CONFIGURATION:
            return extractJointsConfiguration(joints, 0, matrixToPack);
         case VELOCITY:
            return extractJointsVelocity(joints, 0, matrixToPack);
         case ACCELERATION:
            return extractJointsAcceleration(joints, 0, matrixToPack);
         case EFFORT:
            return extractJointsTau(joints, 0, matrixToPack);
         default:
            throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static int extractJointsConfiguration(JointReadOnly[] joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointReadOnly joint = joints[jointIndex];
         startIndex = joint.getJointConfiguration(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsVelocity(JointReadOnly[] joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointReadOnly joint = joints[jointIndex];
         startIndex = joint.getJointVelocity(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsAcceleration(JointReadOnly[] joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointReadOnly joint = joints[jointIndex];
         startIndex = joint.getJointAcceleration(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsTau(JointReadOnly[] joints, int startIndex, DMatrix matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointReadOnly joint = joints[jointIndex];
         startIndex = joint.getJointTau(startIndex, matrixToPack);
      }

      return startIndex;
   }

   /**
    * Iterates through the given {@code joints}, and update their requested state
    * {@code stateSelection} using the given {@code matrix} assuming the state has been previously
    * stored in the proper order.
    *
    * @param joints         the joints to update the state of. Modified.
    * @param stateSelection indicates what state is to be updated, i.e. it can be either configuration,
    *                       velocity, acceleration, or tau (or effort).
    * @param matrix         the matrix in which the new state of the joints is stored. The data is
    *                       expected to be stored as a column vector starting at the first row.
    *                       Modified.
    * @return the number of rows that were used from the matrix.
    */
   public static int insertJointsState(List<? extends JointBasics> joints, JointStateType stateSelection, DMatrix matrix)
   {
      switch (stateSelection)
      {
         case CONFIGURATION:
            return insertJointsConfiguration(joints, 0, matrix);
         case VELOCITY:
            return insertJointsVelocity(joints, 0, matrix);
         case ACCELERATION:
            return insertJointsAcceleration(joints, 0, matrix);
         case EFFORT:
            return insertJointsTau(joints, 0, matrix);
         default:
            throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static int insertJointsConfiguration(List<? extends JointBasics> joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointBasics joint = joints.get(jointIndex);
         startIndex = joint.setJointConfiguration(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsVelocity(List<? extends JointBasics> joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointBasics joint = joints.get(jointIndex);
         startIndex = joint.setJointVelocity(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsAcceleration(List<? extends JointBasics> joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointBasics joint = joints.get(jointIndex);
         startIndex = joint.setJointAcceleration(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsTau(List<? extends JointBasics> joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointBasics joint = joints.get(jointIndex);
         startIndex = joint.setJointTau(startIndex, matrix);
      }

      return startIndex;
   }

   /**
    * Iterates through the given {@code joints}, and update their requested state
    * {@code stateSelection} using the given {@code matrix} assuming the state has been previously
    * stored in the proper order.
    *
    * @param joints         the joints to update the state of. Modified.
    * @param stateSelection indicates what state is to be updated, i.e. it can be either configuration,
    *                       velocity, acceleration, or tau (or effort).
    * @param matrix         the matrix in which the new state of the joints is stored. The data is
    *                       expected to be stored as a column vector starting at the first row.
    *                       Modified.
    * @return the number of rows that were used from the matrix.
    */
   public static int insertJointsState(JointBasics[] joints, JointStateType stateSelection, DMatrix matrix)
   {
      switch (stateSelection)
      {
         case CONFIGURATION:
            return insertJointsConfiguration(joints, 0, matrix);
         case VELOCITY:
            return insertJointsVelocity(joints, 0, matrix);
         case ACCELERATION:
            return insertJointsAcceleration(joints, 0, matrix);
         case EFFORT:
            return insertJointsTau(joints, 0, matrix);
         default:
            throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static int insertJointsConfiguration(JointBasics[] joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];
         startIndex = joint.setJointConfiguration(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsVelocity(JointBasics[] joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];
         startIndex = joint.setJointVelocity(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsAcceleration(JointBasics[] joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];
         startIndex = joint.setJointAcceleration(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsTau(JointBasics[] joints, int startIndex, DMatrix matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];
         startIndex = joint.setJointTau(startIndex, matrix);
      }

      return startIndex;
   }

   /**
    * Returns a list of the children joints of the given {@code rigidBody} sorted such that the joints that are part of a loop closure are treated last.
    *
    * @param rigidBody the query. Not modified.
    * @return the list of children joints sorted such that the joints that are part of a loop closure are treated last.
    */
   public static List<JointReadOnly> sortLoopClosureInChildrenJoints(RigidBodyReadOnly rigidBody)
   {
      List<JointReadOnly> childrenJoints = new ArrayList<>(rigidBody.getChildrenJoints());

      if (childrenJoints.size() > 1)
      { // Reorganize the joints in the children to ensure that loop closures are treated last.
         List<JointReadOnly> loopClosureAncestors = new ArrayList<>();

         for (int i = 0; i < childrenJoints.size(); )
         {
            if (doesSubtreeContainLoopClosure(childrenJoints.get(i).getSuccessor()))
               loopClosureAncestors.add(childrenJoints.remove(i));
            else
               i++;
         }

         childrenJoints.addAll(loopClosureAncestors);
      }
      return childrenJoints;
   }
}