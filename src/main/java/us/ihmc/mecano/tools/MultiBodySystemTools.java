package us.ihmc.mecano.tools;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

/**
 * This class provides a variety of tools to facilitate operations that need to navigate through a
 * multi-body system.
 * 
 */
public class MultiBodySystemTools
{
   /**
    * Retrieves and gets the root body of the multi-body system the given {@code body} belongs to.
    *
    * @param body an arbitrary body that belongs to the multi-body system that this method is to
    *           find the root.
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
    * @param body an arbitrary body that belongs to the multi-body system that this method is to
    *           find the root.
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
    * @param end the rigid-body where to stop the collection of joints.
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
    * @param end the rigid-body where to stop the collection of joints.
    * @return the array of joints representing the path from {@code start} to {@code end}, or
    *         {@code null} if the given rigid-bodies are not part of the same multi-body system.
    */
   public static JointReadOnly[] createJointPath(RigidBodyReadOnly start, RigidBodyReadOnly end)
   {
      boolean flip = false;
      RigidBodyReadOnly descendant = start;
      RigidBodyReadOnly ancestor = end;
      int pathLength = computeDistanceToAncestor(descendant, ancestor);

      if (pathLength < 0)
      {
         flip = true;
         descendant = end;
         ancestor = start;
         pathLength = computeDistanceToAncestor(end, start);

         if (pathLength < 0)
            return null; // The rigid-bodies are not part of the same system.
      }

      JointReadOnly[] jointPath = new JointReadOnly[pathLength];
      RigidBodyReadOnly currentBody = descendant;
      int i = 0;

      while (currentBody != ancestor)
      {
         int j = flip ? pathLength - 1 - i : i;
         JointReadOnly parentJoint = currentBody.getParentJoint();
         jointPath[j] = parentJoint;
         currentBody = parentJoint.getPredecessor();
         i++;
      }

      return jointPath;
   }

   /**
    * Traverses up the kinematic chain from the candidate descendant towards the root body, checking
    * to see if each parent body is the ancestor in question.
    * 
    * @param candidateDescendant the query for the descendant. A rigid-body is the descendant of
    *           another rigid-body if it is between the other rigid-body and an end-effector.
    * @param ancestor the query for the ancestor. A rigid-body is the ancestor of another rigid-body
    *           if it is between the other rigid-body and the root body.
    * @return {@code true} if {@code candidateDescendant} is a descendant of {@code ancestor},
    *         {@code false} otherwise.
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
    * Computes the number of joints that separates the {@code descendant} from its {@code ancestor}.
    * <p>
    * The {@code ancestor} is expected to be located between the root body and the
    * {@code descendant}, if not this method returns {@code -1}.
    * </p>
    *
    * @param descendant the descendant, often it is the end-effector.
    * @param ancestor the ancestor of the descendant, often it is the root body.
    * @return the distance in number of joints between the {@code descendant} and the
    *         {@code ancestor}. This method returns {@code 0} if the two rigid-bodies are the same.
    *         This method returns {@code -1} if the given {@code ancestor} is not located between
    *         the {@code descendant} and the root body.
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
    * Calculates the number of degrees of freedom of the kinematic chain that starts from
    * {@code ancestor} to end to {@code descendant}.
    * 
    * @param ancestor the base of the kinematic chain.
    * @param descendant the end-effector of the kinematic chain.
    * @return the number of degrees of freedom.
    * @throws RuntimeException if the given ancestor and descendant are swapped, or if the do not
    *            belong to the same system.
    * @throws RuntimeException this method does not support in kinematic trees to go through
    *            different branches.
    */
   public static int computeDegreesOfFreedom(RigidBodyReadOnly ancestor, RigidBodyReadOnly descendant)
   {
      int nDoFs = 0;

      RigidBodyReadOnly currentBody = descendant;

      while (currentBody != ancestor)
      {
         JointReadOnly parentJoint = currentBody.getParentJoint();

         if (parentJoint == null)
            throw new RuntimeException("Could not find the ancestor: " + ancestor.getName() + ", to the descendant: " + descendant.getName());

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
    * Collects in order the successor of each joint, i.e. {@link JointReadOnly#getSuccessor()}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param joints the joints to collect the successors of.
    * @return the array containing in order the successor of each joint.
    */
   public static RigidBodyBasics[] collectSuccessors(JointBasics... joints)
   {
      return Stream.of(joints).map(JointBasics::getSuccessor).toArray(RigidBodyBasics[]::new);
   }

   /**
    * Collects for each rigid-body all their support joints, i.e. the joints that are between the
    * rigid-body and the root body, and returns an array containing no duplicate elements.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param rigidBodies the rigid-bodies to collect the support joints of.
    * @return the array containing the support joints of all the given rigid-bodies.
    */
   public static JointReadOnly[] collectSupportJoints(RigidBodyReadOnly... rigidBodies)
   {
      Set<JointReadOnly> supportSet = new LinkedHashSet<>();
      for (RigidBodyReadOnly rigidBody : rigidBodies)
      {
         RigidBodyReadOnly rootBody = getRootBody(rigidBody);
         JointReadOnly[] jointPath = createJointPath(rootBody, rigidBody);
         supportSet.addAll(Arrays.asList(jointPath));
      }

      return supportSet.toArray(new JointReadOnly[supportSet.size()]);
   }

   /**
    * Collects only the joints from {@code source} that are instances of the given {@code clazz} and
    * stores them in {@code destination}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param source the original collection of joints to filter. Not modified.
    * @param clazz the class that the filtered joints have to implement.
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
    * The filtered joints are added to the end of {@code destination} using
    * {@link List#add(Object)}.
    * </p>
    *
    * @param source the original collection of joints to filter. Not modified.
    * @param destination the collection where to store the filtered joints. Modified.
    * @param clazz the class that the filtered joints have to implement.
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
    * @param source the original array of joints to filter. Not modified.
    * @param clazz the class that the filtered joints have to implement.
    * @return the array containing the filtered joints.
    */
   public static <T extends JointReadOnly> T[] filterJoints(JointReadOnly[] source, Class<T> clazz)
   {
      @SuppressWarnings("unchecked")
      T[] retArray = (T[]) Array.newInstance(clazz, computeNumberOfJointsOfType(clazz, source));
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
    * @param source the original array of joints to filter. Not modified.
    * @param destination the array where to store the filtered joints. Modified.
    * @param clazz the class that the filtered joints have to implement.
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
    * Tests that the given {@code joints} are stored in order from root to leaf and represent a
    * continuous kinematic chain.
    * <p>
    * More precisely:
    * <ul>
    * <li>the first joint of the array should be the closest or equal to the root body of the
    * multi-body system they belong.
    * <li>the last joint of the array should be the closest or equal to one of the leaves or
    * end-effectors of the multi-body system they belong.
    * <li>the continuity test asserts that each pair of successive joints in the given array are
    * also successive in the kinematic chain:
    * {@code joints[i] == joints[i+1].getPredecessor().getParentJoint()} &forall; i &in; [0;
    * {@code joints.length - 1}].
    * </ul>
    * </p>
    * 
    * @param joints the query. Not modified.
    * @return {@code true} if the joints are stored in a continuous manner from root to leaf,
    *         {@code false} otherwise.
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
    * <li>the first joint of the list should be the closest or equal to the root body of the
    * multi-body system they belong.
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
    *         {@code false} otherwise.
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
    * Iterates through the given array and compute how many do implement the given {@code clazz}.
    * 
    * @param clazz the query for the joint type.
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
    * @param source the joints holding the state to copy over. Not modified.
    * @param destination the joints which state is to be be updated. Modified.
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
      case TAU:
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
    * Iterates through the given {@code joints}, extract the requested state {@code stateSelection}
    * for each joint, and finally stores the states in order in the given matrix
    * {@code matrixToPack}.
    * 
    * @param joints the joints to extract the state of. Not modified.
    * @param stateSelection indicates what state is to be extract, i.e. it can be either
    *           configuration, velocity, acceleration, or tau (or effort).
    * @param matrixToPack the matrix in which the state of the joints is to be stored. Modified.
    * @return the number of rows used to store the information in the matrix.
    */
   public static int extractJointsState(List<? extends JointReadOnly> joints, JointStateType stateSelection, DenseMatrix64F matrixToPack)
   {
      switch (stateSelection)
      {
      case CONFIGURATION:
         return extractJointsConfiguration(joints, 0, matrixToPack);
      case VELOCITY:
         return extractJointsVelocity(joints, 0, matrixToPack);
      case ACCELERATION:
         return extractJointsAcceleration(joints, 0, matrixToPack);
      case TAU:
         return extractJointsTau(joints, 0, matrixToPack);
      default:
         throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static int extractJointsConfiguration(List<? extends JointReadOnly> joints, int startIndex, DenseMatrix64F matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointConfiguration(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsVelocity(List<? extends JointReadOnly> joints, int startIndex, DenseMatrix64F matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointVelocity(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsAcceleration(List<? extends JointReadOnly> joints, int startIndex, DenseMatrix64F matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointAcceleration(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsTau(List<? extends JointReadOnly> joints, int startIndex, DenseMatrix64F matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointReadOnly joint = joints.get(jointIndex);
         startIndex = joint.getJointTau(startIndex, matrixToPack);
      }

      return startIndex;
   }

   /**
    * Iterates through the given {@code joints}, extract the requested state {@code stateSelection}
    * for each joint, and finally stores the states in order in the given matrix
    * {@code matrixToPack}.
    * 
    * @param joints the joints to extract the state of. Not modified.
    * @param stateSelection indicates what state is to be extract, i.e. it can be either
    *           configuration, velocity, acceleration, or tau (or effort).
    * @param matrixToPack the matrix in which the state of the joints is to be stored. Modified.
    * @return the number of rows used to store the information in the matrix.
    */
   public static int extractJointsState(JointReadOnly[] joints, JointStateType stateSelection, DenseMatrix64F matrixToPack)
   {
      switch (stateSelection)
      {
      case CONFIGURATION:
         return extractJointsConfiguration(joints, 0, matrixToPack);
      case VELOCITY:
         return extractJointsVelocity(joints, 0, matrixToPack);
      case ACCELERATION:
         return extractJointsAcceleration(joints, 0, matrixToPack);
      case TAU:
         return extractJointsTau(joints, 0, matrixToPack);
      default:
         throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static int extractJointsConfiguration(JointReadOnly[] joints, int startIndex, DenseMatrix64F matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointReadOnly joint = joints[jointIndex];
         startIndex = joint.getJointConfiguration(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsVelocity(JointReadOnly[] joints, int startIndex, DenseMatrix64F matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointReadOnly joint = joints[jointIndex];
         startIndex = joint.getJointVelocity(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsAcceleration(JointReadOnly[] joints, int startIndex, DenseMatrix64F matrixToPack)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointReadOnly joint = joints[jointIndex];
         startIndex = joint.getJointAcceleration(startIndex, matrixToPack);
      }

      return startIndex;
   }

   private static int extractJointsTau(JointReadOnly[] joints, int startIndex, DenseMatrix64F matrixToPack)
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
    * @param joints the joints to update the state of. Modified.
    * @param stateSelection indicates what state is to be updated, i.e. it can be either
    *           configuration, velocity, acceleration, or tau (or effort).
    * @param matrix the matrix in which the new state of the joints is stored. The data is expected
    *           to be stored as a column vector starting at the first row. Modified.
    * @return the number of rows that were used from the matrix.
    */
   public static int insertJointsState(List<? extends JointBasics> joints, JointStateType stateSelection, DenseMatrix64F matrix)
   {
      switch (stateSelection)
      {
      case CONFIGURATION:
         return insertJointsConfiguration(joints, 0, matrix);
      case VELOCITY:
         return insertJointsVelocity(joints, 0, matrix);
      case ACCELERATION:
         return insertJointsAcceleration(joints, 0, matrix);
      case TAU:
         return insertJointsTau(joints, 0, matrix);
      default:
         throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static int insertJointsConfiguration(List<? extends JointBasics> joints, int startIndex, DenseMatrix64F matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointBasics joint = joints.get(jointIndex);
         startIndex = joint.setJointConfiguration(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsVelocity(List<? extends JointBasics> joints, int startIndex, DenseMatrix64F matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointBasics joint = joints.get(jointIndex);
         startIndex = joint.setJointVelocity(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsAcceleration(List<? extends JointBasics> joints, int startIndex, DenseMatrix64F matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
      {
         JointBasics joint = joints.get(jointIndex);
         startIndex = joint.setJointAcceleration(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsTau(List<? extends JointBasics> joints, int startIndex, DenseMatrix64F matrix)
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
    * @param joints the joints to update the state of. Modified.
    * @param stateSelection indicates what state is to be updated, i.e. it can be either
    *           configuration, velocity, acceleration, or tau (or effort).
    * @param matrix the matrix in which the new state of the joints is stored. The data is expected
    *           to be stored as a column vector starting at the first row. Modified.
    * @return the number of rows that were used from the matrix.
    */
   public static int insertJointsState(JointBasics[] joints, JointStateType stateSelection, DenseMatrix64F matrix)
   {
      switch (stateSelection)
      {
      case CONFIGURATION:
         return insertJointsConfiguration(joints, 0, matrix);
      case VELOCITY:
         return insertJointsVelocity(joints, 0, matrix);
      case ACCELERATION:
         return insertJointsAcceleration(joints, 0, matrix);
      case TAU:
         return insertJointsTau(joints, 0, matrix);
      default:
         throw new RuntimeException("Unexpected value for stateSelection: " + stateSelection);
      }
   }

   private static int insertJointsConfiguration(JointBasics[] joints, int startIndex, DenseMatrix64F matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];
         startIndex = joint.setJointConfiguration(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsVelocity(JointBasics[] joints, int startIndex, DenseMatrix64F matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];
         startIndex = joint.setJointVelocity(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsAcceleration(JointBasics[] joints, int startIndex, DenseMatrix64F matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];
         startIndex = joint.setJointAcceleration(startIndex, matrix);
      }

      return startIndex;
   }

   private static int insertJointsTau(JointBasics[] joints, int startIndex, DenseMatrix64F matrix)
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         JointBasics joint = joints[jointIndex];
         startIndex = joint.setJointTau(startIndex, matrix);
      }

      return startIndex;
   }
}