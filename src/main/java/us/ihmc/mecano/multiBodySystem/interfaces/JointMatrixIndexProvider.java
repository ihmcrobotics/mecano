package us.ihmc.mecano.multiBodySystem.interfaces;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Provides an interface to reliably store and extract joint information in and from a matrix that
 * is used for the whole system.
 * 
 * @author Sylvain Bertrand
 */
public interface JointMatrixIndexProvider
{
   /**
    * Gets a list of the joints in order their information should be stored in or extracted from a
    * matrix.
    * <p>
    * When this {@code JointMatrixIndexProvider} is obtained from a {@link MultiBodySystemReadOnly},
    * the returned list only contains the joints considered and is expected to preserve the ordering.
    * </p>
    * 
    * @return the ordered joint list.
    */
   List<? extends JointReadOnly> getIndexedJointsInOrder();

   /**
    * Gets the corresponding matrix indices for each degree of freedom of the given joint.
    * 
    * @param joint the joint to get the indices for. Not modified.
    * @return an array of length equal to {@code joint.getDegreesOfFreedom()}, containing the matrix
    *         index for each degree of freedom.
    */
   int[] getJointDoFIndices(JointReadOnly joint);

   /**
    * Gets the corresponding matrix indices for each component needed to describe the given joint's
    * configuration.
    * <p>
    * For some joints, such as {@link SixDoFJointReadOnly} and {@link SphericalJointReadOnly}, the
    * configuration is 1-element larger than the number of degrees of freedom. This is due to using
    * quaternion to represent the joint orientation.
    * </p>
    * 
    * @param joint the joint to get the indices for. Not modified.
    * @return an array of length equal to {@code joint.getConfigurationMatrixSize()}, containing the
    *         matrix index for each component of the joint's configuration.
    */
   int[] getJointConfigurationIndices(JointReadOnly joint);

   /**
    * Creates a new {@link JointMatrixIndexProvider} indexing all the given joints.
    * 
    * @param jointsToIndex the array of joints to be indexed. Not modified.
    * @return the index provider for the given joints.
    */
   public static JointMatrixIndexProvider toIndexProvider(JointReadOnly[] jointsToIndex)
   {
      return toIndexProvider(Arrays.asList(jointsToIndex));
   }

   /**
    * Creates a new {@link JointMatrixIndexProvider} indexing all the given joints.
    * 
    * @param jointsToIndex the collection of joints to be indexed. Not modified.
    * @return the index provider for the given joints.
    */
   public static JointMatrixIndexProvider toIndexProvider(Collection<? extends JointReadOnly> jointsToIndex)
   {
      List<? extends JointReadOnly> indexedJointsInOrder = new ArrayList<>(jointsToIndex);
      Map<JointReadOnly, int[]> jointToDoFIndexMap = new HashMap<>();
      Map<JointReadOnly, int[]> jointToConfigurationIndexMap = new HashMap<>();

      int currentDoFIndex = 0;
      int currentConfigurationIndex = 0;

      for (JointReadOnly joint : indexedJointsInOrder)
      {
         int[] dofIndices = new int[joint.getDegreesOfFreedom()];

         for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
         {
            dofIndices[i] = currentDoFIndex;
            currentDoFIndex++;
         }

         jointToDoFIndexMap.put(joint, dofIndices);

         int[] configurationIndices = new int[joint.getConfigurationMatrixSize()];

         for (int i = 0; i < joint.getConfigurationMatrixSize(); i++)
         {
            configurationIndices[i] = currentConfigurationIndex;
            currentConfigurationIndex++;
         }

         jointToConfigurationIndexMap.put(joint, configurationIndices);
      }

      return new JointMatrixIndexProvider()
      {
         @Override
         public List<? extends JointReadOnly> getIndexedJointsInOrder()
         {
            return indexedJointsInOrder;
         }

         @Override
         public int[] getJointDoFIndices(JointReadOnly joint)
         {
            return jointToDoFIndexMap.get(joint);
         }

         @Override
         public int[] getJointConfigurationIndices(JointReadOnly joint)
         {
            return jointToConfigurationIndexMap.get(joint);
         }
      };
   }
}
