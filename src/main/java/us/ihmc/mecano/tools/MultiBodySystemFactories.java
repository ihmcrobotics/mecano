package us.ihmc.mecano.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.FixedJoint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PlanarJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.SphericalJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FixedJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FixedJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PrismaticJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.PrismaticJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

/**
 * This class provides factories to clone multi-body systems.
 *
 * @author Sylvain Bertrand
 */
public class MultiBodySystemFactories
{
   /**
    * The joint builder used by default.
    * <p>
    * This builder creates {@code SixDoFJoint}, {@code PlanarJoint}, {@code SphericalJoint},
    * {@code RevoluteJoint}, and {@code PrismaticJoint}.
    * </p>
    */
   public static final JointBuilder DEFAULT_JOINT_BUILDER = new JointBuilder()
   {
   };

   /**
    * The rigid-body builder used by default.
    * <p>
    * This builder creates {@code RigidBody}.
    * </p>
    */
   public static final RigidBodyBuilder DEFAULT_RIGID_BODY_BUILDER = new RigidBodyBuilder()
   {
   };

   /**
    * Performs a deep copy of the kinematic chain described by the given {@code start} and {@code end},
    * then filters the cloned joint to return only the ones implementing {@code OneDoFJoint}.
    * <p>
    * The clone of the kinematic chain has its own root body which reference frame is child of the
    * frame after the parent joint of {@code start}. As a result, the clone is an independent
    * multi-body system but its root is following the original multi-body system.
    * </p>
    * <p>
    * Note on kinematic loops: any loop on the path from {@code start} to {@code end} will not be
    * properly cloned. More precisely, this method skip the secondary branch of a kinematic loop, i.e.
    * the branch that starts off the primary branch and ends with the loop closure joint.
    * </p>
    *
    * @param start the rigid-body from where the kinematic chain starts.
    * @param end   the rigid-body where the kinematic chain ends.
    * @return the clone kinematic chain.
    * @throws UnsupportedOperationException if the kinematic chain contains one or more loop closure
    *                                       joints.
    */
   public static OneDoFJointBasics[] cloneOneDoFJointKinematicChain(RigidBodyBasics start, RigidBodyBasics end)
   {
      return cloneKinematicChainAndFilter(MultiBodySystemTools.createOneDoFJointPath(start, end), OneDoFJointBasics.class);
   }

   /**
    * Performs a deep copy of the given {@code originalJoints} then filters the cloned joint to return
    * only the ones implementing {@code OneDoFJoint}.
    * <p>
    * The {@code originalJoints} must represent a continuous kinematic chain. The joints must be stored
    * in order starting from the joint that is the closest to the root, to end with the joint the
    * closest to an end-effector.
    * </p>
    * <p>
    * The clone of the kinematic chain has its own root body which reference frame is child of the
    * frame after the parent joint of {@code originalJoints[0]}. As a result, the clone is an
    * independent multi-body system but its root is following the original multi-body system.
    * </p>
    *
    * @param originalJoints the kinematic chain to clone. Not modified.
    * @return the clone kinematic chain.
    * @throws UnsupportedOperationException if {@code originalJoints} contains one or more loop closure
    *                                       joints.
    */
   public static OneDoFJointBasics[] cloneOneDoFJointKinematicChain(OneDoFJointBasics[] originalJoints)
   {
      return cloneKinematicChainAndFilter(originalJoints, OneDoFJointBasics.class);
   }

   /**
    * Performs a deep copy of the given {@code originalJoints} then filters the cloned joint to return
    * only the ones that implement the given {@code clazz}.
    * <p>
    * The {@code originalJoints} must represent a continuous kinematic chain. The joints must be stored
    * in order starting from the joint that is the closest to the root, to end with the joint the
    * closest to an end-effector.
    * </p>
    * <p>
    * The clone of the kinematic chain has its own root body which reference frame is child of the
    * frame after the parent joint of {@code originalJoints[0]}. As a result, the clone is an
    * independent multi-body system but its root is following the original multi-body system.
    * </p>
    *
    * @param <T> the joint type to be cloned.
    * @param originalJoints the kinematic chain to clone. Not modified.
    * @param clazz          class used to filter the cloned joints that are to be returned.
    * @return the clone kinematic chain.
    * @throws UnsupportedOperationException if {@code originalJoints} contains one or more loop closure
    *                                       joints.
    */
   public static <T extends JointReadOnly> T[] cloneKinematicChainAndFilter(T[] originalJoints, Class<T> clazz)
   {
      return MultiBodySystemTools.filterJoints(cloneKinematicChain(originalJoints), clazz);
   }

   /**
    * Performs a deep copy of the given {@code originalJoints} then filters the cloned joint to return
    * only the ones that implement the given {@code clazz}.
    * <p>
    * The {@code originalJoints} must represent a continuous kinematic chain. The joints must be stored
    * in order starting from the joint that is the closest to the root, to end with the joint the
    * closest to an end-effector.
    * </p>
    * <p>
    * The clone of the kinematic chain has its own root body which reference frame is child of the
    * frame after the parent joint of {@code originalJoints[0]}. As a result, the clone is an
    * independent multi-body system but its root is following the original multi-body system.
    * </p>
    *
    * @param <T> the joint type to be cloned.
    * @param originalJoints the kinematic chain to clone. Not modified.
    * @param clazz          class used to filter the cloned joints that are to be returned.
    * @param cloneSuffix    suffix to append to the cloned joints and rigid-bodies.
    * @return the clone kinematic chain.
    * @throws UnsupportedOperationException if {@code originalJoints} contains one or more loop closure
    *                                       joints.
    */
   public static <T extends JointReadOnly> T[] cloneKinematicChainAndFilter(T[] originalJoints, Class<T> clazz, String cloneSuffix)
   {
      return MultiBodySystemTools.filterJoints(cloneKinematicChain(originalJoints, cloneSuffix), clazz);
   }

   /**
    * Performs a deep copy of the given {@code originalJoints}.
    * <p>
    * The {@code originalJoints} must represent a continuous kinematic chain. The joints must be stored
    * in order starting from the joint that is the closest to the root, to end with the joint the
    * closest to an end-effector.
    * </p>
    * <p>
    * The clone of the kinematic chain has its own root body which reference frame is child of the
    * frame after the parent joint of {@code originalJoints[0]}. As a result, the clone is an
    * independent multi-body system but its root is following the original multi-body system.
    * </p>
    *
    * @param originalJoints the kinematic chain to clone. Not modified.
    * @return the clone kinematic chain.
    * @throws UnsupportedOperationException if {@code originalJoints} contains one or more loop closure
    *                                       joints.
    */
   public static JointBasics[] cloneKinematicChain(JointReadOnly[] originalJoints)
   {
      return cloneKinematicChain(originalJoints, "Copy");
   }

   /**
    * Performs a deep copy of the given {@code originalJoints}.
    * <p>
    * The {@code originalJoints} must represent a continuous kinematic chain. The joints must be stored
    * in order starting from the joint that is the closest to the root, to end with the joint the
    * closest to an end-effector.
    * </p>
    * <p>
    * The clone of the kinematic chain has its own root body which reference frame is child of the
    * frame after the parent joint of {@code originalJoints[0]}. As a result, the clone is an
    * independent multi-body system but its root is following the original multi-body system.
    * </p>
    *
    * @param originalJoints the kinematic chain to clone. Not modified.
    * @param cloneSuffix    suffix to append to the cloned joints and rigid-bodies.
    * @return the clone kinematic chain.
    * @throws UnsupportedOperationException if {@code originalJoints} contains one or more loop closure
    *                                       joints.
    */
   public static JointBasics[] cloneKinematicChain(JointReadOnly[] originalJoints, String cloneSuffix)
   {
      return cloneKinematicChain(originalJoints, cloneSuffix, null);
   }

   /**
    * Performs a deep copy of the given {@code originalJoints}.
    * <p>
    * The {@code originalJoints} must represent a continuous kinematic chain. The joints must be stored
    * in order starting from the joint that is the closest to the root, to end with the joint the
    * closest to an end-effector.
    * </p>
    * <p>
    * The clone of the kinematic chain has its own root body which reference frame is child of the
    * frame after the parent joint of {@code originalJoints[0]} or {@code chainRootFrame} if provided.
    * When {@code chainRootFrame} is not provided, i.e. equal to {@code null}, the clone is an
    * independent multi-body system but its root is following the original multi-body system.
    * </p>
    *
    * @param originalJoints the kinematic chain to clone. Not modified.
    * @param cloneSuffix    suffix to append to the cloned joints and rigid-bodies.
    * @param chainRootFrame the parent frame of the rigid-body of the clone kinematic chain. When
    *                       {@code null}, the frame is equal to frame after the parent joint of
    *                       {@code originalJoints[0]}.
    * @return the clone kinematic chain.
    * @throws UnsupportedOperationException if {@code originalJoints} contains one or more loop closure
    *                                       joints.
    */
   public static JointBasics[] cloneKinematicChain(JointReadOnly[] originalJoints, String cloneSuffix, ReferenceFrame chainRootFrame)
   {
      if (!MultiBodySystemTools.areJointsInContinuousOrder(originalJoints))
         throw new IllegalArgumentException("The given joints do not represent a continuous kinematic chain or are out of order: "
               + Arrays.toString(originalJoints));

      JointBasics[] cloneJoints = new JointBasics[originalJoints.length];
      Map<RigidBodyReadOnly, RigidBodyBasics> originalToCloneBodyMap = new HashMap<>();
      RigidBodyReadOnly originalAncestor = originalJoints[0].getPredecessor();
      RigidBodyBasics cloneAncestor;
      if (originalAncestor.isRootBody())
         cloneAncestor = cloneRigidBody(originalAncestor, chainRootFrame, cloneSuffix, null, null);
      else if (chainRootFrame != null)
         cloneAncestor = new RigidBody(originalAncestor.getName() + cloneSuffix, chainRootFrame);
      else
         cloneAncestor = new RigidBody(originalAncestor.getName() + cloneSuffix, originalAncestor.getParentJoint().getFrameAfterJoint());

      originalToCloneBodyMap.put(originalAncestor, cloneAncestor);

      for (int jointIndex = 0; jointIndex < originalJoints.length; jointIndex++)
      {
         JointReadOnly originalJoint = originalJoints[jointIndex];

         if (originalJoint.isLoopClosure())
            throw new UnsupportedOperationException("Cloning loop closure joints is not supported.");

         RigidBodyReadOnly originalPredecessor = originalJoint.getPredecessor();
         // Retrieve the right predecessor for the joint to clone. The map has to contain the clone predecessor.
         RigidBodyBasics clonePredecessor = originalToCloneBodyMap.get(originalPredecessor);

         // Clone the joint
         JointBasics cloneJoint = cloneJoint(originalJoint, cloneSuffix, clonePredecessor, false, null);

         // Clone the successor
         RigidBodyReadOnly originalSuccessor = originalJoint.getSuccessor();
         RigidBodyBasics cloneSuccessor = cloneRigidBody(originalSuccessor, null, cloneSuffix, cloneJoint, null);
         originalToCloneBodyMap.put(originalSuccessor, cloneSuccessor);

         cloneJoints[jointIndex] = cloneJoint;
      }
      return cloneJoints;
   }

   /**
    * Performs a deep copy of an entire multi-body system.
    * <p>
    * The clone of the multi-body system has its own root body which reference frame shares the same
    * parent as {@code originalRootBody.getBodyFixedFrame()}.
    * </p>
    *
    * @param originalRootBody     the root of the multi-body system to clone. Not modified.
    * @param cloneStationaryFrame the reference frame to which the cloned system is attached to. The
    *                             given frame is expected to be stationary.
    * @param cloneSuffix          suffix to append to the cloned joints and rigid-bodies.
    * @return the clone multi-body system.
    * @throws IllegalArgumentException if the given {@code originalRootBody} is not the root body of
    *                                  its system.
    */
   public static RigidBodyBasics cloneMultiBodySystem(RigidBodyReadOnly originalRootBody, ReferenceFrame cloneStationaryFrame, String cloneSuffix)
   {
      return cloneMultiBodySystem(originalRootBody, cloneStationaryFrame, cloneSuffix, null, null);
   }

   /**
    * Performs a deep copy of an entire multi-body system.
    * <p>
    * The clone of the multi-body system has its own root body which reference frame shares the same
    * parent as {@code originalRootBody.getBodyFixedFrame()}.
    * </p>
    *
    * @param originalRootBody     the root of the multi-body system to clone. Not modified.
    * @param cloneStationaryFrame the reference frame to which the cloned system is attached to. The
    *                             given frame is expected to be stationary.
    * @param cloneSuffix          suffix to append to the cloned joints and rigid-bodies.
    * @param rigidBodyBuilder     the builder to use for creating rigid-bodies. If {@code null},
    *                             {@link #DEFAULT_RIGID_BODY_BUILDER} is used.
    * @param jointBuilder         the builder to use for creating joints. If {@code null},
    *                             {@link #DEFAULT_JOINT_BUILDER} is used.
    * @return the clone multi-body system.
    * @throws IllegalArgumentException if the given {@code originalRootBody} is not the root body of
    *                                  its system.
    */
   public static RigidBodyBasics cloneMultiBodySystem(RigidBodyReadOnly originalRootBody, ReferenceFrame cloneStationaryFrame, String cloneSuffix,
                                                      RigidBodyBuilder rigidBodyBuilder, JointBuilder jointBuilder)
   {
      if (!originalRootBody.isRootBody())
         throw new IllegalArgumentException("The given rigid-body is not the root-body of its multi-body system: " + originalRootBody.getName());

      RigidBodyBasics cloneSubtreeStartBody = cloneRigidBody(originalRootBody, cloneStationaryFrame, cloneSuffix, null, rigidBodyBuilder);
      cloneSubtree(originalRootBody, cloneSubtreeStartBody, cloneSuffix, rigidBodyBuilder, jointBuilder);
      return cloneSubtreeStartBody;
   }

   /**
    * Performs a deep copy of the subtree that starts off {@code originalSubtreeStart}.
    * <p>
    * The clone of the subtree has its own root body which reference frame is child of the frame after
    * the parent joint of {@code originalSubtreeStartBody}. As a result, the clone is an independent
    * multi-body system but its root is following the original multi-body system.
    * </p>
    * <p>
    * WARNING for kinematic loops: this method does not cover the case where
    * {@code originalSubtreeStartBody} is part of the secondary branch of a kinematic loop, i.e. the
    * branch that originates from the primary branch and ends with a loop closure joint.
    * </p>
    *
    * @param originalSubtreeStartBody the rigid-body holding the subtree to be cloned. Not modified.
    * @param cloneSuffix              suffix to append to the cloned joints and rigid-bodies.
    * @return the clone subtree.
    * @throws IllegalArgumentException if {@code originalSubtreeStartBody} is a root body, in which
    *                                  case
    *                                  {@link #cloneMultiBodySystem(RigidBodyReadOnly, ReferenceFrame, String, RigidBodyBuilder, JointBuilder)}
    *                                  should be used.
    */
   public static RigidBodyBasics cloneSubtree(RigidBodyReadOnly originalSubtreeStartBody, String cloneSuffix)
   {
      return cloneSubtree(originalSubtreeStartBody, cloneSuffix, null, null);
   }

   /**
    * Performs a deep copy of the subtree that starts off {@code originalSubtreeStart}.
    * <p>
    * The clone of the subtree has its own root body which reference frame is child of the frame after
    * the parent joint of {@code originalSubtreeStartBody}. As a result, the clone is an independent
    * multi-body system but its root is following the original multi-body system.
    * </p>
    * <p>
    * WARNING for kinematic loops: this method does not cover the case where
    * {@code originalSubtreeStartBody} is part of the secondary branch of a kinematic loop, i.e. the
    * branch that originates from the primary branch and ends with a loop closure joint.
    * </p>
    *
    * @param originalSubtreeStartBody the rigid-body holding the subtree to be cloned. Not modified.
    * @param cloneSuffix              suffix to append to the cloned joints and rigid-bodies.
    * @param rigidBodyBuilder         the builder to use for creating rigid-bodies. If {@code null},
    *                                 {@link #DEFAULT_RIGID_BODY_BUILDER} is used.
    * @param jointBuilder             the builder to use for creating joints. If {@code null},
    *                                 {@link #DEFAULT_JOINT_BUILDER} is used.
    * @return the clone subtree.
    * @throws IllegalArgumentException if {@code originalSubtreeStartBody} is a root body, in which
    *                                  case
    *                                  {@link #cloneMultiBodySystem(RigidBodyReadOnly, ReferenceFrame, String, RigidBodyBuilder, JointBuilder)}
    *                                  should be used.
    */
   public static RigidBodyBasics cloneSubtree(RigidBodyReadOnly originalSubtreeStartBody, String cloneSuffix, RigidBodyBuilder rigidBodyBuilder,
                                              JointBuilder jointBuilder)
   {
      if (originalSubtreeStartBody.isRootBody())
         throw new IllegalArgumentException("originalSubtreeStartBody is a root body of its multi-body system, use MultiBodyFactories.cloneMultiBodySystem(...) instead");

      if (rigidBodyBuilder == null)
         rigidBodyBuilder = DEFAULT_RIGID_BODY_BUILDER;

      RigidBodyBasics cloneSubtreeStartBody = rigidBodyBuilder.buildRoot(originalSubtreeStartBody.getName() + cloneSuffix,
                                                                         new RigidBodyTransform(),
                                                                         originalSubtreeStartBody.getParentJoint().getFrameAfterJoint());

      cloneSubtree(originalSubtreeStartBody, cloneSubtreeStartBody, cloneSuffix, rigidBodyBuilder, jointBuilder);
      return cloneSubtreeStartBody;
   }

   private static void cloneSubtree(RigidBodyReadOnly originalStart, RigidBodyBasics cloneStart, String cloneSuffix, RigidBodyBuilder rigidBodyBuilder,
                                    JointBuilder jointBuilder)
   {
      Map<RigidBodyReadOnly, RigidBodyBasics> originalToCloneBodyMap = new HashMap<>();
      originalToCloneBodyMap.put(originalStart, cloneStart);

      List<JointBasics> loopClosureCloneJoints = new ArrayList<>();
      List<JointReadOnly> loopClosureOriginalJoints = new ArrayList<>();

      for (JointReadOnly originalJoint : originalStart.childrenSubtreeIterable())
      {
         RigidBodyReadOnly originalPredecessor = originalJoint.getPredecessor();
         // Retrieve the right predecessor for the joint to clone. The map has to contain the clone predecessor.
         RigidBodyBasics clonePredecessor = originalToCloneBodyMap.get(originalPredecessor);

         // Clone the joint
         JointBasics cloneJoint = cloneJoint(originalJoint, cloneSuffix, clonePredecessor, originalStart.isRootBody(), jointBuilder);

         if (originalJoint.isLoopClosure())
         { // We rely on the iterator to stop at the loop closure joint.
            loopClosureCloneJoints.add(cloneJoint);
            loopClosureOriginalJoints.add(originalJoint);
            // We will complete their setup at the end to ensure the successors are already created.
            continue;
         }

         // Clone the successor
         RigidBodyReadOnly originalSuccessor = originalJoint.getSuccessor();
         RigidBodyBasics cloneSuccessor = cloneRigidBody(originalSuccessor, null, cloneSuffix, cloneJoint, rigidBodyBuilder);
         originalToCloneBodyMap.put(originalSuccessor, cloneSuccessor);
      }

      for (int loopClosureIndex = 0; loopClosureIndex < loopClosureCloneJoints.size(); loopClosureIndex++)
      {
         JointBasics cloneJoint = loopClosureCloneJoints.get(loopClosureIndex);
         JointReadOnly originalJoint = loopClosureOriginalJoints.get(loopClosureIndex);

         RigidBodyBasics cloneSuccessor = originalToCloneBodyMap.get(originalJoint.getSuccessor());
         RigidBodyTransform cloneTransform = new RigidBodyTransform(originalJoint.getLoopClosureFrame().getTransformToParent());
         cloneTransform.invert();
         cloneJoint.setupLoopClosure(cloneSuccessor, cloneTransform);
      }
   }

   private static JointBasics cloneJoint(JointReadOnly original, String cloneSuffix, RigidBodyBasics clonePredecessor, boolean testIfPredecessorIsRoot,
                                         JointBuilder jointBuilder)
   {
      if (jointBuilder == null)
         jointBuilder = DEFAULT_JOINT_BUILDER;

      if (original instanceof OneDoFJointReadOnly)
         return cloneOneDoFJoint((OneDoFJointReadOnly) original, cloneSuffix, clonePredecessor, jointBuilder);
      else if (original instanceof SixDoFJointReadOnly)
         return cloneSixDoFJoint((SixDoFJointReadOnly) original, cloneSuffix, clonePredecessor, jointBuilder);
      else if (original instanceof PlanarJointReadOnly)
         return clonePlanarJoint((PlanarJointReadOnly) original, cloneSuffix, clonePredecessor, jointBuilder);
      else if (original instanceof SphericalJointReadOnly)
         return cloneSphericalJoint((SphericalJointReadOnly) original, cloneSuffix, clonePredecessor, jointBuilder);
      else if (original instanceof FixedJointReadOnly)
         return cloneFixedJoint((FixedJointReadOnly) original, cloneSuffix, clonePredecessor, jointBuilder);
      else
         throw new UnsupportedOperationException("Unhandled joint type: " + original.getClass().getName());
   }

   private static SixDoFJointBasics cloneSixDoFJoint(SixDoFJointReadOnly original, String cloneSuffix, RigidBodyBasics clonePredecessor,
                                                     JointBuilder jointBuilder)
   {
      String jointNameOriginal = original.getName();
      RigidBodyTransform jointTransform = getCloneJointTransformToParent(original);
      return jointBuilder.buildSixDoFJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform);
   }

   private static PlanarJoint clonePlanarJoint(PlanarJointReadOnly original, String cloneSuffix, RigidBodyBasics clonePredecessor, JointBuilder jointBuilder)
   {
      String jointNameOriginal = original.getName();
      RigidBodyTransform jointTransform = getCloneJointTransformToParent(original);
      return new PlanarJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform);
   }

   private static OneDoFJointBasics cloneOneDoFJoint(OneDoFJointReadOnly original, String cloneSuffix, RigidBodyBasics clonePredecessor,
                                                     JointBuilder jointBuilder)
   {
      String jointNameOriginal = original.getName();
      RigidBodyTransform jointTransform = getCloneJointTransformToParent(original);

      OneDoFJointBasics clone;

      if (original instanceof RevoluteJointReadOnly)
         clone = jointBuilder.buildRevoluteJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform, original.getJointAxis());
      else if (original instanceof PrismaticJointReadOnly)
         clone = jointBuilder.buildPrismaticJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform, original.getJointAxis());
      else
         throw new RuntimeException("Unhandled type of " + OneDoFJoint.class.getSimpleName() + ": " + original.getClass().getSimpleName());

      clone.setJointLimits(original.getJointLimitLower(), original.getJointLimitUpper());
      clone.setVelocityLimits(original.getVelocityLimitLower(), original.getVelocityLimitUpper());
      clone.setEffortLimits(original.getEffortLimitLower(), original.getEffortLimitUpper());

      return clone;
   }

   private static SphericalJointBasics cloneSphericalJoint(SphericalJointReadOnly original, String cloneSuffix, RigidBodyBasics clonePredecessor,
                                                           JointBuilder jointBuilder)
   {
      String jointNameOriginal = original.getName();
      RigidBodyTransform jointTransform = getCloneJointTransformToParent(original);
      return jointBuilder.buildSphericalJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform);
   }

   private static FixedJointBasics cloneFixedJoint(FixedJointReadOnly original, String cloneSuffix, RigidBodyBasics clonePredecessor, JointBuilder jointBuilder)
   {
      String jointNameOriginal = original.getName();
      RigidBodyTransform jointTransform = getCloneJointTransformToParent(original);
      return jointBuilder.buildFixedJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform);
   }

   private static RigidBodyTransform getCloneJointTransformToParent(JointReadOnly original)
   {
      if (original.getFrameBeforeJoint() == original.getPredecessor().getBodyFixedFrame())
         return null;
      else
         return original.getFrameBeforeJoint().getTransformToParent();
   }

   private static RigidBodyBasics cloneRigidBody(RigidBodyReadOnly original, ReferenceFrame cloneStationaryFrame, String cloneSuffix,
                                                 JointBasics parentJointOfClone, RigidBodyBuilder rigidBodyBuilder)
   {
      if (original.isRootBody() && parentJointOfClone != null)
         throw new IllegalArgumentException("Inconsistent set of arguments. If the original body is the root body, the parent joint should be null.");

      if (rigidBodyBuilder == null)
         rigidBodyBuilder = DEFAULT_RIGID_BODY_BUILDER;

      String nameOriginal = original.getName();

      if (parentJointOfClone == null)
      {
         /*
          * Regardless of whether the original body is the root body or not, we create the clone as the root
          * body of its own multi-body system. This allows to keep the clone subtree detached from the
          * original one but still ensure that the clone follows in space the pose of the original body.
          */
         MovingReferenceFrame originalBodyFixedFrame = original.getBodyFixedFrame();
         if (cloneStationaryFrame == null)
            cloneStationaryFrame = originalBodyFixedFrame;
         return rigidBodyBuilder.buildRoot(nameOriginal + cloneSuffix, originalBodyFixedFrame.getTransformToParent(), cloneStationaryFrame);
      }
      else
      {
         SpatialInertiaReadOnly originalInertia = original.getInertia();

         double mass = originalInertia.getMass();
         Matrix3DReadOnly momentOfInertia = originalInertia.getMomentOfInertia();
         RigidBodyTransform inertiaPose = new RigidBodyTransform(original.getBodyFixedFrame().getTransformToParent());
         RigidBodyBasics clone = rigidBodyBuilder.build(nameOriginal + cloneSuffix, parentJointOfClone, momentOfInertia, mass, inertiaPose);
         clone.getInertia().getCenterOfMassOffset().set((Tuple3DReadOnly) originalInertia.getCenterOfMassOffset());
         return clone;
      }
   }

   /**
    * Interface for creating custom joints.
    * <p>
    * This is particularly useful for cloning a multi-body system and changing the joint implementation
    * to be used via
    * {@link MultiBodySystemFactories#cloneMultiBodySystem(RigidBodyReadOnly, ReferenceFrame, String, RigidBodyBuilder, JointBuilder)}.
    * </p>
    *
    * @author Sylvain Bertrand
    */
   public static interface JointBuilder
   {
      /**
       * Builds a new implementation of {@code SixDoFJointBasics}.
       *
       * @param name              the joint name.
       * @param predecessor       the predecessor of the joint.
       * @param transformToParent the transform to the frame after the parent joint.
       * @return the new 6-DoF joint.
       */
      default SixDoFJointBasics buildSixDoFJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
      {
         return new SixDoFJoint(name, predecessor, transformToParent);
      }

      /**
       * Builds a new implementation of {@code PlanarJointBasics}.
       *
       * @param name              the joint name.
       * @param predecessor       the predecessor of the joint.
       * @param transformToParent the transform to the frame after the parent joint.
       * @return the new planar joint.
       */
      default PlanarJointBasics buildPlanarJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
      {
         return new PlanarJoint(name, predecessor, transformToParent);
      }

      /**
       * Builds a new implementation of {@code SphericalJointBasics}.
       *
       * @param name              the joint name.
       * @param predecessor       the predecessor of the joint.
       * @param transformToParent the transform to the frame after the parent joint.
       * @return the new spherical joint.
       */
      default SphericalJointBasics buildSphericalJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
      {
         return new SphericalJoint(name, predecessor, transformToParent);
      }

      /**
       * Builds a new implementation of {@code RevoluteJointBasics}.
       *
       * @param name              the joint name.
       * @param predecessor       the predecessor of the joint.
       * @param transformToParent the transform to the frame after the parent joint.
       * @param jointAxis         the joint axis.
       * @return the new revolute joint.
       */
      default RevoluteJointBasics buildRevoluteJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent, Vector3DReadOnly jointAxis)
      {
         return new RevoluteJoint(name, predecessor, transformToParent, jointAxis);
      }

      /**
       * Builds a new implementation of {@code PrismaticJointBasics}.
       *
       * @param name              the joint name.
       * @param predecessor       the predecessor of the joint.
       * @param transformToParent the transform to the frame after the parent joint.
       * @param jointAxis         the joint axis.
       * @return the new prismatic joint.
       */
      default PrismaticJointBasics buildPrismaticJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent,
                                                       Vector3DReadOnly jointAxis)
      {
         return new PrismaticJoint(name, predecessor, transformToParent, jointAxis);
      }

      /**
       * Builds a new implementation of {@code FixedJointBasics}.
       *
       * @param name              the joint name.
       * @param predecessor       the predecessor of the joint.
       * @param transformToParent the transform to the frame after the parent joint.
       * @return the new fixed joint.
       */
      default FixedJointBasics buildFixedJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent)
      {
         return new FixedJoint(name, predecessor, transformToParent);
      }
   }

   /**
    * Interface for creating custom rigid-bodies.
    * <p>
    * This is particularly useful for cloning a multi-body system and changing the rigid-body
    * implementation to be used via
    * {@link MultiBodySystemFactories#cloneMultiBodySystem(RigidBodyReadOnly, ReferenceFrame, String, RigidBodyBuilder, JointBuilder)}.
    * </p>
    *
    * @author Sylvain Bertrand
    */
   public static interface RigidBodyBuilder
   {
      /**
       * Builds a new root body.
       *
       * @param bodyName              the body's name.
       * @param transformToParent     provides the pose of this rigid-body's body-fixed-frame with respect
       *                              to the parentStationaryFrame.
       * @param parentStationaryFrame the parent stationary, i.e. non-moving with respect to world frame,
       *                              frame to which this rigid-body will create and attach its body fixed
       *                              frame.
       * @return the new root body.
       */
      default RigidBodyBasics buildRoot(String bodyName, RigidBodyTransform transformToParent, ReferenceFrame parentStationaryFrame)
      {
         return new RigidBody(bodyName, transformToParent, parentStationaryFrame);
      }

      /**
       * Builds a new rigid-body.
       *
       * @param bodyName        the body's name.
       * @param parentJoint     the joint directly attached to this rigid-body and located between this
       *                        rigid-body and the root body of the robot.
       * @param momentOfInertia the 3D moment of inertia of this rigid-body.
       * @param mass            the mass of this rigid-body.
       * @param inertiaPose     defines the transform from this rigid-body body-fixed-frame to the
       *                        parentJoint.getFrameAfterJointFrame(). The given moment of inertia is
       *                        assumed to be expressed in that body-fixed-frame. Also note that the
       *                        translation part corresponds to the position of this rigid-body center of
       *                        mass position expressed in parentJoint.getFrameAfterJointFrame().
       * @return the new rigid-body.
       */
      default RigidBodyBasics build(String bodyName, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransform inertiaPose)
      {
         return new RigidBody(bodyName, parentJoint, momentOfInertia, mass, inertiaPose);
      }
   }
}
