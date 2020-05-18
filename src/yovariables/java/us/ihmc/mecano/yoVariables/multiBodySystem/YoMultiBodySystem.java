package us.ihmc.mecano.yoVariables.multiBodySystem;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories.JointBuilder;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;
import us.ihmc.mecano.yoVariables.tools.YoMultiBodySystemFactories;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * A {@code YoMultiBodySystem} can be created from any {@code MultiBodySystemReadOnly}.
 * <p>
 * The original multi-body system is cloned and the joints are upgraded to be backed by
 * {@code YoVariable}s. The resulting system remains compatible with the common algorithms as it
 * implements {@code MultiBodySystemBasics}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class YoMultiBodySystem implements MultiBodySystemBasics
{
   private final RigidBodyBasics yoRootBody;
   private final List<? extends JointBasics> allJoints;
   private final List<? extends JointBasics> jointsToConsider;
   private final List<? extends JointBasics> jointsToIgnore;
   private final JointMatrixIndexProvider jointMatrixIndexProvider;

   /**
    * Creates a new yo-variablelized multi-body system with the same properties as the given
    * {@code input}.
    *
    * @param input           the multi-body system to clone. Not modified.
    * @param stationaryFrame the root frame to which yo-multi-body system is attached.
    * @param registry        the registry to register child variables to.
    */
   public YoMultiBodySystem(MultiBodySystemReadOnly input, ReferenceFrame stationaryFrame, YoVariableRegistry registry)
   {
      this(input, stationaryFrame, registry, MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER, YoMultiBodySystemFactories.newYoJointBuilder(registry));
   }

   /**
    * Creates a new yo-variablelized multi-body system with the same properties as the given
    * {@code input}.
    *
    * @param input            the multi-body system to clone. Not modified.
    * @param stationaryFrame  the root frame to which yo-multi-body system is attached.
    * @param registry         the registry to register child variables to.
    * @param rigidBodyBuilder use a custom rigid-body builder. Can be {@code null}, the default builder
    *                         is {@link MultiBodySystemFactories#DEFAULT_RIGID_BODY_BUILDER}.
    * @param yoJointBuilder   use a custom yo-joint builder, the default used is
    *                         {@link YoMultiBodySystemFactories#newYoJointBuilder(YoVariableRegistry)}.
    */
   public YoMultiBodySystem(MultiBodySystemReadOnly input, ReferenceFrame stationaryFrame, YoVariableRegistry registry, RigidBodyBuilder rigidBodyBuilder,
                            JointBuilder yoJointBuilder)
   {
      yoRootBody = MultiBodySystemFactories.cloneMultiBodySystem(input.getRootBody(), stationaryFrame, "", rigidBodyBuilder, yoJointBuilder);
      allJoints = SubtreeStreams.fromChildren(yoRootBody).collect(Collectors.toList());

      Set<String> nameOfJointsToConsider = input.getJointsToConsider().stream().map(JointReadOnly::getName).collect(Collectors.toSet());
      jointsToConsider = SubtreeStreams.fromChildren(yoRootBody).filter(joint -> nameOfJointsToConsider.contains(joint.getName())).collect(Collectors.toList());

      Set<String> nameOfJointsToIgnore = input.getJointsToIgnore().stream().map(JointReadOnly::getName).collect(Collectors.toSet());
      jointsToIgnore = SubtreeStreams.fromChildren(yoRootBody).filter(joint -> nameOfJointsToIgnore.contains(joint.getName())).collect(Collectors.toList());

      jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(jointsToConsider);
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyBasics getRootBody()
   {
      return yoRootBody;
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends JointBasics> getAllJoints()
   {
      return allJoints;
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends JointBasics> getJointsToConsider()
   {
      return jointsToConsider;
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends JointBasics> getJointsToIgnore()
   {
      return jointsToIgnore;
   }

   /** {@inheritDoc} */
   @Override
   public JointMatrixIndexProvider getJointMatrixIndexProvider()
   {
      return jointMatrixIndexProvider;
   }
}
