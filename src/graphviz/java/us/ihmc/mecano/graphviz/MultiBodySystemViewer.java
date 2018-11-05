package us.ihmc.mecano.graphviz;

import static guru.nidi.graphviz.model.Factory.*;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import guru.nidi.graphviz.attribute.Color;
import guru.nidi.graphviz.engine.Format;
import guru.nidi.graphviz.engine.Graphviz;
import guru.nidi.graphviz.model.MutableGraph;
import guru.nidi.graphviz.model.MutableNode;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Tool used to create a graphical representation of a multi-body system as a block diagram saved
 * into a file.
 * 
 * @author Sylvain Bertrand
 */
public class MultiBodySystemViewer
{
   private final RigidBodyReadOnly rootBody;
   private Format format = Format.SVG;

   /**
    * Display modes which are either: display rigid-bodies only, display joints only, or display both
    * rigid-bodies and joints.
    * 
    * @author Sylvain Bertrand
    */
   public enum Display
   {
      /** For displaying rigid-bodies only. */
      RIGID_BODIES,
      /** For displaying joints only. */
      JOINTS,
      /** For displaying rigid-bodies and joints. */
      BOTH
   };

   private Display display = Display.JOINTS;
   private final List<JointLabelProvider> jointLabelProviders = new ArrayList<>();
   private final List<RigidBodyLabelProvider> rigidBodyLabelProviders = new ArrayList<>();

   /**
    * Creates a new viewer for visualizing the subtree starting off the given {@code rootBody}.
    * 
    * @param rootBody the origin of the subtree to visualize.
    */
   public MultiBodySystemViewer(RigidBodyReadOnly rootBody)
   {
      this.rootBody = rootBody;
      jointLabelProviders.add(joint -> joint.getName() + ", nameId: " + joint.hashCode());
      rigidBodyLabelProviders.add(rigidBody -> rigidBody.getName() + ", nameId: " + rigidBody.hashCode());
   }

   /**
    * Changes the display mode to switch between: rigid-bodies only, joints only, or both.
    * 
    * @param display the new display mode, default value {@link Display#JOINTS}.
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer setDisplay(Display display)
   {
      this.display = display;
      return this;
   }

   /**
    * Registers a custom label provider to display addition information about each joint.
    * 
    * @param labelProvider the custom label provider.
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer addLabelProvider(JointLabelProvider labelProvider)
   {
      jointLabelProviders.add(labelProvider);
      return this;
   }

   /**
    * Registers a custom label provider to display addition information about each rigid-body.
    * 
    * @param labelProvider the custom label provider.
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer addLabelProvider(RigidBodyLabelProvider labelProvider)
   {
      rigidBodyLabelProviders.add(labelProvider);
      return this;
   }

   /**
    * Adds information about the final type of each joint.
    * 
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer showJointType()
   {
      jointLabelProviders.add(joint -> joint.getClass().getSimpleName());
      return this;
   }

   /**
    * Adds information about the state for each joint.
    * 
    * @param jointStateTypes to select configuration, velocity, acceleration, and/or joint effort.
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer showJointStates(JointStateType... jointStateTypes)
   {
      for (JointStateType state : jointStateTypes)
         jointLabelProviders.add(joint -> getJointStateAsString(joint, state));
      return this;
   }

   /**
    * Adds joint axis information for one degree of freedom joints.
    * 
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer showOneDoFJointAxis()
   {
      jointLabelProviders.add(MultiBodySystemViewer::getJointAxisLabel);
      return this;
   }

   /**
    * Adds information about each rigid-body's mass.
    * 
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer showRigidBodyMass()
   {
      rigidBodyLabelProviders.add(rigidBody -> {
         if (rigidBody.isRootBody())
            return null;
         else
            return "mass = " + ReferenceFrameTreeViewer.getLabelOf(rigidBody.getInertia().getMass());
      });
      return this;
   }

   /**
    * Adds information about each rigid-body's inertia.
    * <p>
    * The displayed inertia is expressed in the body-fixed frame.
    * </p>
    * 
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer showRigidBodyInertia()
   {
      rigidBodyLabelProviders.add(rigidBody -> {
         if (rigidBody.isRootBody())
            return null;
         else
            return "inertia =\n" + getMatrixLabel(rigidBody.getInertia().getMomentOfInertia());
      });
      return this;
   }

   /**
    * Adds information about each rigid-body's center of mass.
    * <p>
    * The center of mass position is expressed in the frame right after the parent joint.
    * </p>
    * 
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer showRigidBodyCenterOfMass()
   {
      rigidBodyLabelProviders.add(rigidBody -> {
         if (rigidBody.isRootBody())
            return null;
         FramePoint3D centerOfMass = new FramePoint3D(rigidBody.getInertia().getCenterOfMassOffset());
         centerOfMass.changeFrame(rigidBody.getParentJoint().getFrameAfterJoint());
         return "center of mass = " + EuclidCoreIOTools.getTuple3DString(centerOfMass);
      });
      return this;
   }

   /**
    * Sets the image format to use.
    * 
    * @param format default value {@code Format.SVG}.
    * @return {@code this} for chaining operations.
    */
   public MultiBodySystemViewer renderingFormat(Format format)
   {
      this.format = format;
      return this;
   }

   /**
    * Creates the graph and output the block diagram into a file.
    * 
    * @param outputFileName the path to the file to create. The file extension is added internally.
    */
   public void view(String outputFileName)
   {
      MutableGraph graph = mutGraph("MultiBodySystemView").setDirected(true);
      MutableNode rootNode = createRigidBodyNode(rootBody, graph);
      addChildrenToGraph(rootBody, rootNode, graph);

      try
      {
         Graphviz.fromGraph(graph).render(format).toFile(new File(outputFileName + "." + format.name().toLowerCase()));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void addChildrenToGraph(RigidBodyReadOnly currentBody, MutableNode currentNode, MutableGraph graph)
   {
      for (JointReadOnly childJoint : currentBody.getChildrenJoints())
      {
         RigidBodyReadOnly childBody = childJoint.getSuccessor();
         MutableNode childJointNode, childBodyNode;

         switch (display)
         {
         case RIGID_BODIES:
            childBodyNode = createRigidBodyNode(childBody, graph);
            graph.addLink(currentNode.addLink(childBodyNode));
            addChildrenToGraph(childBody, childBodyNode, graph);
            break;

         case JOINTS:
            childJointNode = createJointNode(childJoint, graph);
            graph.addLink(currentNode.addLink(childJointNode));
            addChildrenToGraph(childBody, childJointNode, graph);
            break;

         case BOTH:
            childJointNode = createJointNode(childJoint, graph);
            childBodyNode = createRigidBodyNode(childBody, graph);
            graph.addLink(currentNode.addLink(childJointNode));
            graph.addLink(childJointNode.addLink(childBodyNode));
            addChildrenToGraph(childBody, childBodyNode, graph);
            break;
         default:
            throw new RuntimeException("Unexpected value for Display: " + display);
         }
      }
   }

   private MutableNode createRigidBodyNode(RigidBodyReadOnly rigidBody, MutableGraph graph)
   {
      String label = rigidBodyLabelProviders.get(0).getLabel(rigidBody);

      for (int i = 1; i < rigidBodyLabelProviders.size(); i++)
      {
         String additionalLabel = rigidBodyLabelProviders.get(i).getLabel(rigidBody);
         if (additionalLabel != null)
            label += "\n" + additionalLabel;
      }

      MutableNode rigidBodyNode = mutNode(label);
      rigidBodyNode.add(Color.DARKORCHID);
      graph.add(rigidBodyNode);

      return rigidBodyNode;
   }

   private MutableNode createJointNode(JointReadOnly joint, MutableGraph graph)
   {
      String label = jointLabelProviders.get(0).getLabel(joint);

      for (int i = 1; i < jointLabelProviders.size(); i++)
      {
         String additionalLabel = jointLabelProviders.get(i).getLabel(joint);
         if (additionalLabel != null)
            label += "\n" + additionalLabel;
      }

      MutableNode jointNode = mutNode(label);
      jointNode.add(Color.DARKGREEN);
      graph.add(jointNode);

      return jointNode;
   }

   private static String getJointStateAsString(JointReadOnly joint, JointStateType state)
   {
      if (joint instanceof SixDoFJointReadOnly)
         return getSixDoFJointStateAsString((SixDoFJointReadOnly) joint, state);
      else if (joint instanceof OneDoFJointReadOnly)
         return getOneDoFJointStateAsString((OneDoFJointReadOnly) joint, state);
      else if (joint instanceof PlanarJointReadOnly)
         return getPlanarJointStateAsString((PlanarJointReadOnly) joint, state);
      else if (joint instanceof SphericalJointReadOnly)
         return getSphericalJointStateAsString((SphericalJointReadOnly) joint, state);
      else
         throw new UnsupportedOperationException("Unsupported joint type: " + joint.getClass().getName());
   }

   private static String getSixDoFJointStateAsString(SixDoFJointReadOnly joint, JointStateType state)
   {
      switch (state)
      {
      case CONFIGURATION:
         Pose3DReadOnly pose = joint.getJointPose();
         return "(x,y,z) = " + pose.getPosition().toString() + "\n" + ReferenceFrameTreeViewer.getOrientationLabel(pose.getOrientation());

      case VELOCITY:
         TwistReadOnly jointTwist = joint.getJointTwist();
         return "linear velocity = " + EuclidCoreIOTools.getTuple3DString(jointTwist.getLinearPart()) + "\nangular velocity = "
               + EuclidCoreIOTools.getTuple3DString(jointTwist.getAngularPart());

      case ACCELERATION:
         SpatialAccelerationReadOnly jointAcceleration = joint.getJointAcceleration();
         return "linear acceleration = " + EuclidCoreIOTools.getTuple3DString(jointAcceleration.getLinearPart()) + "\nangular acceleration = "
               + EuclidCoreIOTools.getTuple3DString(jointAcceleration.getAngularPart());

      case EFFORT:
         WrenchReadOnly jointWrench = joint.getJointWrench();
         return "force = " + EuclidCoreIOTools.getTuple3DString(jointWrench.getLinearPart()) + "\nmoment = "
               + EuclidCoreIOTools.getTuple3DString(jointWrench.getAngularPart());

      default:
         throw new UnsupportedOperationException("Unsupported value of JointStateType: " + state);
      }
   }

   private static String getPlanarJointStateAsString(PlanarJointReadOnly joint, JointStateType state)
   {
      switch (state)
      {
      case CONFIGURATION:
         Pose3DReadOnly pose = joint.getJointPose();
         return "(x,z) = " + ReferenceFrameTreeViewer.getLabelOf(pose.getX(), pose.getZ()) + "\n(pitch) = "
               + ReferenceFrameTreeViewer.getLabelOf(pose.getPitch());

      case VELOCITY:
         TwistReadOnly jointTwist = joint.getJointTwist();
         return "linear velocity = " + ReferenceFrameTreeViewer.getLabelOf(jointTwist.getLinearPartX(), jointTwist.getLinearPartZ()) + "\nangular velocity = "
               + ReferenceFrameTreeViewer.getLabelOf(jointTwist.getAngularPartY());

      case ACCELERATION:
         SpatialAccelerationReadOnly jointAcceleration = joint.getJointAcceleration();
         return "linear acceleration = " + ReferenceFrameTreeViewer.getLabelOf(jointAcceleration.getLinearPartX(), jointAcceleration.getLinearPartZ())
               + "\nangular acceleration = " + ReferenceFrameTreeViewer.getLabelOf(jointAcceleration.getAngularPartY());

      case EFFORT:
         WrenchReadOnly jointWrench = joint.getJointWrench();
         return "force = " + ReferenceFrameTreeViewer.getLabelOf(jointWrench.getLinearPartX(), jointWrench.getLinearPartZ()) + "\nmoment = "
               + ReferenceFrameTreeViewer.getLabelOf(jointWrench.getAngularPartY());

      default:
         throw new UnsupportedOperationException("Unsupported value of JointStateType: " + state);
      }
   }

   private static String getSphericalJointStateAsString(SphericalJointReadOnly joint, JointStateType state)
   {
      switch (state)
      {
      case CONFIGURATION:
         return ReferenceFrameTreeViewer.getOrientationLabel(joint.getJointOrientation());

      case VELOCITY:
         return "angular velocity = " + EuclidCoreIOTools.getTuple3DString(joint.getJointAngularVelocity());

      case ACCELERATION:
         return "angular acceleration = " + EuclidCoreIOTools.getTuple3DString(joint.getJointAngularAcceleration());

      case EFFORT:
         return "moment = " + EuclidCoreIOTools.getTuple3DString(joint.getJointTorque());

      default:
         throw new UnsupportedOperationException("Unsupported value of JointStateType: " + state);
      }
   }

   private static String getOneDoFJointStateAsString(OneDoFJointReadOnly joint, JointStateType state)
   {
      switch (state)
      {
      case CONFIGURATION:
         return "q = " + ReferenceFrameTreeViewer.getLabelOf(joint.getQ());

      case VELOCITY:
         return "qd = " + ReferenceFrameTreeViewer.getLabelOf(joint.getQd());

      case ACCELERATION:
         return "qdd = " + ReferenceFrameTreeViewer.getLabelOf(joint.getQdd());

      case EFFORT:
         return "tau = " + ReferenceFrameTreeViewer.getLabelOf(joint.getTau());

      default:
         throw new UnsupportedOperationException("Unsupported value of JointStateType: " + state);
      }
   }

   private static String getMatrixLabel(Matrix3DReadOnly matrix)
   {
      String ret = EuclidCoreIOTools.getStringOf("/", " \\\\\n", ", ", matrix.getM00(), matrix.getM01(), matrix.getM02());
      ret += EuclidCoreIOTools.getStringOf("|", " |\n", ", ", matrix.getM10(), matrix.getM11(), matrix.getM12());
      ret += EuclidCoreIOTools.getStringOf("\\\\", " /", ", ", matrix.getM20(), matrix.getM21(), matrix.getM22());
      return ret;
   }

   private static String getJointAxisLabel(JointReadOnly joint)
   {
      if (joint instanceof OneDoFJointReadOnly)
      {
         return "axis = " + EuclidCoreIOTools.getTuple3DString(((OneDoFJointReadOnly) joint).getJointAxis());
      }
      else
      {
         return null;
      }
   }

   /**
    * Interface used to implement custom label maker for joints.
    * <p>
    * Multiple {@code LabelProvider} can be registered to a viewer to display more information about
    * joints.
    * </p>
    * 
    * @author Sylvain Bertrand
    */
   public static interface JointLabelProvider
   {
      /**
       * Gets the label to display for the given joint.
       * 
       * @param joint the joint to make the label from.
       * @return the label to display in the graphical node.
       */
      String getLabel(JointReadOnly joint);
   }

   /**
    * Interface used to implement custom label maker for rigid-bodies.
    * <p>
    * Multiple {@code LabelProvider} can be registered to a viewer to display more information about
    * rigid-bodies.
    * </p>
    * 
    * @author Sylvain Bertrand
    */
   public static interface RigidBodyLabelProvider
   {
      /**
       * Gets the label to display for the given rigid-body.
       * 
       * @param rigidBody the rigid-body to make the label from.
       * @return the label to display in the graphical node.
       */
      String getLabel(RigidBodyReadOnly rigidBody);
   }

   /**
    * Generates a basic view of the subtree of joint starting off {@code rootBody} and saves it in the
    * working directory as <tt>jointSystemView.svg</tt>
    * <p>
    * For generating a custom view, create a new {@link MultiBodySystemViewer}, configure it using
    * {@link #addLabelProvider(JointLabelProvider)}, and view it using {@link #view(String)}.
    * </p>
    * 
    * @param rootBody the root of the subtree to view.
    */
   public static void viewSimpleJointSubtree(RigidBodyReadOnly rootBody)
   {
      new MultiBodySystemViewer(rootBody).setDisplay(Display.JOINTS).view("jointSystemView");
   }

   /**
    * Generates a default view of the subtree of joint starting off {@code rootBody} and saves it in
    * the working directory as <tt>jointSystemView.svg</tt>
    * <p>
    * For generating a custom view, create a new {@link MultiBodySystemViewer}, configure it using
    * {@link #addLabelProvider(JointLabelProvider)}, and view it using {@link #view(String)}.
    * </p>
    * 
    * @param rootBody the root of the subtree to view.
    */
   public static void viewJointSubtree(RigidBodyReadOnly rootBody)
   {
      new MultiBodySystemViewer(rootBody).setDisplay(Display.JOINTS).showJointStates(JointStateType.CONFIGURATION, JointStateType.VELOCITY)
            .view("jointSystemView");
   }

   /**
    * Generates a basic view of the subtree of rigid-bodies starting off {@code rootBody} and saves it
    * in the working directory as <tt>rigidBodySystemView.svg</tt>
    * <p>
    * For generating a custom view, create a new {@link MultiBodySystemViewer}, configure it using
    * {@link #addLabelProvider(RigidBodyLabelProvider)}, and view it using {@link #view(String)}.
    * </p>
    * 
    * @param rootBody the root of the subtree to view.
    */
   public static void viewSimpleRigidBodySubtree(RigidBodyReadOnly rootBody)
   {
      new MultiBodySystemViewer(rootBody).setDisplay(Display.RIGID_BODIES).view("rigidBodySystemView");
   }

   /**
    * Generates a default view of the subtree of rigid-bodies starting off {@code rootBody} and saves
    * it in the working directory as <tt>rigidBodySystemView.svg</tt>
    * <p>
    * For generating a custom view, create a new {@link MultiBodySystemViewer}, configure it using
    * {@link #addLabelProvider(RigidBodyLabelProvider)}, and view it using {@link #view(String)}.
    * </p>
    * 
    * @param rootBody the root of the subtree to view.
    */
   public static void viewRigidBodySubtree(RigidBodyReadOnly rootBody)
   {
      new MultiBodySystemViewer(rootBody).setDisplay(Display.RIGID_BODIES).showRigidBodyMass().showRigidBodyCenterOfMass().showRigidBodyInertia()
            .view("rigidBodySystemView");
   }

   /**
    * Example of how to use a multi-body system viewer.
    * 
    * @param args the arguments are not used in this example.
    */
   public static void main(String[] args)
   {
      Random random = new Random(43535);
      List<? extends JointBasics> joints = MultiBodySystemRandomTools.nextJointTree(random, 30);

      RigidBodyReadOnly rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getSuccessor());
      for (JointStateType stateToRandomize : JointStateType.values())
         MultiBodySystemRandomTools.nextState(random, stateToRandomize, joints);

      new MultiBodySystemViewer(rootBody).showJointType().showOneDoFJointAxis().showRigidBodyMass().showRigidBodyCenterOfMass().showRigidBodyInertia()
            .showJointStates(JointStateType.CONFIGURATION, JointStateType.VELOCITY, JointStateType.ACCELERATION, JointStateType.EFFORT).setDisplay(Display.BOTH)
            .view("MultiBodySystemViewExample");
   }
}
