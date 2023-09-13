package us.ihmc.mecano.yoVariables.multiBodySystem;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.yoVariables.multiBodySystem.inertial.YoSpatialInertia;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class YoRigidBody implements RigidBodyBasics
{
   /** This is where the physical properties of this rigid-body are stored. */
   private final YoSpatialInertia spatialInertia;

   private final MovingReferenceFrame bodyFixedFrame;

   private final JointBasics parentJoint;

   private final List<JointBasics> parentLoopClosureJoints = new ArrayList<>();

   private final List<JointBasics> childrenJoints = new ArrayList<>();

   private final String name;

   private final String nameId;
   
   public YoRigidBody(String bodyName, RigidBodyTransformReadOnly transformToParent, ReferenceFrame parentStationaryFrame)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      name = bodyName;
      spatialInertia = null;
      bodyFixedFrame = MovingReferenceFrame.constructFrameFixedInParent(bodyName + "Frame", parentStationaryFrame, transformToParent);
      parentJoint = null;
      nameId = bodyName;
   }

   private YoRigidBody(String bodyName, JointBasics parentJoint, RigidBodyTransformReadOnly inertiaPose, YoRegistry registry)
   {
      if (bodyName == null)
         throw new IllegalArgumentException("Name can not be null");

      name = bodyName;
      this.parentJoint = parentJoint;

      ReferenceFrame frameAfterJoint = parentJoint.getFrameAfterJoint();
      bodyFixedFrame = MovingReferenceFrame.constructFrameFixedInParent(bodyName + "CoM", frameAfterJoint, inertiaPose);
      spatialInertia = new YoSpatialInertia(bodyFixedFrame, bodyFixedFrame, registry);
      // inertia should be expressed in body frame, otherwise it will change
      spatialInertia.getBodyFrame().checkReferenceFrameMatch(spatialInertia.getReferenceFrame());
      parentJoint.setSuccessor(this);
      nameId = parentJoint.getPredecessor().getNameId() + NAME_ID_SEPARATOR + bodyName;
   }

   public YoRigidBody(String bodyName, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransformReadOnly inertiaPose, YoRegistry registry)
   {
      this(bodyName, parentJoint, inertiaPose, registry);
      spatialInertia.getMomentOfInertia().set(momentOfInertia);
      spatialInertia.setMass(mass);
   }

   @Override
   public SpatialInertiaBasics getInertia()
   {
      return spatialInertia;
   }

   @Override
   public MovingReferenceFrame getBodyFixedFrame()
   {
      return bodyFixedFrame;
   }

   @Override
   public JointBasics getParentJoint()
   {
      return parentJoint;
   }

   @Override
   public List<JointBasics> getChildrenJoints()
   {
      return childrenJoints;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getNameId()
   {
      return nameId;
   }

   @Override
   public YoRigidBody[] subtreeArray()
   {
      return subtreeStream().toArray(YoRigidBody[]::new);
   }
}
