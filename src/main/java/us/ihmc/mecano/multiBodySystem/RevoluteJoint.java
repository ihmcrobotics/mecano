package us.ihmc.mecano.multiBodySystem;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MecanoFactories.RevoluteJointTransformUpdater;

/**
 * A {@code RevoluteJoint} is a joint has 1 degree of freedom of rotation.
 * 
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public class RevoluteJoint extends OneDoFJoint implements RevoluteJointBasics
{
   /** The axis around which this joint can rotate. */
   private final FrameVector3D jointAxis;
   /** Variable to store intermediate results for garbage-free operations. */
   private final Vector3D rotationVector = new Vector3D();
   /** Local transform updater for optimizing the transform update based on this joint axis. */
   private final RevoluteJointTransformUpdater jointTransformUpdater;

   /**
    * Creates a new revolute joint.
    * <p>
    * This constructor is typically used to create a root joint.
    * </p>
    * 
    * @param name        the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param jointAxis   the axis around which this joint can rotate. Not modified.
    */
   public RevoluteJoint(String name, RigidBodyBasics predecessor, Vector3DReadOnly jointAxis)
   {
      this(name, predecessor, (RigidBodyTransform) null, jointAxis);
   }

   /**
    * Creates a new revolute joint which has only a translation offset with respect to its parent.
    * 
    * @param name        the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param jointOffset the offset in translation with respect to the frame after the parent joint.
    *                    Not modified.
    * @param jointAxis   the axis around which this joint can rotate. Not modified.
    */
   public RevoluteJoint(String name, RigidBodyBasics predecessor, Tuple3DReadOnly jointOffset, Vector3DReadOnly jointAxis)
   {
      this(name, predecessor, new RigidBodyTransform(new Quaternion(), jointOffset), jointAxis);
   }

   /**
    * Creates a new revolute joint.
    * 
    * @param name              the name for the new joint.
    * @param predecessor       the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    * @param jointAxis         the axis around which this joint can rotate. Not modified.
    */
   public RevoluteJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent, Vector3DReadOnly jointAxis)
   {
      super(name, predecessor, jointAxis, new Vector3D(), transformToParent);
      this.jointAxis = new FrameVector3D(beforeJointFrame, jointAxis);
      jointTransformUpdater = MecanoFactories.newRevoluteJointTransformUpdater(this);
   }

   /** {@inheritDoc} */
   @Override
   public void getJointConfiguration(RigidBodyTransform jointTransform)
   {
      jointTransformUpdater.updateJointTransform(jointTransform);
   }

   /** {@inheritDoc} */
   @Override
   public void setJointOrientation(Orientation3DReadOnly jointOrientation)
   {
      jointOrientation.getRotationVector(rotationVector);
      setQ(rotationVector.dot(jointAxis));
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getJointAxis()
   {
      return jointAxis;
   }
}
