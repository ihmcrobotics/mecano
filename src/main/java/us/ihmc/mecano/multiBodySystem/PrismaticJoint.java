package us.ihmc.mecano.multiBodySystem;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.PrismaticJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * A {@code PrismaticJoint} is a joint has 1 degree of freedom of translation.
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public class PrismaticJoint extends OneDoFJoint implements PrismaticJointBasics
{
   /** The axis along which this joint can translate. */
   private final FrameVector3D jointAxis;

   /**
    * Creates a new prismatic joint which has only a translation offset with respect to its parent.
    *
    * @param name        the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param jointOffset the offset in translation with respect to the frame after the parent joint.
    *                    Not modified.
    * @param jointAxis   the axis along which this joint can translate. Not modified.
    */
   public PrismaticJoint(String name, RigidBodyBasics predecessor, Tuple3DReadOnly jointOffset, Vector3DReadOnly jointAxis)
   {
      this(name, predecessor, new RigidBodyTransform(new Quaternion(), jointOffset), jointAxis);
   }

   /**
    * Creates a new prismatic joint.
    *
    * @param name              the name for the new joint.
    * @param predecessor       the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    * @param jointAxis         the axis along which this joint can translate. Not modified.
    */
   public PrismaticJoint(String name, RigidBodyBasics predecessor, RigidBodyTransformReadOnly transformToParent, Vector3DReadOnly jointAxis)
   {
      super(name, predecessor, new Vector3D(), jointAxis, transformToParent);
      this.jointAxis = new FrameVector3D(beforeJointFrame, jointAxis);
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getJointAxis()
   {
      return jointAxis;
   }
}
