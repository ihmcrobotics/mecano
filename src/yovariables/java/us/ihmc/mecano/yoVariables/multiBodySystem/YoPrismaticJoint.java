package us.ihmc.mecano.yoVariables.multiBodySystem;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.PrismaticJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * A {@code YoPrismaticJoint} is a joint has 1 degree of freedom of translation and has its state
 * backed by {@code YoVariable}s.
 *
 * @author Sylvain Bertrand
 */
public class YoPrismaticJoint extends YoOneDoFJoint implements PrismaticJointBasics
{
   /** The axis along which this joint can translate. */
   private final FrameVector3D jointAxis;

   /**
    * Creates a new prismatic joint which has only a translation offset with respect to its parent and
    * which state is backed by {@code YoVariable}s.
    *
    * @param name        the name for the new joint.
    * @param predecessor the rigid-body connected to and preceding this joint.
    * @param jointOffset the offset in translation with respect to the frame after the parent joint.
    *                    Not modified.
    * @param jointAxis   the axis along which this joint can translate. Not modified.
    * @param registry    the registry to register child variables to.
    */
   public YoPrismaticJoint(String name, RigidBodyBasics predecessor, Tuple3DReadOnly jointOffset, Vector3DReadOnly jointAxis, YoRegistry registry)
   {
      this(name, predecessor, new RigidBodyTransform(new Quaternion(), jointOffset), jointAxis, registry);
   }

   /**
    * Creates a new prismatic joint which state is backed by {@code YoVariable}s.
    *
    * @param name              the name for the new joint.
    * @param predecessor       the rigid-body connected to and preceding this joint.
    * @param transformToParent the transform to the frame after the parent joint. Not modified.
    * @param jointAxis         the axis along which this joint can translate. Not modified.
    * @param registry          the registry to register child variables to.
    */
   public YoPrismaticJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent, Vector3DReadOnly jointAxis,
                           YoRegistry registry)
   {
      super(name, predecessor, new Vector3D(), jointAxis, transformToParent, registry);
      this.jointAxis = new FrameVector3D(beforeJointFrame, jointAxis);
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DBasics getJointAxis()
   {
      return jointAxis;
   }
}
