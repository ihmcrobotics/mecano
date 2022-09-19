package us.ihmc.mecano.yoVariables.spatial;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Implementation of {@link FixedFrameTwistBasics} backed by {@link YoVariable}s.
 * <p>
 * A twist always describes the relative velocity of a body with respect to a base. These two
 * entities are referred to here by using two reference frames: a {@code bodyFrame} that is
 * considered to be rigidly attached to the body, and a {@code baseFrame} that is considered to be
 * rigidly attached to the base.
 * </p>
 * <p>
 * When using a {@code YoFixedFrameTwist}, it is important to note that the reference frame in which
 * it is expressed does not only refer to the coordinate system in which the angular and linear 3D
 * vectors are expressed. The origin of the reference frame is also used as the point where the
 * velocity is measured. While the angular part remains the same as the point of measurement
 * changes, the linear part does depend on its location.
 * </p>
 * <p>
 * The convention when using a spatial vector in matrix operations is that the angular part occupies
 * the 3 first rows and the linear part the 3 last as follows:<br>
 *
 * <pre>
 *     / angularX \
 *     | angularY |
 *     | angularZ |
 * V = | linearX  |
 *     | linearY  |
 *     \ linearX  /
 * </pre>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class YoFixedFrameTwist implements FixedFrameTwistBasics, Settable<YoFixedFrameTwist>
{
   /**
    * Reference frame rigidly attached to the body that this twist describes the motion of.
    */
   private final ReferenceFrame bodyFrame;
   /**
    * Reference frame rigidly attached to the base that this twist uses as reference for quantifying
    * the body's velocity.
    */
   private final ReferenceFrame baseFrame;
   /** This is where we store the internal data. */
   private final YoFixedFrameSpatialVector spatialVector;
   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D observerPosition = new Point3D();

   /**
    * Creates a new twist with its components set to zero and initializes its reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param bodyFrame        what we are specifying the twist of.
    * @param baseFrame        with respect to what we are specifying the twist.
    * @param expressedInFrame in which reference frame the twist is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameTwist(String namePrefix, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this(bodyFrame, baseFrame, new YoFixedFrameSpatialVector(namePrefix, expressedInFrame, registry));
   }

   /**
    * Creates a new twist with its components set to zero and initializes its reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param nameSuffix       a string to use as the suffix for child variable names.
    * @param bodyFrame        what we are specifying the twist of.
    * @param baseFrame        with respect to what we are specifying the twist.
    * @param expressedInFrame in which reference frame the twist is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameTwist(String namePrefix, String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
                            YoRegistry registry)
   {
      this(bodyFrame, baseFrame, new YoFixedFrameSpatialVector(namePrefix, nameSuffix, expressedInFrame, registry));
   }

   /**
    * Creates a new twist given its angular and linear parts and initializes its reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param bodyFrame   what we are specifying the twist of.
    * @param baseFrame   with respect to what we are specifying the twist.
    * @param angularPart the vector to use for the angular part.
    * @param linearPart  the vector to use for the linear part.
    */
   public YoFixedFrameTwist(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, YoFrameVector3D angularPart, YoFrameVector3D linearPart)
   {
      this(bodyFrame, baseFrame, new YoFixedFrameSpatialVector(angularPart, linearPart));
   }

   /**
    * Creates a new twist given the spatial vector holding data and initializes its reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param bodyFrame     what we are specifying the twist of.
    * @param baseFrame     with respect to what we are specifying the twist.
    * @param spatialVector the spatial vector to use for holding data.
    */
   public YoFixedFrameTwist(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, YoFixedFrameSpatialVector spatialVector)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.spatialVector = spatialVector;
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoFixedFrameTwist other)
   {
      FixedFrameTwistBasics.super.set(other);
   }

   /**
    * Transform this twist using the given transform. Effectively, the new twist
    * (T<sub>body</sub><sup>des, base</sup> = [&omega;<sub>new</sub>; &nu;<sub>new</sub>]) is
    * calculated as follows:
    *
    * <pre>
    * &omega;<sub>new</sub> = R &omega;
    * &nu;<sub>new</sub> = R &nu; + P &times; &omega;<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega; and &nu; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation parts of the given transform.
    * </ul>
    * </p>
    */
   @Override
   public void applyTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransform)
      {
         applyTransform((RigidBodyTransform) transform);
      }
      else
      {
         observerPosition.setToZero();
         observerPosition.applyTransform(transform);
         spatialVector.applyTransform(transform);
         addCrossToLinearPart(observerPosition, getAngularPart());
      }
   }

   /**
    * Transform this twist by the inverse of the given transform.
    * <p>
    * Effectively, the new twist (T<sub>body</sub><sup>des, base</sup> = [&omega;<sub>new</sub>;
    * &nu;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;<sub>new</sub> = R<sup>T</sup> &omega;
    * &nu;<sub>new</sub> = R<sup>T</sup> &nu; - (R<sup>T</sup> P) &times; &omega;<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega; and &nu; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation parts of the given transform.
    * </ul>
    * </p>
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransform)
      {
         applyInverseTransform((RigidBodyTransform) transform);
      }
      else
      {
         observerPosition.setToZero();
         observerPosition.applyInverseTransform(transform);
         spatialVector.applyInverseTransform(transform);
         addCrossToLinearPart(observerPosition, getAngularPart());
      }
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getBaseFrame()
   {
      return baseFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return spatialVector.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameVector3D getAngularPart()
   {
      return spatialVector.getAngularPart();
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameVector3D getLinearPart()
   {
      return spatialVector.getLinearPart();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(SpatialVectorReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof TwistReadOnly)
         return FixedFrameTwistBasics.super.equals((TwistReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this twist as follows:<br>
    * Twist of bodyFrame, with respect to baseFrame: [angular = (x, y, z), linear = (x, y, z)] -
    * expressedInFrame
    *
    * @return the {@code String} representing this twist.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getTwistString(this);
   }
}
