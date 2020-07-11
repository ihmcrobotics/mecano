package us.ihmc.mecano.yoVariables.spatial;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Implementation of {@link FixedFrameSpatialAccelerationBasics} backed by {@link YoVariable}s.
 * <p>
 * A spatial acceleration always describes the relative velocity of a body with respect to a base.
 * These two entities are referred to here by using two reference frames: a {@code bodyFrame} that
 * is considered to be rigidly attached to the body, and a {@code baseFrame} that is considered to
 * be rigidly attached to the base.
 * </p>
 * <p>
 * When using a {@code YoFixedFrameSpatialAcceleration}, it is important to note that the reference
 * frame in which it is expressed does not only refer to the coordinate system in which the angular
 * and linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the acceleration is measured. While the angular part remains the same as the point of
 * measurement changes, the linear part does depend on its location.
 * </p>
 * <p>
 * This framework for representing in an accurate and safe manner spatial accelerations is based on
 * the Ph.D. thesis of Vincent Duindam entitled <i>"Port-Based Modeling and Control for Efficient
 * Bipedal Walking Robots"</i>. Duindam's publications can be found
 * <a href="http://sites.google.com/site/vincentduindam/publications">here</a>. Several references
 * to this work are spread throughout the code.
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
public class YoFixedFrameSpatialAcceleration implements FixedFrameSpatialAccelerationBasics, GeometryObject<YoFixedFrameSpatialAcceleration>
{
   /**
    * Reference frame rigidly attached to the body that this spatial acceleration describes the motion
    * of.
    */
   private final ReferenceFrame bodyFrame;
   /**
    * Reference frame rigidly attached to the base that this spatial acceleration uses as reference for
    * quantifying the body's acceleration.
    */
   private final ReferenceFrame baseFrame;
   /** This is where we store the internal data. */
   private final YoFixedFrameSpatialVector spatialVector;
   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D observerPosition = new Point3D();

   /**
    * Creates a new spatial acceleration with its components set to zero and initializes its reference
    * frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param bodyFrame        what we are specifying the spatial acceleration of.
    * @param baseFrame        with respect to what we are specifying the spatial acceleration.
    * @param expressedInFrame in which reference frame the spatial acceleration is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameSpatialAcceleration(String namePrefix, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
                                          YoRegistry registry)
   {
      this(bodyFrame, baseFrame, new YoFixedFrameSpatialVector(namePrefix, expressedInFrame, registry));
   }

   /**
    * Creates a new spatial acceleration with its components set to zero and initializes its reference
    * frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param nameSuffix       a string to use as the suffix for child variable names.
    * @param bodyFrame        what we are specifying the spatial acceleration of.
    * @param baseFrame        with respect to what we are specifying the spatial acceleration.
    * @param expressedInFrame in which reference frame the spatial acceleration is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameSpatialAcceleration(String namePrefix, String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame baseFrame,
                                          ReferenceFrame expressedInFrame, YoRegistry registry)
   {
      this(bodyFrame, baseFrame, new YoFixedFrameSpatialVector(namePrefix, nameSuffix, expressedInFrame, registry));
   }

   /**
    * Creates a new spatial acceleration given its angular and linear parts and initializes its
    * reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param bodyFrame   what we are specifying the spatial acceleration of.
    * @param baseFrame   with respect to what we are specifying the spatial acceleration.
    * @param angularPart the vector to use for the angular part.
    * @param linearPart  the vector to use for the linear part.
    */
   public YoFixedFrameSpatialAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, YoFrameVector3D angularPart, YoFrameVector3D linearPart)
   {
      this(bodyFrame, baseFrame, new YoFixedFrameSpatialVector(angularPart, linearPart));
   }

   /**
    * Creates a new spatial acceleration given the spatial vector holding data and initializes its
    * reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param bodyFrame     what we are specifying the spatial acceleration of.
    * @param baseFrame     with respect to what we are specifying the spatial acceleration.
    * @param spatialVector the spatial vector to use for holding data.
    */
   public YoFixedFrameSpatialAcceleration(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, YoFixedFrameSpatialVector spatialVector)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.spatialVector = spatialVector;
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoFixedFrameSpatialAcceleration other)
   {
      FixedFrameSpatialAccelerationBasics.super.set(other);
   }

   /**
    * Transform this spatial acceleration using the given transform.
    * <p>
    * Effectively, the new spatial acceleration (A<sub>body</sub><sup>des, base</sup> =
    * [&omega;'<sub>new</sub>; &alpha;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;'<sub>new</sub> = R &omega;'
    * &alpha;<sub>new</sub>  = R &alpha; + P &times; &omega;'<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega;' and &alpha; are the current angular and linear parts of this spatial vector,
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
    * Transform this spatial acceleration by the inverse of the given transform.
    * <p>
    * Effectively, the new spatial acceleration (A<sub>body</sub><sup>des, base</sup> =
    * [&omega;'<sub>new</sub>; &alpha;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;'<sub>new</sub> = R<sup>T</sup> &omega;'
    * &alpha;<sub>new</sub>  = R<sup>T</sup> &alpha; - (R<sup>T</sup> P) &times; &omega;'<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega;' and &alpha; are the current angular and linear parts of this spatial vector,
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
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon} and both vectors have the same frames.
    *
    * @param other   the other motion vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(YoFixedFrameSpatialAcceleration other, double epsilon)
   {
      return FixedFrameSpatialAccelerationBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same spatial acceleration to an
    * {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for spatial accelerations might evolve. In the meantime, the current
    * assumption is that two spatial accelerations are geometrically equal if both their 3D angular and
    * 3D linear accelerations are independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other spatial acceleration to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial accelerations represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   @Override
   public boolean geometricallyEquals(YoFixedFrameSpatialAcceleration other, double epsilon)
   {
      return FixedFrameSpatialAccelerationBasics.super.geometricallyEquals(other, epsilon);
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
      if (object instanceof SpatialAccelerationReadOnly)
         return FixedFrameSpatialAccelerationBasics.super.equals((SpatialAccelerationReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this spatial acceleration vector as follows:<br>
    * Spatial acceleration of bodyFrame, with respect to baseFrame: [angular = (x, y, z), linear = (x,
    * y, z)] - expressedInFrame
    *
    * @return the {@code String} representing this spatial acceleration vector.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getSpatialAccelerationString(this);
   }
}
