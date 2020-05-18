package us.ihmc.mecano.yoVariables.spatial;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialForceBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Implementation of {@link FixedFrameSpatialForceBasics} backed by {@link YoVariable}s.
 * <p>
 * When using a {@code SpatialForceVector}, it is important to note that the reference frame in
 * which it is expressed does not only refer to the coordinate system in which the angular and
 * linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the spatial force is measured.
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
public class YoFixedFrameSpatialForce implements FixedFrameSpatialForceBasics, GeometryObject<YoFixedFrameSpatialForce>
{
   /** This is where we store the internal data. */
   private final YoFixedFrameSpatialVector spatialVector;
   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D pointOfApplication = new Point3D();

   /**
    * Creates a new spatial force with its components set to zero and initializes its reference frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param expressedInFrame in which reference frame the spatial force is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameSpatialForce(String namePrefix, ReferenceFrame expressedInFrame, YoVariableRegistry registry)
   {
      this(new YoFixedFrameSpatialVector(namePrefix, expressedInFrame, registry));
   }

   /**
    * Creates a new spatial force with its components set to zero and initializes its reference frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param namePrefix       a unique name string to use as the prefix for child variable names.
    * @param nameSuffix       a string to use as the suffix for child variable names.
    * @param expressedInFrame in which reference frame the spatial force is expressed.
    * @param registry         the registry to register child variables to.
    */
   public YoFixedFrameSpatialForce(String namePrefix, String nameSuffix, ReferenceFrame expressedInFrame, YoVariableRegistry registry)
   {
      this(new YoFixedFrameSpatialVector(namePrefix, nameSuffix, expressedInFrame, registry));
   }

   /**
    * Creates a new spatial force given its angular and linear parts and initializes its reference
    * frame.
    * <p>
    * Note that the reference frame is final.
    * </p>
    *
    * @param angularPart the vector to use for the angular part.
    * @param linearPart  the vector to use for the linear part.
    */
   public YoFixedFrameSpatialForce(YoFrameVector3D angularPart, YoFrameVector3D linearPart)
   {
      this(new YoFixedFrameSpatialVector(angularPart, linearPart));
   }

   /**
    * Creates a new spatial acceleration given the spatial vector holding data and initializes its
    * reference frames.
    * <p>
    * Note that the reference frames are final.
    * </p>
    *
    * @param spatialVector the spatial vector to use for holding data.
    */
   public YoFixedFrameSpatialForce(YoFixedFrameSpatialVector spatialVector)
   {
      this.spatialVector = spatialVector;
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoFixedFrameSpatialForce other)
   {
      FixedFrameSpatialForceBasics.super.set(other);
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
    * Transform this spatial force using the given transform.
    * <p>
    * Effectively, the new spatial force F<sub>des</sub> = [&tau;<sub>new</sub>; f<sub>new</sub>] is
    * calculated as follows:
    *
    * <pre>
    * &tau;<sub>new</sub> = R &tau; + P &times f<sub>new</sub>
    * f<sub>new</sub> = R f
    * </pre>
    *
    * where:
    * <ul>
    * <li>&tau; and f are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the given transform.
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
         pointOfApplication.setToZero();
         pointOfApplication.applyTransform(transform); // p
         spatialVector.applyTransform(transform);
         addCrossToAngularPart(pointOfApplication, getLinearPart()); // p x R * f
      }
   }

   /**
    * Transform this spatial force by the inverse of the given transform.
    * <p>
    * Effectively, the new spatial force F<sub>des</sub> = [&tau;<sub>new</sub>; f<sub>new</sub>] is
    * calculated as follows:
    *
    * <pre>
    * &tau;<sub>new</sub> = R<sup>T</sup> &tau; - ( R<sup>T</sup> P ) &times f<sub>new</sub>
    * f<sub>new</sub> = R<sup>T</sup> f
    * </pre>
    *
    * where:
    * <ul>
    * <li>&tau; and f are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the given transform.
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
         pointOfApplication.setToZero();
         pointOfApplication.applyInverseTransform(transform); // p
         spatialVector.applyInverseTransform(transform);
         addCrossToAngularPart(pointOfApplication, getLinearPart()); // p x R * f
      }
   }

   /**
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon} and both vectors are expressed in the same reference frame.
    *
    * @param other   the other vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(YoFixedFrameSpatialForce other, double epsilon)
   {
      return FixedFrameSpatialForceBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same spatial force to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for spatial forces might evolve. In the meantime, the current assumption
    * is that two spatial forces are geometrically equal if both their 3D torque and 3D force are
    * independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other vector to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial forces represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *                                         respectively match the reference frames of {@code this}.
    */
   @Override
   public boolean geometricallyEquals(YoFixedFrameSpatialForce other, double epsilon)
   {
      return FixedFrameSpatialForceBasics.super.geometricallyEquals(other, epsilon);
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
      if (object instanceof SpatialForceReadOnly)
         return FixedFrameSpatialForceBasics.super.equals((SpatialVectorReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this spatial force vector as follows:<br>
    * Spatial Force: [angular = (x, y, z), linear = (x, y, z)] - expressedInFrame
    *
    * @return the {@code String} representing this spatial force vector.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getSpatialForceVectorString(this);
   }
}
