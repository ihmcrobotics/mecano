package us.ihmc.mecano.spatial;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;

/**
 * A {@code SpatialForceVector} is a vector composed of 6 components with an angular part and a
 * linear part. The angular part represents a 3D moment and the linear part a 3D force.
 * <p>
 * When using a {@code SpatialForceVector}, it is important to note that the reference frame in
 * which it is expressed does not only refer to the coordinate system in which the angular and
 * linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the spatial force is measured. Let's consider two reference frames A and B which axes are
 * parallel but have different origins, changing the frame of a spatial force vector from A to B
 * will not affect the linear part, i.e. the 3D force, but will still affect the value of the
 * angular part, i.e. the 3D moment. See {@link #changeFrame(ReferenceFrame)} for more information.
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
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public class SpatialForce implements SpatialForceBasics, GeometryObject<SpatialForce>
{
   /** This is where we store the internal data. */
   private final SpatialVector spatialVector = new SpatialVector();
   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D pointOfApplication = new Point3D();
   /** Variable to store intermediate results for garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new spatial force vector with its components set to zero and its reference frame set to
    * {@code ReferenceFrame.getWorldFrame()}.
    */
   public SpatialForce()
   {
   }

   /**
    * Creates a new spatial force vector with its components set to zero and initializes its reference
    * frame.
    *
    * @param expressedInFrame the initial frame in which this vector is expressed.
    */
   public SpatialForce(ReferenceFrame expressedInFrame)
   {
      setToZero(expressedInFrame);
   }

   /**
    * Creates a new spatial force vector and initializes its components and the reference frame it is
    * expressed in.
    *
    * @param expressedInFrame the initial frame in which this vector is expressed.
    * @param angularPart      the vector holding the values for the angular part. Not modified.
    * @param linearPart       the vector holding the values for the linear part. Not modified.
    */
   public SpatialForce(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setIncludingFrame(expressedInFrame, angularPart, linearPart);
   }

   /**
    * Creates a new spatial force vector from the given reference frame and matrix.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param matrix           the column vector containing the values for this vector's components. Not
    *                         modified.
    */
   public SpatialForce(ReferenceFrame expressedInFrame, DMatrix matrix)
   {
      setIncludingFrame(expressedInFrame, matrix);
   }

   /**
    * Creates a new spatial force vector from the given reference frame and array.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param array            the array containing the new values for this vector's components. Not
    *                         modified.
    */
   public SpatialForce(ReferenceFrame expressedInFrame, double[] array)
   {
      setIncludingFrame(expressedInFrame, array);
   }

   /**
    * Copy constructor.
    *
    * @param other the other spatial force vector to copy. Not modified.
    */
   public SpatialForce(SpatialForceReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(SpatialForce other)
   {
      SpatialForceBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      spatialVector.setReferenceFrame(expressedInFrame);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return spatialVector.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getAngularPart()
   {
      return spatialVector.getAngularPart();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getLinearPart()
   {
      return spatialVector.getLinearPart();
   }

   /**
    * Transforms this vector such that the result is the same physical spatial force but expressed in a
    * different frame.
    * <p>
    * Once this spatial force is transformed, the reference frame "expressed-in-frame" is updated to
    * {@code desiredFrame}. In the case, {@code this.expressedInFrame == desiredFrame}, this method
    * does nothing.
    * </p>
    * <p>
    * Note that in addition to transforming the angular and linear parts so their components are
    * expressed in {@code desiredFrame}, the moment is also affected by the position of the new frame.
    * </p>
    * <p>
    * Effectively, the new spatial force F<sub>des</sub> = [&tau;<sub>new</sub>; f<sub>new</sub>] is
    * calculated as follows:
    *
    * <pre>
    * &tau;<sub>new</sub> = &tau; + P &times f<sub>new</sub>
    * f<sub>new</sub> = R f
    * </pre>
    *
    * where:
    * <ul>
    * <li>&tau; and f are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the current "expressed-in-frame" with respect to
    * the desired frame.
    * </ul>
    * </p>
    * <p>
    * See Duindam, <i>Port-Based Modeling and Control for Efficient Bipedal Walking Robots</i>, page
    * 36, equation 2.47.
    * </p>
    *
    * @param desiredFrame the new reference frame in which this spatial force is to be expressed.
    */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // trivial case:
      if (getReferenceFrame() == desiredFrame)
      {
         return;
      }

      getReferenceFrame().getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      setReferenceFrame(desiredFrame);
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
   public boolean epsilonEquals(SpatialForce other, double epsilon)
   {
      return SpatialForceBasics.super.epsilonEquals(other, epsilon);
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
   public boolean geometricallyEquals(SpatialForce other, double epsilon)
   {
      return SpatialForceBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@code this.equals((SpatialForceReadOnly) object)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof SpatialForceReadOnly)
         return SpatialForceBasics.super.equals((SpatialForceReadOnly) object);
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
