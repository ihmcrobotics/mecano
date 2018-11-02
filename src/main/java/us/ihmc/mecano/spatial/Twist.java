package us.ihmc.mecano.spatial;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialMotionReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;

/**
 * A {@code Twist} is a vector composed of 6 components with an angular part and a linear part. The
 * angular part represents a 3D angular velocity and the linear part a 3D linear velocity.
 * <p>
 * A twist always describes the relative velocity of a body with respect to a base. These two
 * entities are referred to here by using two reference frames: a {@code bodyFrame} that is
 * considered to be rigidly attached to the body, and a {@code baseFrame} that is considered to be
 * rigidly attached to the base.
 * </p>
 * <p>
 * When using a {@code Twist}, it is important to note that the reference frame in which it is
 * expressed does not only refer to the coordinate system in which the angular and linear 3D vectors
 * are expressed. The origin of the reference frame is also used as the point where the velocity is
 * measured. While the angular part remains the same as the point of measurement changes, the linear
 * part does depend on its location. As an example for visualizing this effect, imagine yourself
 * standing on a platform quickly rotating: when standing at its center of rotation, you do not feel
 * any wind, but as you walk away from the center, the wind becomes stronger. This highlights that
 * in both situations the velocity of the platform is the same but the linear velocity depends on
 * where you are measuring it. See {@link #changeFrame(ReferenceFrame)} for more information.
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
public class Twist implements TwistBasics, GeometryObject<Twist>
{
   /**
    * Reference frame rigidly attached to the body that this twist describes the motion of.
    */
   private ReferenceFrame bodyFrame;
   /**
    * Reference frame rigidly attached to the base that this twist uses as reference for quantifying
    * the body's velocity.
    */
   private ReferenceFrame baseFrame;
   /** This is where we store the internal data. */
   private final SpatialVector spatialVector = new SpatialVector();

   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D observerPosition = new Point3D();
   /** Variable to store intermediate results for garbage-free operations. */
   private RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new twist with its components set to zero and its reference frames set to
    * {@code null}.
    */
   public Twist()
   {
      setToZero(null, null, null);
   }

   /**
    * Creates a new twist with its components set to zero and initializes its reference frames.
    *
    * @param bodyFrame what we are specifying the twist of.
    * @param baseFrame with respect to what we are specifying the twist.
    * @param expressedInFrame in which reference frame the twist is expressed.
    */
   public Twist(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      setToZero(bodyFrame, baseFrame, expressedInFrame);
   }

   /**
    * Creates a new twist and initializes its components and reference frames.
    *
    * @param bodyFrame what we are specifying the twist of.
    * @param baseFrame with respect to what we are specifying the twist.
    * @param expressedInFrame in which reference frame the twist is expressed.
    * @param angularPart the vector holding the values for the angular part. Not modified.
    * @param linearPart the vector holding the values for the linear part. Not modified.
    */
   public Twist(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setIncludingFrame(bodyFrame, baseFrame, expressedInFrame, angularPart, linearPart);
   }

   /**
    * Creates a new twist and initializes its components and reference frames.
    *
    * @param bodyFrame what we are specifying the twist of.
    * @param baseFrame with respect to what we are specifying the twist.
    * @param angularPart the vector holding the values for the angular part, it is expressed in the
    *           "expressed-in-frame" to use for this twist. Not modified.
    * @param linearPart the vector holding the values for the linear part, it is expressed in the
    *           "expressed-in-frame" to use for this twist. Not modified.
    */
   public Twist(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      setIncludingFrame(bodyFrame, baseFrame, angularPart, linearPart);
   }

   /**
    * Creates a new twist and initializes its components and reference frames.
    *
    * @param bodyFrame what we are specifying the twist of.
    * @param baseFrame with respect to what we are specifying the twist.
    * @param spatialVector the vector holding the values, it is expressed in the
    *           "expressed-in-frame" to use for this twist. Not modified.
    */
   public Twist(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, SpatialVectorReadOnly spatialVector)
   {
      setIncludingFrame(bodyFrame, baseFrame, spatialVector);
   }

   /**
    * Creates a new twist from the given reference frames and matrix.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame what we are specifying the twist of.
    * @param baseFrame with respect to what we are specifying the twist.
    * @param expressedInFrame in which reference frame the twist is expressed.
    * @param matrix the column vector containing the values for this vector's components. Not
    *           modified.
    */
   public Twist(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setIncludingFrame(bodyFrame, baseFrame, expressedInFrame, matrix);
   }

   /**
    * Creates a new twist from the given reference frames and array.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame what we are specifying the twist of.
    * @param baseFrame with respect to what we are specifying the twist.
    * @param expressedInFrame in which reference frame the twist is expressed.
    * @param array the array containing the new values for this vector's components. Not modified.
    */
   public Twist(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      setIncludingFrame(bodyFrame, baseFrame, expressedInFrame, array);
   }

   /**
    * Copy constructor.
    *
    * @param other the other spatial motion vector to copy. Not modified.
    */
   public Twist(SpatialMotionReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(Twist other)
   {
      TwistBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setBaseFrame(ReferenceFrame baseFrame)
   {
      this.baseFrame = baseFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {
      this.bodyFrame = bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      spatialVector.setReferenceFrame(expressedInFrame);
   }

   /**
    * Transforms this vector such that the result is the same physical twist but expressed in a
    * different frame.
    * <p>
    * Once this twist is transformed, the reference frame "expressed-in-frame" is updated to
    * {@code desiredFrame}. In the case, {@code this.expressedInFrame == desiredFrame}, this method
    * does nothing.
    * </p>
    * <p>
    * Note that in addition to transforming the angular and linear parts so their components are
    * expressed in {@code desiredFrame}, the linear part is also affected by the position of the new
    * frame.
    * </p>
    * <p>
    * Effectively, the new twist (T<sub>body</sub><sup>des, base</sup> = [&omega;<sub>new</sub>;
    * &nu;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;<sub>new</sub> = R &omega;
    * &nu;<sub>new</sub> = R &nu; + P &times; &omega;<sub>new</sub>
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega; and &nu; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the current "expressed-in-frame" with respect
    * to the desired frame.
    * </ul>
    * </p>
    * <p>
    * See Duindam, <i>Port-Based Modeling and Control for Efficient Bipedal Walking Robots</i>, page
    * 25, lemma 2.8 (c).
    * </p>
    *
    * @param desiredFrame the new reference frame in which this twist is to be expressed.
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
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon} and both vectors have the same frames.
    *
    * @param other the other motion vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Twist other, double epsilon)
   {
      return TwistBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same twist to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the
    * definition of "geometrically-equal" for twists might evolve. In the meantime, the current
    * assumption is that two twists are geometrically equal if both their 3D angular and 3D linear
    * velocities are independently geometrically equal, see
    * {@link Vector3DReadOnly#geometricallyEquals(Vector3DReadOnly, double)}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other twist to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two twists represent the same physical quantity, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *            respectively match the reference frames of {@code this}.
    */
   @Override
   public boolean geometricallyEquals(Twist other, double epsilon)
   {
      return TwistBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(SpatialVectorReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof TwistReadOnly)
         return TwistBasics.super.equals((TwistReadOnly) object);
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
