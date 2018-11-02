package us.ihmc.mecano.spatial;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * A {@link SpatialInertia} is a 6 by 6 matrix that gathers both the angular and linear inertia
 * including the cross components. The form of the matrix depends on the frame in which it is
 * expressed. When the frame's origin coincides with the center of mass position and that its axes
 * are aligned with the principal directions of the inertia ellipsoid, the spatial inertia matrix
 * takes the following form:
 * 
 * <pre>
 *     / J<sub>x,x</sub> 0   0   0 0 0 \
 *     | 0   J<sub>y,y</sub> 0   0 0 0 |
 * I = | 0   0   J<sub>z,z</sub> 0 0 0 |
 *     | 0   0   0   m 0 0 |
 *     | 0   0   0   0 m 0 |
 *     \ 0   0   0   0 0 m /
 * </pre>
 * 
 * where <tt>m</tt> is the total mass, and <tt>J<sub>x,x</sub></tt>, <tt>J<sub>y,y</sub></tt>, and
 * <tt>J<sub>z,z</sub></tt> are the moments of inertia around the axes x, y, and z. <br>
 * When the frame in which the inertia is expressed is arbitrary, the spatial inertia takes the
 * following general form:
 * 
 * <pre>
 *     / J<sub>x,x</sub> J<sub>x,y</sub> J<sub>x,z</sub>   0 -mz  my \
 *     | J<sub>x,y</sub> J<sub>y,y</sub> J<sub>y,z</sub>  mz   0 -mx |
 * I = | J<sub>x,z</sub> J<sub>y,z</sub> J<sub>z,z</sub> -my  mx   0 |
 *     |   0  mz -my   m   0   0 |
 *     | -mz   0  mx   0   m   0 |
 *     \  my -mx   0   0   0   m /
 * </pre>
 * 
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public class SpatialInertia implements SpatialInertiaBasics, GeometryObject<SpatialInertia>
{
   /**
    * Reference frame rigidly attached to the body that this spatial inertia matrix describes the
    * inertia of.
    */
   private ReferenceFrame bodyFrame;
   /** The reference frame in which this inertia is expressed. */
   private ReferenceFrame expressedInFrame;

   /** The body's moment of inertia, or rotational inertia. */
   private final Matrix3D momentOfInertia = new Matrix3D();
   /** The total mass of the body. */
   private double mass = 0.0;
   /**
    * The offset of the body's center of mass position and the origin of {@link #expressedInFrame}.
    */
   private final FixedFrameVector3DBasics centerOfMassOffset = MecanoFactories.newFixedFrameVector3DBasics(this);

   /** Variable to store intermediate results for garbage-free operations. */
   private final Point3D translation = new Point3D();
   /** Variable to store intermediate results for garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new spatial inertia with its components set to zero and its reference frames set to
    * {@code null}.
    */
   public SpatialInertia()
   {
      bodyFrame = null;
      expressedInFrame = null;
   }

   /**
    * Creates a new spatial inertia matrix with its components set to zero and initializes its
    * reference frames.
    * 
    * @param bodyFrame what we are specifying the spatial inertia of.
    * @param expressedInFrame in which reference frame the spatial inertia is expressed.
    */
   public SpatialInertia(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setToZero(bodyFrame, expressedInFrame);
   }

   /**
    * Creates a new spatial inertia matrix and initializes its components and reference frames.
    * <p>
    * This method assumes that the body's center of mass is located at the origin of the given
    * {@code expressedInFrame}.
    * </p>
    *
    * @param bodyFrame what we are specifying the spatial inertia of.
    * @param expressedInFrame in which reference frame the spatial inertia is expressed.
    * @param momentOfInertia the moment of inertia of the body expressed in
    *           {@code expressedInFrame}. Not modified.
    * @param mass the mass of the rigid body.
    */
   public SpatialInertia(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Matrix3DReadOnly momentOfInertia, double mass)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, momentOfInertia, mass);
   }

   /**
    * Creates a new spatial inertia matrix and initializes its components and reference frames.
    * <p>
    * This method assumes that the body's center of mass is located at the origin of the given
    * {@code expressedInFrame} and that the axes of the given {@code expressedInFrame} are aligned
    * with the principal axes of the inertia ellipsoid.
    * </p>
    * 
    * @param bodyFrame what we are specifying the spatial inertia of.
    * @param expressedInFrame in which reference frame the spatial inertia is expressed.
    * @param Ixx the moment of inertia around the x-axis of the given {@code expressedInFrame}.
    * @param Iyy the moment of inertia around the y-axis of the given {@code expressedInFrame}.
    * @param Izz the moment of inertia around the z-axis of the given {@code expressedInFrame}.
    * @param mass the mass of the rigid body.
    */
   public SpatialInertia(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double Ixx, double Iyy, double Izz, double mass)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, Ixx, Iyy, Izz, mass);
   }

   /**
    * Creates a new spatial inertia matrix and initializes its components and reference frames.
    * 
    * @param bodyFrame what we are specifying the spatial inertia of.
    * @param expressedInFrame in which reference frame the spatial inertia is expressed.
    * @param momentOfInertia the moment of inertia of the body expressed in
    *           {@code expressedInFrame}. Not modified.
    * @param mass the mass of the rigid body.
    * @param centerOfMassOffset the offset of the body's center of mass position and the origin of
    *           the given {@code expressedInFrame}. Not modified.
    */
   public SpatialInertia(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Matrix3DReadOnly momentOfInertia, double mass,
                         Tuple3DReadOnly centerOfMassOffset)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, momentOfInertia, mass, centerOfMassOffset);
   }

   /**
    * Copy constructor.
    *
    * @param other the other spatial inertia to copy. Not modified.
    */
   public SpatialInertia(SpatialInertia other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(SpatialInertia other)
   {
      SpatialInertiaBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      this.expressedInFrame = expressedInFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {
      this.bodyFrame = bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setMass(double mass)
   {
      this.mass = mass;
   }

   /**
    * Transforms this matrix such that the result is the same physical spatial inertia but expressed
    * in a different frame.
    * <p>
    * Once this spatial inertia is transformed, the reference frame "expressed-in-frame" is updated
    * to {@code desiredFrame}. In the case, {@code this.expressedInFrame == desiredFrame}, this
    * method does nothing.
    * </p>
    * <p>
    * Note that in addition to transforming the coordinate system in which the components are
    * expressed, the point at which the inertia is estimated is also changed to be at the origin of
    * the given {@code desiredFrame}.
    * </p>
    * <p>
    * See the Word&trade; document located in the document folder of this project for more
    * information about the transformation rule for spatial inertia. Also see Duindam, <i>Port-Based
    * Modeling and Control for Efficient Bipedal Walking Robots</i>, page 40, equation (2.57) from
    * which the equations here were derived.
    * </p>
    *
    * @param desiredFrame the new reference frame in which this spatial inertia is to be expressed.
    */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == expressedInFrame)
         return;

      expressedInFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      expressedInFrame = desiredFrame;
   }

   /**
    * Transforms this spatial inertia using the given transform.
    * <p>
    * See the Word&trade; document located in the document folder of this project for more
    * information about the transformation rule for spatial inertia. Also see Duindam, <i>Port-Based
    * Modeling and Control for Efficient Bipedal Walking Robots</i>, page 40, equation (2.57) from
    * which the equations here were derived.
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
         translation.setToZero();
         translation.applyTransform(transform);

         // Let's first apply the rotation onto the CoM and the mass moment of inertia:
         momentOfInertia.applyTransform(transform);
         centerOfMassOffset.applyTransform(transform);

         // Now we can simply apply the translation on the CoM and mass moment of inertia:
         MecanoTools.translateMomentOfInertia(mass, centerOfMassOffset, false, translation, momentOfInertia);
         centerOfMassOffset.add(translation);
      }
   }

   /**
    * Transforms this spatial inertia by the inverse of the given transform.
    * <p>
    * See the Word&trade; document located in the document folder of this project for more
    * information about the transformation rule for spatial inertia. Also see Duindam, <i>Port-Based
    * Modeling and Control for Efficient Bipedal Walking Robots</i>, page 40, equation (2.57) from
    * which the equations here were derived.
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
         translation.setToZero();
         translation.applyInverseTransform(transform);
         
         // Let's first apply the rotation onto the CoM and the mass moment of inertia:
         momentOfInertia.applyInverseTransform(transform);
         centerOfMassOffset.applyInverseTransform(transform);
         
         // Now we can simply apply the translation on the CoM and mass moment of inertia:
         MecanoTools.translateMomentOfInertia(mass, centerOfMassOffset, true, translation, momentOfInertia);
         centerOfMassOffset.add(translation);
      }
   }

   /**
    * @return the frame in which this inertia is expressed.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return expressedInFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public Matrix3DBasics getMomentOfInertia()
   {
      return momentOfInertia;
   }

   /** {@inheritDoc} */
   @Override
   public double getMass()
   {
      return mass;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getCenterOfMassOffset()
   {
      return centerOfMassOffset;
   }

   /**
    * Tests if this spatial inertia and {@code other} are equal given the tolerance {@code epsilon}.
    * <p>
    * If the two spatial inertia matrices are expressed in different reference frames, this method
    * returns automatically {@code false}.
    * </p>
    * <p>
    * The test performs a component-wise comparison in turn on the mass, center of mass offset, and
    * moment of inertia of both spatial inertia matrices. If any of these comparisons fails, this
    * method returns {@code false}.
    * </p>
    * 
    * @param other the other spatial inertia matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two spatial inertia matrices are considered equal, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(SpatialInertia other, double epsilon)
   {
      return SpatialInertiaBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same spatial inertia to an
    * {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the
    * definition of "geometrically-equal" for spatial inertia might evolve. In the meantime, the
    * current assumption is that two spatial inertia matrices are geometrically equal if they are
    * epsilon equal, see {@link #epsilonEquals(SpatialInertiaReadOnly, double)}.
    * </p>
    *
    * @param other the other spatial inertia to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial inertia matrices represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *            respectively match the reference frames of {@code this}.
    */
   @Override
   public boolean geometricallyEquals(SpatialInertia other, double epsilon)
   {
      return SpatialInertiaBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(SpatialInertiaReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof SpatialInertiaReadOnly)
         return SpatialInertiaBasics.super.equals((SpatialInertiaReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this spatial inertia matrix as follows:<br>
    * 
    * <pre>
    * Spatial inertia of bodyFrame expressed in World:
    * / 0.253,  0.000,  0.000,  0.000,  0.554, -0.247 \
    * | 0.000,  0.994,  0.000, -0.554,  0.000,  0.387 |
    * | 0.000,  0.000,  0.084,  0.247, -0.387,  0.000 |
    * | 0.000, -0.554,  0.247,  0.773,  0.000,  0.000 |
    * | 0.554,  0.000, -0.387,  0.000,  0.773,  0.000 |
    * \-0.247,  0.387,  0.000,  0.000,  0.000,  0.773 /
    * </pre>
    * 
    *
    * @return the {@code String} representing this spatial inertia matrix.
    */
   @Override
   public String toString()
   {
      return MecanoIOTools.getSpatialInertiaString(this);
   }
}