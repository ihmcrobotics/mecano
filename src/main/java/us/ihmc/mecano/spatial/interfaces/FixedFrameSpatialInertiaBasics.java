package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;

/**
 * Write and read interface for a spatial inertia matrix which reference frames can not be changed.
 * <p>
 * A spatial inertial matrix is a 6 by 6 matrix that gathers both the angular and linear inertia
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
 * </p>
 * 
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface FixedFrameSpatialInertiaBasics extends SpatialInertiaReadOnly, Clearable
{
   /**
    * Gets the reference to the moment of inertia.
    */
   @Override
   Matrix3DBasics getMomentOfInertia();

   /**
    * Sets the mass to use with this spatial inertia matrix.
    * 
    * @param mass the new mass value.
    */
   void setMass(double mass);

   /**
    * Gets the reference to the center of mass offset from the origin of the frame in which this
    * spatial inertia is expressed.
    */
   @Override
   FixedFrameVector3DBasics getCenterOfMassOffset();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return SpatialInertiaReadOnly.super.containsNaN();
   }

   /**
    * Sets all the components of this spatial inertia to zero.
    */
   @Override
   default void setToZero()
   {
      getMomentOfInertia().setToZero();
      setMass(0.0);
      getCenterOfMassOffset().setToZero();
   }

   /**
    * Sets all the components of this spatial inertia to {@link Double#NaN}.
    */
   @Override
   default void setToNaN()
   {
      getMomentOfInertia().setToNaN();
      setMass(Double.NaN);
      getCenterOfMassOffset().setToNaN();
   }

   /**
    * Sets the moment of inertia to a diagonal matrix.
    * 
    * @param Ixx the moment of inertia around the x-axis.
    * @param Iyy the moment of inertia around the y-axis.
    * @param Izz the moment of inertia around the z-axis.
    */
   default void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      getMomentOfInertia().set(Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz);
   }

   /**
    * Sets this spatial inertia to {@code other}.
    * 
    * @param other the other spatial inertia to copy values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} does not have the same frames as
    *            {@code this}.
    */
   default void set(SpatialInertiaReadOnly other)
   {
      checkReferenceFrameMatch(other);
      getMomentOfInertia().set(other.getMomentOfInertia());
      setMass(other.getMass());
      getCenterOfMassOffset().set(other.getCenterOfMassOffset());
   }

   /**
    * Adds the other spatial inertia to this.
    * 
    * @param other the other inertia to add. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void add(SpatialInertiaReadOnly other)
   {
      other.checkReferenceFrameMatch(getReferenceFrame());
      getMomentOfInertia().add(other.getMomentOfInertia());
      getCenterOfMassOffset().scale(getMass());
      getCenterOfMassOffset().scaleAdd(other.getMass(), other.getCenterOfMassOffset(), getCenterOfMassOffset());
      setMass(getMass() + other.getMass());
      if (Math.abs(getMass()) >= 1.0e-7)
         getCenterOfMassOffset().scale(1.0 / getMass());
   }
}
