package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Write and read interface for a spatial inertia matrix which reference frames can be changed.
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
public interface SpatialInertiaBasics extends FixedFrameSpatialInertiaBasics, FrameChangeable
{
   /**
    * Sets the frame attached to the body that this matrix describes the spatial inertia of.
    * <p>
    * This method does not modify anything but the body frame.
    * </p>
    *
    * @param bodyFrame the new body frame.
    */
   void setBodyFrame(ReferenceFrame bodyFrame);

   /**
    * Sets all the components of this spatial inertia to zero and updates its reference frames.
    *
    * @param bodyFrame the new body frame.
    * @param expressedInFrame the new reference frame in which this inertia is expressed.
    */
   default void setToZero(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setBodyFrame(bodyFrame);
      setReferenceFrame(expressedInFrame);
      setToZero();
   }

   /**
    * Sets all the components of this spatial inertia to {@link Double#NaN} and sets its reference
    * frames.
    *
    * @param bodyFrame the new body frame.
    * @param expressedInFrame the new reference frame in which this inertia is expressed.
    */
   default void setToNaN(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setBodyFrame(bodyFrame);
      setReferenceFrame(expressedInFrame);
      setToNaN();
   }

   /**
    * Sets this spatial inertia to {@code other}.
    * 
    * @param other the other spatial inertia to copy values and reference frames from. Not modified.
    */
   default void setIncludingFrame(SpatialInertiaReadOnly other)
   {
      setBodyFrame(other.getBodyFrame());
      setReferenceFrame(other.getReferenceFrame());
      getMomentOfInertia().set(other.getMomentOfInertia());
      setMass(other.getMass());
      getCenterOfMassOffset().set(other.getCenterOfMassOffset());
   }

   /**
    * Sets the moment of inertia to a diagonal matrix, sets the mass to the given value, and sets
    * the center of mass offset to zero.
    * <p>
    * Use this method if {@code expressedInFrame} has origin that coincides with the center of mass
    * and has its axes aligned with the principal directions of the inertia ellipsoid.
    * </p>
    * 
    * @param bodyFrame what we are specifying the inertia of.
    * @param expressedInFrame the new reference frame in which this spatial inertia is expressed.
    * @param Ixx the moment of inertia around the x-axis.
    * @param Iyy the moment of inertia around the y-axis.
    * @param Izz the moment of inertia around the z-axis.
    * @param mass the new mass value.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double Ixx, double Iyy, double Izz, double mass)
   {
      setBodyFrame(bodyFrame);
      setReferenceFrame(expressedInFrame);
      setMomentOfInertia(Ixx, Iyy, Izz);
      setMass(mass);
      getCenterOfMassOffset().setToZero();
   }

   /**
    * Sets the moment of inertia and mass to the given ones and sets the center of mass offset.
    * <p>
    * Use this method if {@code expressedInFrame} has origin that coincides with the center of mass.
    * </p>
    * 
    * @param bodyFrame what we are specifying the inertia of.
    * @param expressedInFrame the new reference frame in which this spatial inertia is expressed.
    * @param momentOfInertia the 3 by 3 moment of inertia matrix. Not modified.
    * @param mass the new mass value.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Matrix3DReadOnly momentOfInertia, double mass)
   {
      setBodyFrame(bodyFrame);
      setReferenceFrame(expressedInFrame);
      getMomentOfInertia().set(momentOfInertia);
      setMass(mass);
      getCenterOfMassOffset().setToZero();
   }

   /**
    * Sets all the components of this spatial inertia matrix.
    * 
    * @param bodyFrame what we are specifying the inertia of.
    * @param expressedInFrame the new reference frame in which this spatial inertia is expressed.
    * @param momentOfInertia the 3 by 3 moment of inertia matrix. Not modified.
    * @param mass the new mass value.
    * @param centerOfMassOffset the new offset of the center of mass with respect to
    *           {@code expressedInFrame}'s origin. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Matrix3DReadOnly momentOfInertia, double mass,
                                  Tuple3DReadOnly centerOfMassOffset)
   {
      setBodyFrame(bodyFrame);
      setReferenceFrame(expressedInFrame);
      getMomentOfInertia().set(momentOfInertia);
      setMass(mass);
      getCenterOfMassOffset().set(centerOfMassOffset);
   }

   /**
    * Transforms this spatial inertia using the given transform.
    * <p>
    * See the Word&trade; document located in the document folder of this project for more
    * information about the transformation rule for spatial inertia. Also see Duindam, <i>Port-Based
    * Modeling and Control for Efficient Bipedal Walking Robots</i>, page 40, equation (2.57) from
    * which the equations here were derived.
    * </p>
    * 
    * @throws UnsupportedOperationException if the given transform is not a
    *            {@code RigidBodyTransform}.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransform)
         applyTransform((RigidBodyTransform) transform);
      else
         throw new UnsupportedOperationException("The feature applyTransform is not supported for the transform of the type: "
               + transform.getClass().getSimpleName());
   }

   /**
    * Transforms this spatial inertia by the inverse of the given transform.
    * <p>
    * See the Word&trade; document located in the document folder of this project for more
    * information about the transformation rule for spatial inertia. Also see Duindam, <i>Port-Based
    * Modeling and Control for Efficient Bipedal Walking Robots</i>, page 40, equation (2.57) from
    * which the equations here were derived.
    * </p>
    * 
    * @throws UnsupportedOperationException if the given transform is not a
    *            {@code RigidBodyTransform}.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransform)
         applyInverseTransform((RigidBodyTransform) transform);
      else
         throw new UnsupportedOperationException("The feature applyInverseTransform is not supported for the transform of the type: "
               + transform.getClass().getSimpleName());
   }

   /**
    * Transforms this spatial inertia using the given transform.
    * <p>
    * See the Word&trade; document located in the document folder of this project for more
    * information about the transformation rule for spatial inertia. Also see Duindam, <i>Port-Based
    * Modeling and Control for Efficient Bipedal Walking Robots</i>, page 40, equation (2.57) from
    * which the equations here were derived.
    * </p>
    * 
    * @param transform the transform to use on this. Not modified.
    */
   default void applyTransform(RigidBodyTransform transform)
   {
      if (transform.hasRotation())
      {
         // Let's first apply the rotation onto the CoM and the mass moment of inertia:
         MecanoTools.transformSymmetricMatrix3D(transform.getRotationMatrix(), getMomentOfInertia());
         getCenterOfMassOffset().applyTransform(transform);
      }

      if (transform.hasTranslation())
      {
         // Now we can simply apply the translation on the CoM and mass moment of inertia:
         MecanoTools.translateMomentOfInertia(getMass(), getCenterOfMassOffset(), false, transform.getTranslationVector(), getMomentOfInertia());
         getCenterOfMassOffset().add(transform.getTranslationVector());
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
    * 
    * @param transform the transform to use on this. Not modified.
    */
   default void applyInverseTransform(RigidBodyTransform transform)
   {
      if (transform.hasTranslation())
      {
         // Now we can simply apply the translation on the CoM and mass moment of inertia:
         MecanoTools.translateMomentOfInertia(getMass(), getCenterOfMassOffset(), true, transform.getTranslationVector(), getMomentOfInertia());
         getCenterOfMassOffset().sub(transform.getTranslationVector());
      }

      if (transform.hasRotation())
      {
         // Let's first apply the rotation onto the CoM and the mass moment of inertia:
         MecanoTools.inverseTransformSymmetricMatrix3D(transform.getRotationMatrix(), getMomentOfInertia());
         getCenterOfMassOffset().applyInverseTransform(transform);
      }
   }
}
