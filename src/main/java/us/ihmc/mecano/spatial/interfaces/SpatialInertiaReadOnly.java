package us.ihmc.mecano.spatial.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Read-only interface for a spatial inertia matrix.
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
public interface SpatialInertiaReadOnly extends ReferenceFrameHolder
{
   /**
    * The tolerance used when testing whether the center of mass offset is negligible or not.
    */
   static final double COM_OFFSET_ZERO_EPSILON = 1.0e-11;

   /**
    * Gets the read-only reference to the moment of inertia.
    * 
    * @return the moment of inertia matrix.
    */
   Matrix3DReadOnly getMomentOfInertia();

   /**
    * Gets the total mass value.
    * 
    * @return the mass.
    */
   double getMass();

   /**
    * Gets the read-only reference to the center of mass offset from the origin of the frame in which
    * this spatial inertia is expressed.
    * 
    * @return the center of mass offset.
    */
   FrameVector3DReadOnly getCenterOfMassOffset();

   /**
    * Gets the reference frame that is rigidly attached to the body this matrix is describing the
    * spatial inertia of.
    *
    * @return the body frame.
    */
   ReferenceFrame getBodyFrame();

   /**
    * Tests whether any component of this spatial inertia matrix is equal to {@link Double#NaN}.
    * 
    * @return {@code true} if at least one component of this inertia is {@link Double#NaN},
    *         {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return getMomentOfInertia().containsNaN() || Double.isNaN(getMass()) || getCenterOfMassOffset().containsNaN();
   }

   /**
    * Tests if the center of mass offset is negligible.
    * 
    * @return {@code true} if the center of mass offset can be ignored, {@code false} otherwise.
    */
   default boolean isCenterOfMassOffsetZero()
   {
      return getCenterOfMassOffset().lengthSquared() < COM_OFFSET_ZERO_EPSILON;
   }

   /**
    * Checks that all frames, i.e. the body frame and the "expressed in frame", are identical between
    * the two spatial inertia matrices.
    *
    * @param other the other object holding onto the reference frames to compare against the reference
    *           frames held by {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if the reference frames are not the same:
    *            {@code this.getBodyFrame() != other.getBodyFrame()} or
    *            {@code this.getReferenceFrame() != other.getReferenceFrame()}.
    */
   default void checkReferenceFrameMatch(SpatialInertiaReadOnly other) throws ReferenceFrameMismatchException
   {
      ReferenceFrame expressedInFrame = other.getReferenceFrame();
      ReferenceFrame bodyFrame = other.getBodyFrame();
      checkReferenceFrameMatch(bodyFrame, expressedInFrame);
   }

   /**
    * Checks that the reference frames used by {@code this} correspond to the given ones.
    *
    * @param bodyFrame the query for the body frame.
    * @param expressedInFrame the query for the "expressed in frame".
    * @throws ReferenceFrameMismatchException if the reference frames are not the same:
    *            {@code this.getBodyFrame() != bodyFrame} or
    *            {@code this.getReferenceFrame() != expressedInFrame}.
    */
   default void checkReferenceFrameMatch(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      ReferenceFrameHolder.super.checkReferenceFrameMatch(expressedInFrame);
      checkBodyFrameMatch(bodyFrame);
   }

   /**
    * Checks if the body frame held by {@code this} matches the query {@code bodyFrame}.
    *
    * @param bodyFrame the query to compare against the body frame held by {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if the two reference frames are not the same:
    *            {@code this.getBodyFrame() != bodyFrame}.
    */
   default void checkBodyFrameMatch(ReferenceFrame bodyFrame)
   {
      if (getBodyFrame() != bodyFrame)
         throw new ReferenceFrameMismatchException("bodyFrame mismatch: this.bodyFrame = " + getBodyFrame() + ", other bodyFrame = " + bodyFrame);
   }

   /**
    * Checks that the center of mass offset is negligible.
    * 
    * @throws RuntimeException if it is not negligible.
    */
   default void checkIfCenterOfMassOffsetIsZero()
   {
      if (!isCenterOfMassOffsetZero())
      {
         throw new RuntimeException("Center of mass does not coincide with " + getReferenceFrame() + "'s origin.");
      }
   }

   /**
    * Using Newton's second law of motion with a rigid-body to compute the net moment from its
    * acceleration.
    * <p>
    * This method requires this spatial inertia to be expressed at the center of mass. This simplifies
    * greatly the equations.
    * </p>
    * <p>
    * When the given {@code acceleration} or {@code twist} is {@code null}, it assumed to be equal to
    * zero.
    * </p>
    * 
    * @param acceleration the spatial acceleration of {@code this.bodyFrame} with respect to world and
    *           expressed in {@code this.bodyFrame}. Can be {@code null}. Not modified.
    * @param twist the twist of {@code this.bodyFrame} with respect to world and expressed in
    *           {@code this.bodyFrame}, Can be {@code null}. Not modified.
    * @param dynamicWrenchToPack the wrench used to store the net wrench of {@code this.bodyFrame}
    *           expressed in {@code bodyFrame}. Modified.
    * @throws RuntimeException if the center of mass offset is non-zero.
    * @throws ReferenceFrameMismatchException if {@code this.bodyFrame != this.expressedInFrame}.
    * @throws RuntimeException if either the given twist or acceleration base frame is not an inertial
    *            frame.
    * @throws ReferenceFrameMismatchException if either the given twist or acceleration body frame is
    *            not the same as {@code this.bodyFrame}.
    * @throws ReferenceFrameMismatchException if either the given twist or acceleration
    *            "expressed-in-frame" is not the same as {@code this.expressedInFrame}.
    */
   default void computeDynamicWrenchFast(SpatialAccelerationReadOnly acceleration, TwistReadOnly twist, WrenchBasics dynamicWrenchToPack)
   {
      checkIfCenterOfMassOffsetIsZero(); // otherwise this operation would be a lot less efficient

      checkBodyFrameMatch(getReferenceFrame());

      Vector3DReadOnly angularVelocity;
      Vector3DReadOnly linearVelocity;

      if (twist != null)
      {
         twist.checkBodyFrameMatch(getBodyFrame());
         twist.getBaseFrame().checkIsAStationaryFrame();
         twist.checkExpressedInFrameMatch(getReferenceFrame());
         angularVelocity = twist.getAngularPart();
         linearVelocity = twist.getLinearPart();
      }
      else
      {
         angularVelocity = null;
         linearVelocity = null;
      }

      Vector3DReadOnly angularAcceleration;
      Vector3DReadOnly linearAcceleration;

      if (acceleration != null)
      {
         acceleration.checkBodyFrameMatch(getBodyFrame());
         acceleration.getBaseFrame().checkIsAStationaryFrame();
         acceleration.checkExpressedInFrameMatch(getReferenceFrame());

         angularAcceleration = acceleration.getAngularPart();
         linearAcceleration = acceleration.getLinearPart();
      }
      else
      {
         angularAcceleration = null;
         linearAcceleration = null;
      }

      dynamicWrenchToPack.setToZero(getBodyFrame(), getReferenceFrame());

      Vector3DBasics dynamicMoment = dynamicWrenchToPack.getAngularPart();
      Vector3DBasics dynamicForce = dynamicWrenchToPack.getLinearPart();

      MecanoTools.computeDynamicMomentFast(getMomentOfInertia(), angularAcceleration, angularVelocity, dynamicMoment);
      MecanoTools.computeDynamicForceFast(getMass(), linearAcceleration, angularVelocity, linearVelocity, dynamicForce);
   }

   /**
    * Calculates the kinetic co-energy as introduced in the Ph.D. thesis of Vincent Duindam entitled
    * <i>"Port-Based Modeling and Control for Efficient Bipedal Walking Robots"</i>, page 38.
    * <p>
    * This method does not require this spatial inertia to be expressed in any particular frame.
    * </p>
    * 
    * @param twist the twist of {@code this.bodyFrame} with respect to world and expressed in
    *           {@code this.expressedInFrame}. Not modified.
    * @return the value of the kinetic co-energy.
    * @throws ReferenceFrameMismatchException if {@code twist.getBodyFrame() != this.getBodyFrame()},
    *            or if the given {@code twist} is not expressed in the same reference frame as
    *            {@code this}.
    * @throws RuntimeException if the base frame of the given {@code twist} is not an inertial frame.
    */
   default double computeKineticCoEnergy(TwistReadOnly twist)
   {
      twist.checkBodyFrameMatch(getBodyFrame());
      twist.getBaseFrame().checkIsAStationaryFrame();
      twist.checkExpressedInFrameMatch(getReferenceFrame());

      return MecanoTools.computeKineticCoEnergy(getMomentOfInertia(), getMass(), getCenterOfMassOffset(), twist.getAngularPart(),
            twist.getLinearPart());
   }

   /**
    * Packs this spatial inertia matrix into the given {@code DenseMatrix64F} as follows:
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
    * @param matrixToPack the matrix in which this spatial inertia is stored. Modified.
    */
   default void get(DenseMatrix64F matrixToPack)
   {
      get(0, 0, matrixToPack);
   }

   /**
    * Packs this spatial inertia matrix into the given {@code DenseMatrix64F} as follows:
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
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param startColumn the first column index to start writing in the dense-matrix.
    * @param matrixToPack the matrix in which this spatial inertia is stored. Modified.
    */
   default void get(int startRow, int startColumn, DenseMatrix64F matrixToPack)
   {
      // upper left block
      getMomentOfInertia().get(startRow, startColumn, matrixToPack);

      // upper right block
      MecanoTools.toTildeForm(getMass(), getCenterOfMassOffset(), false, startRow, startColumn + 3, matrixToPack);

      // lower left
      MecanoTools.toTildeForm(getMass(), getCenterOfMassOffset(), true, startRow + 3, startColumn, matrixToPack);

      // lower right block
      startRow += 3;
      startColumn += 3;

      for (int i = 0; i < 3; i++)
      {
         matrixToPack.set(startRow + i, startColumn + i, getMass());
         matrixToPack.set(startRow + i, startColumn + (i + 1) % 3, 0.0);
         matrixToPack.set(startRow + i, startColumn + (i + 2) % 3, 0.0);
      }
   }

   /**
    * Tests if this spatial inertia and {@code other} are equal given the tolerance {@code epsilon}.
    * <p>
    * If the two spatial inertia matrices have different reference frames, this method returns
    * automatically {@code false}.
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
   default boolean epsilonEquals(SpatialInertiaReadOnly other, double epsilon)
   {
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!EuclidCoreTools.epsilonEquals(getMass(), other.getMass(), epsilon))
         return false;
      if (!getCenterOfMassOffset().epsilonEquals(other.getCenterOfMassOffset(), epsilon))
         return false;
      if (!getMomentOfInertia().epsilonEquals(other.getMomentOfInertia(), epsilon))
         return false;
      return true;
   }

   /**
    * Tests if {@code this} and {@code other} represent the same spatial inertia to an {@code epsilon}.
    * <p>
    * It is likely that the implementation of this method will change in the future as the definition
    * of "geometrically-equal" for spatial inertia might evolve. In the meantime, the current
    * assumption is that two spatial inertia matrices are geometrically equal if they are epsilon
    * equal, see {@link #epsilonEquals(SpatialInertiaReadOnly, double)}.
    * </p>
    *
    * @param other the other spatial inertia to compare against this. Not modified.
    * @param epsilon the tolerance to use for the comparison.
    * @return {@code true} if the two spatial inertia matrices represent the same physical quantity,
    *         {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the reference frames of {@code other} do not
    *            respectively match the reference frames of {@code this}.
    */
   default boolean geometricallyEquals(SpatialInertiaReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return epsilonEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this spatial inertia matrix is exactly equal to {@code other}
    * and have the same reference frames.
    *
    * @param other the other spatial inertia to compare against this. Not modified.
    * @return {@code true} if the two spatial inertia matrices are exactly equal component-wise,
    *         {@code false} otherwise.
    */
   default boolean equals(SpatialInertiaReadOnly other)
   {
      if (other == null)
         return false;
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!getMomentOfInertia().equals(other.getMomentOfInertia()))
         return false;
      if (getMass() != other.getMass())
         return false;
      if (!getCenterOfMassOffset().equals(other.getCenterOfMassOffset()))
         return false;
      return true;
   }
}
