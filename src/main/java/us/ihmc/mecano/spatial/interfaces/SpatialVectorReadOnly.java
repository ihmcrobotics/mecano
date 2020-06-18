package us.ihmc.mecano.spatial.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

/**
 * Read-only interface for a spatial vector.
 * <p>
 * A spatial vector is a vector composed of 6 components with an angular part and a linear part.
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
public interface SpatialVectorReadOnly extends ReferenceFrameHolder
{
   /** A verbose way to refer to the size of a spatial vector. */
   static final int SIZE = 6;

   /**
    * Gets the read-only reference to the angular part of this vector.
    * <p>
    * Note that the frame of the angular part is always equal to this vector reference frame.
    * </p>
    *
    * @return the angular part as read-only.
    */
   FrameVector3DReadOnly getAngularPart();

   /**
    * Gets the read-only reference to the linear part of this vector.
    * <p>
    * Note that the frame of the linear part is always equal to this vector reference frame.
    * </p>
    *
    * @return the linear part as read-only.
    */
   FrameVector3DReadOnly getLinearPart();

   /**
    * Gets the reference frame in which this vector is currently expressed.
    *
    * @return the reference frame associated with {@code this}.
    */
   @Override
   ReferenceFrame getReferenceFrame();

   /**
    * Tests if this vector contains a {@link Double#NaN}.
    *
    * @return {@code true} if this vector contains a {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return getAngularPart().containsNaN() || getLinearPart().containsNaN();
   }

   /**
    * Gets the x-component of this vector's angular part.
    *
    * @return the x-component of this vector's angular part.
    */
   default double getAngularPartX()
   {
      return getAngularPart().getX();
   }

   /**
    * Gets the y-component of this vector's angular part.
    *
    * @return the y-component of this vector's angular part.
    */
   default double getAngularPartY()
   {
      return getAngularPart().getY();
   }

   /**
    * Gets the z-component of this vector's angular part.
    *
    * @return the z-component of this vector's angular part.
    */
   default double getAngularPartZ()
   {
      return getAngularPart().getZ();
   }

   /**
    * Gets the x-component of this vector's linear part.
    *
    * @return the x-component of this vector's linear part.
    */
   default double getLinearPartX()
   {
      return getLinearPart().getX();
   }

   /**
    * Gets the y-component of this vector's linear part.
    *
    * @return the y-component of this vector's linear part.
    */
   default double getLinearPartY()
   {
      return getLinearPart().getY();
   }

   /**
    * Gets the z-component of this vector's linear part.
    *
    * @return the z-component of this vector's linear part.
    */
   default double getLinearPartZ()
   {
      return getLinearPart().getZ();
   }

   /**
    * Selects a component of this spatial vector based on {@code index} and returns its value.
    * <ul>
    * <li>For an {@code index} &in; [0, 2], the corresponding components are {@code x}, {@code y}, and
    * {@code z} of this vector's angular part.
    * <li>For an {@code index} &in; [3, 5], the corresponding components are {@code x}, {@code y}, and
    * {@code z} of this vector's linear part.
    * </ul>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 5].
    */
   default double getElement(int index)
   {
      switch (index)
      {
         case 0:
            return getAngularPartX();
         case 1:
            return getAngularPartY();
         case 2:
            return getAngularPartZ();
         case 3:
            return getLinearPartX();
         case 4:
            return getLinearPartY();
         case 5:
            return getLinearPartZ();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Packs the components of this vector in an array starting from its first index.
    * <p>
    * The components are packed in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param arrayToPack the array in which this vector is stored. Modified.
    */
   default void get(double[] arrayToPack)
   {
      get(0, arrayToPack);
   }

   /**
    * Packs the components of this vector in an array starting from {@code startIndex}.
    * <p>
    * The components are packed in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startIndex  the index in the array where the first component is stored.
    * @param arrayToPack the array in which this vector is stored. Modified.
    */
   default void get(int startIndex, double[] arrayToPack)
   {
      getAngularPart().get(startIndex, arrayToPack);
      getLinearPart().get(startIndex + 3, arrayToPack);
   }

   /**
    * Packs the components of this vector in an array starting from its first index.
    * <p>
    * The components are packed in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param arrayToPack the array in which this vector is stored. Modified.
    */
   default void get(float[] arrayToPack)
   {
      get(0, arrayToPack);
   }

   /**
    * Packs the components of this vector in an array starting from {@code startIndex}.
    * <p>
    * The components are packed in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startIndex  the index in the array where the first component is stored.
    * @param arrayToPack the array in which this vector is stored. Modified.
    */
   default void get(int startIndex, float[] arrayToPack)
   {
      getAngularPart().get(startIndex, arrayToPack);
      getLinearPart().get(startIndex + 3, arrayToPack);
   }

   /**
    * Packs the components of this vector in a column vector starting from its first row index.
    * <p>
    * The components are packed in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param matrixToPack the column vector in which this vector is stored. Modified.
    */
   default void get(DMatrix matrixToPack)
   {
      get(0, 0, matrixToPack);
   }

   /**
    * Packs the components of this vector in a column vector starting from {@code startRow}.
    * <p>
    * The components are packed in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startRow     the first row index to start writing in the dense-matrix.
    * @param matrixToPack the column vector in which this vector is stored. Modified.
    */
   default void get(int startRow, DMatrix matrixToPack)
   {
      get(startRow, 0, matrixToPack);
   }

   /**
    * Packs the components of this vector in a column vector starting from {@code startRow} at the
    * column index {@code column}.
    * <p>
    * The components are packed in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startRow     the first row index to start writing in the dense-matrix.
    * @param column       the column index to write in the dense-matrix.
    * @param matrixToPack the column vector in which this vector is stored. Modified.
    */
   default void get(int startRow, int column, DMatrix matrixToPack)
   {
      getAngularPart().get(startRow, column, matrixToPack);
      getLinearPart().get(startRow + 3, column, matrixToPack);
   }

   /**
    * Calculates and returns the value of the dot product of this vector with {@code other}.
    * <p>
    * For instance, the dot product of two vectors p and q is defined as: <br>
    * p &sdot; q = &sum;<sub>i=1:6</sub>(p<sub>i</sub> &times; q<sub>i</sub>)
    * </p>
    *
    * @param other the other vector used for the dot product. Not modified.
    * @return the value of the dot product.
    */
   default double dot(SpatialVectorReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return getAngularPart().dot(other.getAngularPart()) + getLinearPart().dot(other.getLinearPart());
   }

   /**
    * Calculates and returns the magnitude of this vector.
    * <p>
    * length = &radic;(angular<sub>x</sub><sup>2</sup> + angular<sub>y</sub><sup>2</sup> +
    * angular<sub>z</sub><sup>2</sup> + linear<sub>x</sub><sup>2</sup> + linear<sub>y</sub><sup>2</sup>
    * + linear<sub>z</sub><sup>2</sup>)
    * </p>
    *
    * @return the magnitude of this vector.
    */
   default double length()
   {
      return Math.sqrt(lengthSquared());
   }

   /**
    * Calculates and returns the square of the magnitude of this vector.
    * <p>
    * length<sup>2</sup> = angular<sub>x</sub><sup>2</sup> + angular<sub>y</sub><sup>2</sup> +
    * angular<sub>z</sub><sup>2</sup> + linear<sub>x</sub><sup>2</sup> + linear<sub>y</sub><sup>2</sup>
    * + linear<sub>z</sub><sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #length()} when calculation speed matters and
    * knowledge of the actual magnitude does not, i.e. when comparing several vectors by theirs
    * magnitude.
    * </p>
    *
    * @return the square of the magnitude of this vector.
    */
   default double lengthSquared()
   {
      return getAngularPart().lengthSquared() + getLinearPart().lengthSquared();
   }

   /**
    * Tests on a per component basis if this vector is equal to the given {@code other} to an
    * {@code epsilon} and both vectors are expressed in the same reference frame.
    *
    * @param other   the other vector to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two vectors are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(SpatialVectorReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!getAngularPart().epsilonEquals(other.getAngularPart(), epsilon))
         return false;
      if (!getLinearPart().epsilonEquals(other.getLinearPart(), epsilon))
         return false;
      return true;
   }

   /**
    * Tests on a per component basis, if this vector is exactly equal to {@code other} and expressed in
    * the same reference frame.
    *
    * @param other the other vector to compare against this. Not modified.
    * @return {@code true} if the two vectors are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(SpatialVectorReadOnly other)
   {
      if (other == null)
         return false;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!getAngularPart().equals(other.getAngularPart()))
         return false;
      if (!getLinearPart().equals(other.getLinearPart()))
         return false;
      return true;
   }
}
