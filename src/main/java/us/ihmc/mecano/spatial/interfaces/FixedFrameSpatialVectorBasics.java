package us.ihmc.mecano.spatial.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Write and read interface for a spatial vector that is expressed in an unchangeable reference
 * frame.
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
public interface FixedFrameSpatialVectorBasics extends SpatialVectorReadOnly, Clearable, Transformable
{
   /**
    * Gets the reference to the angular part of this vector.
    * <p>
    * Note that the frame of the angular part is always equal to this vector reference frame.
    * </p>
    *
    * @return the angular part.
    */
   @Override
   FixedFrameVector3DBasics getAngularPart();

   /**
    * Gets the reference to the linear part of this vector.
    * <p>
    * Note that the frame of the linear part is always equal to this vector reference frame.
    * </p>
    *
    * @return the linear part.
    */
   @Override
   FixedFrameVector3DBasics getLinearPart();

   /**
    * Sets all the components of this vector to zero.
    */
   @Override
   default void setToZero()
   {
      getAngularPart().setToZero();
      getLinearPart().setToZero();
   }

   /**
    * Sets all the components of this vector to {@link Double#NaN}.
    */
   @Override
   default void setToNaN()
   {
      getAngularPart().setToNaN();
      getLinearPart().setToNaN();
   }

   /**
    * Tests if this vector contains a {@link Double#NaN}.
    *
    * @return {@code true} if this vector contains a {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return SpatialVectorReadOnly.super.containsNaN();
   }

   /**
    * Sets the value for the x-component of this vector's angular part.
    *
    * @param x the new value.
    */
   default void setAngularPartX(double x)
   {
      getAngularPart().setX(x);
   }

   /**
    * Sets the value for the y-component of this vector's angular part.
    *
    * @param y the new value.
    */
   default void setAngularPartY(double y)
   {
      getAngularPart().setY(y);
   }

   /**
    * Sets the value for the z-component of this vector's angular part.
    *
    * @param z the new value.
    */
   default void setAngularPartZ(double z)
   {
      getAngularPart().setZ(z);
   }

   /**
    * Sets the value for the x-component of this vector's linear part.
    *
    * @param x the new value.
    */
   default void setLinearPartX(double x)
   {
      getLinearPart().setX(x);
   }

   /**
    * Sets the value for the y-component of this vector's linear part.
    *
    * @param y the new value.
    */
   default void setLinearPartY(double y)
   {
      getLinearPart().setY(y);
   }

   /**
    * Sets the value for the z-component of this vector's linear part.
    *
    * @param z the new value.
    */
   default void setLinearPartZ(double z)
   {
      getLinearPart().setZ(z);
   }

   /**
    * Selects a component of this spatial vector based on {@code index} and sets it to {@code value}.
    * <ul>
    * <li>For an {@code index} &in; [0, 2], the corresponding components are {@code x}, {@code y}, and
    * {@code z} of this vector's angular part.
    * <li>For an {@code index} &in; [3, 5], the corresponding components are {@code x}, {@code y}, and
    * {@code z} of this vector's linear part.
    * </ul>
    *
    * @param index the index of the component to set.
    * @param value the new value of the selected component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 5].
    */
   default void setElement(int index, double value)
   {
      switch (index)
      {
         case 0:
            setAngularPartX(value);
            break;
         case 1:
            setAngularPartY(value);
            break;
         case 2:
            setAngularPartZ(value);
            break;
         case 3:
            setLinearPartX(value);
            break;
         case 4:
            setLinearPartY(value);
            break;
         case 5:
            setLinearPartZ(value);
            break;
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Sets this vector's components from the given array {@code array}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param array the array containing the new values for this vector's components. Not modified.
    */
   default void set(double[] array)
   {
      set(0, array);
   }

   /**
    * Sets this vector's components from the given array {@code array}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startIndex the first index to start reading from in the array.
    * @param array      the array containing the new values for this vector's components. Not modified.
    */
   default void set(int startIndex, double[] array)
   {
      getAngularPart().set(startIndex, array);
      getLinearPart().set(startIndex + 3, array);
   }

   /**
    * Sets this vector's components from the given array {@code array}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param array the array containing the new values for this vector's components. Not modified.
    */
   default void set(float[] array)
   {
      set(0, array);
   }

   /**
    * Sets this vector's components from the given array {@code array}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startIndex the first index to start reading from in the array.
    * @param array      the array containing the new values for this vector's components. Not modified.
    */
   default void set(int startIndex, float[] array)
   {
      getAngularPart().set(startIndex, array);
      getLinearPart().set(startIndex + 3, array);
   }

   /**
    * Sets this vector's components from the given column vector starting to read from its first row
    * index.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param matrix the column vector containing the new values for this vector's components. Not
    *               modified.
    */
   default void set(DenseMatrix64F matrix)
   {
      set(0, matrix);
   }

   /**
    * Sets this vector's components from the given column vector starting to read from
    * {@code startRow}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix   the column vector containing the new values for this vector's components. Not
    *                 modified.
    */
   default void set(int startRow, DenseMatrix64F matrix)
   {
      set(startRow, 0, matrix);
   }

   /**
    * Sets this vector's components from the given column vector starting to read from {@code startRow}
    * at the column index {@code column}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column   the column index to read in the dense-matrix.
    * @param matrix   the column vector containing the new values for this vector's components. Not
    *                 modified.
    */
   default void set(int startRow, int column, DenseMatrix64F matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 6, column + 1, matrix);
      getAngularPart().set(matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column));
      getLinearPart().set(matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column));
   }

   /**
    * Sets this vector given an angular part and linear part.
    *
    * @param angularPart the vector holding the new values for the angular part. Not modified.
    * @param linearPart  the vector holding the new values for the linear part. Not modified.
    */
   default void set(Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      getAngularPart().set(angularPart);
      getLinearPart().set(linearPart);
   }

   /**
    * Sets this vector to {@code other}.
    *
    * @param other the other vector to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(SpatialVectorReadOnly other)
   {
      set(other.getReferenceFrame(), other.getAngularPart(), other.getLinearPart());
   }

   /**
    * Sets this vector to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(SpatialVectorReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other vector to copy. Not modified.
    */
   default void setMatchingFrame(SpatialVectorReadOnly other)
   {
      getAngularPart().set((Vector3DReadOnly) other.getAngularPart());
      getLinearPart().set((Vector3DReadOnly) other.getLinearPart());
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this vector given an angular part and linear part.
    * <p>
    * If the arguments are expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(SpatialVectorReadOnly)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * to {@code angularPart} and {@code linearPart} once transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param angularPart the vector holding the new values for the angular part. Not modified.
    * @param linearPart  the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setMatchingFrame(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      getAngularPart().set((Vector3DReadOnly) angularPart);
      getLinearPart().set((Vector3DReadOnly) linearPart);
      angularPart.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this vector given an angular part and linear part.
    *
    * @param angularPart the vector holding the new values for the angular part. Not modified.
    * @param linearPart  the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      set(angularPart.getReferenceFrame(), angularPart, linearPart);
   }

   /**
    * Sets this vector given an angular part and linear part.
    *
    * @param expressedInFrame the reference frame in which the vectors are expressed.
    * @param angularPart      the vector holding the new values for the angular part. Not modified.
    * @param linearPart       the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if {@code expressedInFrame} is not equal to the frame in
    *                                         which this spatial vector is currently expressed.
    */
   default void set(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      checkReferenceFrameMatch(expressedInFrame);
      set(angularPart, linearPart);
   }

   /**
    * Scales the components of this vector by the given {@code scalar}.
    * <p>
    * this = scalar * this
    * </p>
    *
    * @param scalar the scale factor to use.
    */
   default void scale(double scalar)
   {
      getAngularPart().scale(scalar);
      getLinearPart().scale(scalar);
   }

   /**
    * Changes the sign of each component of this vector.
    */
   default void negate()
   {
      getAngularPart().negate();
      getLinearPart().negate();
   }

   /**
    * Normalizes this vector such that its magnitude is equal to 1 after calling this method and its
    * direction remains unchanged.
    */
   default void normalize()
   {
      scale(1.0 / length());
   }

   /**
    * Adds the given vector to this vector performing a per-component addition.
    * <p>
    * {@code this += other}
    * </p>
    *
    * @param other the other vector to add to this vector. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void add(SpatialVectorReadOnly other)
   {
      checkReferenceFrameMatch(other);
      add((Vector3DReadOnly) other.getAngularPart(), (Vector3DReadOnly) other.getLinearPart());
   }

   /**
    * Adds to this vector's components the given column vector starting to read from its first row
    * index.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param matrix the column vector containing the values to add to this vector's components. Not
    *               modified.
    */
   default void add(DenseMatrix64F matrix)
   {
      add(0, matrix);
   }

   /**
    * Adds to this vector's components the given column vector starting to read from {@code startRow}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix   the column vector containing the values to add to this vector's components. Not
    *                 modified.
    */
   default void add(int startRow, DenseMatrix64F matrix)
   {
      add(startRow, 0, matrix);
   }

   /**
    * Adds to this vector's components the given column vector starting to read from {@code startRow}
    * at the column index {@code column}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column   the column index to read in the dense-matrix.
    * @param matrix   the column vector containing the values to add to this vector's components. Not
    *                 modified.
    */
   default void add(int startRow, int column, DenseMatrix64F matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 6, column + 1, matrix);
      getAngularPart().add(matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column));
      getLinearPart().add(matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column));
   }

   /**
    * Adds the given vectors to this vector angular and linear parts.
    * <p>
    * {@code this.angularPart += angular}<br>
    * {@code this.linearPart += linear}
    * </p>
    *
    * @param angular the vector to add to this vector's angular part. Not modified.
    * @param linear  the vector to add to this vector's linear part. Not modified.
    */
   default void add(Vector3DReadOnly angular, Vector3DReadOnly linear)
   {
      getAngularPart().add(angular);
      getLinearPart().add(linear);
   }

   /**
    * Adds the given vectors to this vector angular and linear parts.
    * <p>
    * {@code this.angularPart += angular}<br>
    * {@code this.linearPart += linear}
    * </p>
    *
    * @param angular the vector to add to this vector's angular part. Not modified.
    * @param linear  the vector to add to this vector's linear part. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments are not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void add(FrameVector3DReadOnly angular, FrameVector3DReadOnly linear)
   {
      getAngularPart().add(angular);
      getLinearPart().add(linear);
   }

   /**
    * Subtracts the given vector to this vector performing a per-component subtraction.
    * <p>
    * {@code this -= other}
    * </p>
    *
    * @param other the other vector to subtract to this vector. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void sub(SpatialVectorReadOnly other)
   {
      checkReferenceFrameMatch(other);
      sub((Vector3DReadOnly) other.getAngularPart(), (Vector3DReadOnly) other.getLinearPart());
   }

   /**
    * Subtracts from this vector's components the given column vector starting to read from its first
    * row index.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param matrix the column vector containing the values to subtract from this vector's components.
    *               Not modified.
    */
   default void sub(DenseMatrix64F matrix)
   {
      sub(0, matrix);
   }

   /**
    * Subtracts from this vector's components the given column vector starting to read from
    * {@code startRow}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix   the column vector containing the values to subtract from this vector's
    *                 components. Not modified.
    */
   default void sub(int startRow, DenseMatrix64F matrix)
   {
      sub(startRow, 0, matrix);
   }

   /**
    * Subtracts from this vector's components the given column vector starting to read from
    * {@code startRow} at the column index {@code column}.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column   the column index to read in the dense-matrix.
    * @param matrix   the column vector containing the values to subtract from this vector's
    *                 components. Not modified.
    */
   default void sub(int startRow, int column, DenseMatrix64F matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 6, column + 1, matrix);
      getAngularPart().sub(matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column));
      getLinearPart().sub(matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column), matrix.unsafe_get(startRow++, column));
   }

   /**
    * Subtracts the given vectors to this vector angular and linear parts.
    * <p>
    * {@code this.angularPart -= angular}<br>
    * {@code this.linearPart -= linear}
    * </p>
    *
    * @param angular the vector to subtract to this vector's angular part. Not modified.
    * @param linear  the vector to subtract to this vector's linear part. Not modified.
    */
   default void sub(Vector3DReadOnly angular, Vector3DReadOnly linear)
   {
      getAngularPart().sub(angular);
      getLinearPart().sub(linear);
   }

   /**
    * Subtracts the given vectors to this vector angular and linear parts.
    * <p>
    * {@code this.angularPart -= angular}<br>
    * {@code this.linearPart -= linear}
    * </p>
    *
    * @param angular the vector to subtract to this vector's angular part. Not modified.
    * @param linear  the vector to subtract to this vector's linear part. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments are not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void sub(FrameVector3DReadOnly angular, FrameVector3DReadOnly linear)
   {
      getAngularPart().sub(angular);
      getLinearPart().sub(linear);
   }

   /**
    * Calculates the cross product of {@code tuple1} and {@code tuple2} and adds the result to this
    * vector's angular part.
    * <p>
    * {@code this.angularPart = this.angularPart + tuple1 x tuple2}.
    * </p>
    *
    * @param tuple1 the first tuple in the cross product. Not modified.
    * @param tuple2 the second tuple in the cross product. Not modified.
    */
   default void addCrossToAngularPart(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      MecanoTools.addCrossToVector(tuple1, tuple2, getAngularPart());
   }

   /**
    * Calculates the cross product of {@code tuple1} and {@code tuple2} and adds the result to this
    * vector's linear part.
    * <p>
    * {@code this.linearPart = this.linearPart + tuple1 x tuple2}.
    * </p>
    *
    * @param tuple1 the first tuple in the cross product. Not modified.
    * @param tuple2 the second tuple in the cross product. Not modified.
    */
   default void addCrossToLinearPart(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      MecanoTools.addCrossToVector(tuple1, tuple2, getLinearPart());
   }

   /** {@inheritDoc} */
   @Override
   default void applyTransform(Transform transform)
   {
      getAngularPart().applyTransform(transform);
      getLinearPart().applyTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      getAngularPart().applyInverseTransform(transform);
      getLinearPart().applyInverseTransform(transform);
   }
}
