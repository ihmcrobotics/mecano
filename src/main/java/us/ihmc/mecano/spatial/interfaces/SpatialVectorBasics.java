package us.ihmc.mecano.spatial.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a spatial vector that is expressed in a changeable reference frame.
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
public interface SpatialVectorBasics extends FixedFrameSpatialVectorBasics, FrameChangeable
{

   /**
    * Sets the reference frame of this vector without changing the value of its components.
    */
   @Override
   void setReferenceFrame(ReferenceFrame expressedInFrame);

   /**
    * Sets all the components of this vector to zero and sets its reference frame to
    * {@code expressedInFrame}.
    *
    * @param expressedInFrame the new reference frame.
    */
   default void setToZero(ReferenceFrame expressedInFrame)
   {
      setReferenceFrame(expressedInFrame);
      setToZero();
   }

   /**
    * Sets all the components of this vector to {@link Double#NaN} and sets its reference frame to
    * {@code expressedInFrame}.
    *
    * @param expressedInFrame the new reference frame.
    */
   default void setToNaN(ReferenceFrame expressedInFrame)
   {
      setReferenceFrame(expressedInFrame);
      setToNaN();
   }

   /**
    * Sets this vector to {@code other} and updates the frame of this vector.
    *
    * @param other the other vector to copy. Not modified.
    */
   default void setIncludingFrame(SpatialVectorReadOnly other)
   {
      setReferenceFrame(other.getReferenceFrame());
      getAngularPart().set((Vector3DReadOnly) other.getAngularPart());
      getLinearPart().set((Vector3DReadOnly) other.getLinearPart());
   }

   /**
    * Sets this vector given an angular part and linear part and updates the frame of this vector.
    *
    * @param angularPart the vector holding the new values for the angular part and expressed in the
    *           new "expressed-in-frame" to use for this spatial vector. Not modified.
    * @param linearPart the vector holding the new values for the linear part and expressed in the
    *           new "expressed-in-frame" to use for this spatial vector. Not modified.
    * @throws ReferenceFrameMismatchException if {@code angularPart} and {@code linearPart} are not
    *            expressed in the same reference frame.
    */
   default void setIncludingFrame(FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      setIncludingFrame(angularPart.getReferenceFrame(), angularPart, linearPart);
   }

   /**
    * Sets this vector given an angular part and linear part and updates the frame of this vector.
    *
    * @param expressedInFrame the reference frame in which the vectors are expressed.
    * @param angularPart the vector holding the new values for the angular part. Not modified.
    * @param linearPart the vector holding the new values for the linear part. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setReferenceFrame(expressedInFrame);
      set(angularPart, linearPart);
   }

   /**
    * Sets this vector's components from the given array {@code array} and updates the frame of this
    * vector.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param array the array containing the new values for this vector's components. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, double[] array)
   {
      setReferenceFrame(expressedInFrame);
      set(array);
   }

   /**
    * Sets this vector's components from the given array {@code array} and updates the frame of this
    * vector.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param startIndex the first index to start reading from in the array.
    * @param array the array containing the new values for this vector's components. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, int startIndex, double[] array)
   {
      setReferenceFrame(expressedInFrame);
      set(startIndex, array);
   }

   /**
    * Sets this vector's components from the given array {@code array} and updates the frame of this
    * vector.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param array the array containing the new values for this vector's components. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, float[] array)
   {
      setIncludingFrame(expressedInFrame, 0, array);
   }

   /**
    * Sets this vector's components from the given array {@code array} and updates the frame of this
    * vector.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param startIndex the first index to start reading from in the array.
    * @param array the array containing the new values for this vector's components. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, int startIndex, float[] array)
   {
      setReferenceFrame(expressedInFrame);
      set(startIndex, array);
   }

   /**
    * Sets this vector's components from the given column vector starting to read from its first row
    * index and updates the frame of this vector.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param matrix the column vector containing the new values for this vector's components. Not
    *           modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setReferenceFrame(expressedInFrame);
      set(matrix);
   }

   /**
    * Sets this vector's components from the given column vector starting to read from
    * {@code startRow} and updates the frame of this vector.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix the column vector containing the new values for this vector's components. Not
    *           modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, int startRow, DenseMatrix64F matrix)
   {
      setReferenceFrame(expressedInFrame);
      set(startRow, matrix);
   }

   /**
    * Sets this vector's components from the given column vector starting to read from
    * {@code startRow} at the column index {@code column} and updates the frame of this vector.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column the column index to read in the dense-matrix.
    * @param matrix the column vector containing the new values for this vector's components. Not
    *           modified.
    */
   default void setIncludingFrame(ReferenceFrame expressedInFrame, int startRow, int column, DenseMatrix64F matrix)
   {
      setReferenceFrame(expressedInFrame);
      set(startRow, column, matrix);
   }
}
