package us.ihmc.mecano.spatial.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.Wrench;

/**
 * Write and read interface for a wrench which reference frames can be changed.
 * <p>
 * A wrench is a vector composed of 6 components with an angular part and a linear part. The angular
 * part represents a 3D torque and the linear part a 3D force.
 * </p>
 * <p>
 * The concept of a wrench is to describe an external spatial force, i.e. 3D torque and 3D force,
 * that is applied to a body. In this framework, the body on which the force is applied is referred
 * to using a reference frame commonly named {@code bodyFrame}. This reference frame is always
 * assumed to be rigidly attached to the body. As a result, a wrench is nothing more than a spatial
 * force to which a {@code bodyFrame} is associated.
 * </p>
 * <p>
 * When using a {@code WrenchBasics}, it is important to note that the reference frame in which it
 * is expressed does not only refer to the coordinate system in which the angular and linear 3D
 * vectors are expressed. The origin of the reference frame is also used as the point where the
 * spatial force is measured. Let's consider two reference frames A and B which axes are parallel
 * but have different origins, changing the frame of a spatial force vector from A to B will not
 * affect the linear part, i.e. the 3D force, but will still affect the value of the angular part,
 * i.e. the 3D moment. See {@link Wrench#changeFrame(ReferenceFrame)} for more information.
 * </p>
 * <p>
 * The convention when using a wrench in matrix operations is that the angular part occupies the 3
 * first rows and the linear part the 3 last as follows:<br>
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
public interface WrenchBasics extends FixedFrameWrenchBasics, SpatialForceBasics
{
   /**
    * Sets the frame attached to the body that the wrench is applied to.
    * <p>
    * This method does not modify anything but the body frame. If the new body frame is rigidly
    * attached to the same body, then and only then this wrench remains valid. Otherwise, the values of
    * this wrench should immediately be updated.
    * </p>
    *
    * @param bodyFrame the new body frame.
    */
   void setBodyFrame(ReferenceFrame bodyFrame);

   /**
    * Sets all the components of this vector to zero and updates its reference frames.
    *
    * @param bodyFrame        the new body frame.
    * @param expressedInFrame the new reference frame in which this motion is expressed.
    */
   default void setToZero(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setBodyFrame(bodyFrame);
      setToZero(expressedInFrame);
   }

   /**
    * Sets all the components of this vector to {@link Double#NaN} and sets its reference frames.
    *
    * @param bodyFrame        the new body frame.
    * @param expressedInFrame the new reference frame.
    */
   default void setToNaN(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setBodyFrame(bodyFrame);
      setToNaN(expressedInFrame);
   }

   /**
    * Sets this wrench to {@code other} including the reference frames.
    *
    * @param other the other wrench used to update {@code this}. Not modified.
    */
   default void setIncludingFrame(WrenchReadOnly other)
   {
      setIncludingFrame(other.getBodyFrame(), other);
   }

   /**
    * Sets this wrench to {@code spatialVector} and updates all its reference frames.
    *
    * @param bodyFrame     the body frame associated with the given spatial force.
    * @param spatialVector the spatial vector to copy values from. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(spatialVector);
   }

   /**
    * Sets this wrench given an angular part and linear part and updates all its reference frames.
    *
    * @param bodyFrame   the body frame associated with the given spatial force.
    * @param angularPart the vector holding the new values for the angular part. Not modified.
    * @param linearPart  the vector holding the new values for the linear part. Not modified.
    * @throws ReferenceFrameMismatchException if the given {@code angularPart} and {@code linearPart}
    *                                         are not expressed in the same reference frame.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(angularPart, linearPart);
   }

   /**
    * Sets this motion vector given an angular part and linear part and updates all its reference
    * frames.
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the given motion is expressed.
    * @param angularPart      the vector holding the new values for the angular part. Not modified.
    * @param linearPart       the vector holding the new values for the linear part. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, angularPart, linearPart);
   }

   /**
    * Sets this wrench's components from the given array {@code array} and updates all its reference
    * frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param array            the array containing the new values for this wrench's components. Not
    *                         modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, array);
   }

   /**
    * Sets this wrench's components from the given array {@code array} and updates all its reference
    * frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param startIndex       the first index to start reading from in the array.
    * @param array            the array containing the new values for this wrench's components. Not
    *                         modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, int startIndex, double[] array)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, startIndex, array);
   }

   /**
    * Sets this wrench's components from the given array {@code array} and updates all its reference
    * frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param array            the array containing the new values for this wrench's components. Not
    *                         modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, float[] array)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, array);
   }

   /**
    * Sets this wrench's components from the given array {@code array} and updates all its reference
    * frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param startIndex       the first index to start reading from in the array.
    * @param array            the array containing the new values for this wrench's components. Not
    *                         modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, int startIndex, float[] array)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, startIndex, array);
   }

   /**
    * Sets this wrench's components from the given column vector starting to read from {@code startRow}
    * and updates all its reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param matrix           the column vector containing the new values for this wrench's components.
    *                         Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, matrix);
   }

   /**
    * Sets this wrench's components from the given column vector starting to read from {@code startRow}
    * at the column index {@code column} and updates all its reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param startRow         the first row index to start reading in the dense-matrix.
    * @param matrix           the column vector containing the new values for this wrench's components.
    *                         Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, int startRow, DenseMatrix64F matrix)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, startRow, matrix);
   }

   /**
    * Sets this wrench's components from the given column vector starting to read from its first row
    * index and updates all its reference frames.
    * <p>
    * The components are read in the following order: {@code angularPartX}, {@code angularPartY},
    * {@code angularPartZ}, {@code linearPartX}, {@code linearPartY}, {@code linearPartZ}.
    * </p>
    *
    * @param bodyFrame        the body frame associated with the given spatial force.
    * @param expressedInFrame the reference frame in which the data is expressed.
    * @param startRow         the first row index to start reading in the dense-matrix.
    * @param column           the column index to read in the dense-matrix.
    * @param matrix           the column vector containing the new values for this wrench's components.
    *                         Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, int startRow, int column, DenseMatrix64F matrix)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, startRow, column, matrix);
   }

   /**
    * Sets this wrench given a 3D moment and 3D force that are exerted at {@code pointOfApplication}
    * and updates this vector reference frame.
    * <p>
    * Effectively, this wrench is updated as follow:
    *
    * <pre>
    * &tau;<sub>this</sub> = &tau;<sub>new</sub> + P &times; f<sub>new</sub>
    * f<sub>this</sub> = f<sub>new</sub>
    * </pre>
    *
    * where &tau; and f are the angular and linear parts respectively, and P is the
    * {@code pointOfApplication}.
    * </p>
    * <p>
    * When the given {@code angularPart} is {@code null}, it is assumed to be zero.
    * </p>
    *
    * @param bodyFrame          the body frame associated with the given spatial force.
    * @param expressedInFrame   the reference frame in which the arguments are expressed.
    * @param angularPart        the 3D moment that is applied. Can be {@code null}. Not modified.
    * @param linearPart         the 3D force that is applied. Not modified.
    * @param pointOfApplication the location where the force is exerted. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart,
                                  Point3DReadOnly pointOfApplication)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, angularPart, linearPart, pointOfApplication);
   }

   /**
    * Sets this spatial force given a 3D moment and 3D force that are exerted at
    * {@code pointOfApplication} and updates this vector reference frame.
    * <p>
    * Effectively, this wrench is updated as follow:
    *
    * <pre>
    * &tau;<sub>this</sub> = &tau;<sub>new</sub> + P &times; f<sub>new</sub>
    * f<sub>this</sub> = f<sub>new</sub>
    * </pre>
    *
    * where &tau; and f are the angular and linear parts respectively, and P is the
    * {@code pointOfApplication}.
    * </p>
    * <p>
    * When the given {@code angularPart} is {@code null}, it is assumed to be zero.
    * </p>
    *
    * @param bodyFrame          the body frame associated with the given spatial force.
    * @param expressedInFrame   the reference frame in which the arguments are expressed.
    * @param angularPart        the 3D moment that is applied. Can be {@code null}. Not modified.
    * @param linearPart         the 3D force that is applied. Not modified.
    * @param pointOfApplication the location where the force is exerted. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame.
    */
   default void setIncludingFrame(ReferenceFrame bodyFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart,
                                  FramePoint3DReadOnly pointOfApplication)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(angularPart, linearPart, pointOfApplication);
   }
}
