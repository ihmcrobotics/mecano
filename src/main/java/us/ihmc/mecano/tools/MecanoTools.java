package us.ihmc.mecano.tools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ejml.MatrixDimensionException;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

/**
 * This class provides a variety of generic tools.
 *
 * @author Sylvain Bertrand
 */
public class MecanoTools
{
   /**
    * Change the first character of the given string to be upper-case only if needed.
    *
    * @param string the string to capitalize.
    * @return the capitalized string.
    */
   public static String capitalize(String string)
   {
      if (string == null || string.isEmpty())
         return string;

      if (Character.isUpperCase(string.charAt(0)))
         return string;

      return Character.toUpperCase(string.charAt(0)) + string.substring(1, string.length());
   }

   /**
    * Convenience method to calculate the dot product of two tuples.
    *
    * @param tuple1 the first tuple in the dot product. Not modified.
    * @param tuple2 the second tuple in the dot product. Not modified.
    * @return the value of the dot product of the two tuples.
    * @deprecated Use {@code TupleTools.dot(tuple1, tuple2)} instead.
    */
   @Deprecated
   public static double tuple3DDotProduct(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      return TupleTools.dot(tuple1, tuple2);
   }

   /**
    * Tests if the given matrix is symmetric.
    *
    * @param matrixToTest the 3D matrix to test for symmetry. Not modified.
    * @param epsilon      the tolerance to use in this test.
    * @return {@code true} if the given matrix is symmetric, {@code false} otherwise.
    */
   public static boolean isMatrix3DSymmetric(Matrix3DReadOnly matrixToTest, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(0.0, matrixToTest.getM01() - matrixToTest.getM10(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(0.0, matrixToTest.getM02() - matrixToTest.getM20(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(0.0, matrixToTest.getM12() - matrixToTest.getM21(), epsilon))
         return false;
      return true;
   }

   /**
    * Tests if the given matrix only has non-zero coefficients on its diagonal.
    *
    * @param matrixToTest the 3D matrix to test. Not modified.
    * @param epsilon      the tolerance used on the off-diagonal coefficients.
    * @return {@code true} if all the off-diagonal terms of the matrix are equal to zero, {@code false}
    *         otherwise.
    */
   public static boolean isMatrix3DDiagonal(Matrix3DReadOnly matrixToTest, double epsilon)
   {
      return isMatrix3DDiagonal(matrixToTest.getM00(),
                                matrixToTest.getM01(),
                                matrixToTest.getM02(),
                                matrixToTest.getM10(),
                                matrixToTest.getM11(),
                                matrixToTest.getM12(),
                                matrixToTest.getM20(),
                                matrixToTest.getM21(),
                                matrixToTest.getM22(),
                                epsilon);
   }

   /**
    * Tests if the given matrix only has non-zero coefficients on its diagonal.
    *
    * @param m00     first matrix element in the first row.
    * @param m01     second matrix element in the first row.
    * @param m02     third matrix element in the first row.
    * @param m10     first matrix element in the second row.
    * @param m11     second matrix element in the second row.
    * @param m12     third matrix element in the second row.
    * @param m20     first matrix element in the third row.
    * @param m21     second matrix element in the third row.
    * @param m22     third matrix element in the third row.
    * @param epsilon the tolerance used on the off-diagonal coefficients.
    * @return {@code true} if all the off-diagonal terms of the matrix are equal to zero, {@code false}
    *         otherwise.
    */
   public static boolean isMatrix3DDiagonal(double m00,
                                            double m01,
                                            double m02,
                                            double m10,
                                            double m11,
                                            double m12,
                                            double m20,
                                            double m21,
                                            double m22,
                                            double epsilon)
   {
      if (Math.abs(m01) <= epsilon && Math.abs(m02) <= epsilon && Math.abs(m12) <= epsilon)
      {
         if (Math.abs(m10) <= epsilon && Math.abs(m20) <= epsilon && Math.abs(m21) <= epsilon)
            return true;
      }
      return false;
   }

   /**
    * Asserts that the given matrix is symmetric.
    *
    * @param matrixToTest the 3D matrix to verify. Not modified.
    * @param epsilon      the tolerance to use.
    * @throws RuntimeException if the given matrix is not symmetric.
    */
   public static void checkIfMatrix3DIsSymmetric(Matrix3DReadOnly matrixToTest, double epsilon)
   {
      if (!isMatrix3DSymmetric(matrixToTest, epsilon))
         throw new RuntimeException("The matrix is not symmetric:\n" + matrixToTest.toString());
   }

   /**
    * Checks that the given {@code matrixToTest} has a minimum size of [{@code minRows},
    * {@code minColumns}].
    *
    * @param minRows      the minimum number of rows that the matrix should have.
    * @param minColumns   the minimum number of columns that the matrix should have.
    * @param matrixToTest the matrix which size is to be tested.
    * @throws MatrixDimensionException if the matrix does not meet the minimum size.
    * @deprecated Use {@code EuclidCoreTools.checkMatrixMinimumSize(minRows, minColumns, matrixToTest)}
    *             instead.
    */
   @Deprecated
   public static void checkMatrixMinimumSize(int minRows, int minColumns, DMatrix matrixToTest)
   {
      EuclidCoreTools.checkMatrixMinimumSize(minRows, minColumns, matrixToTest);
   }

   /**
    * Performs in-place the following operation:
    *
    * <pre>
    * vectorToModify += crossTerm1 &times; crossTerm2
    * </pre>
    *
    * @param crossTerm1     the first term of the cross product. Not modified.
    * @param crossTerm2     the second term of the cross product. Not modified.
    * @param vectorToModify the vector to which the result of the cross product should be added.
    *                       Modified.
    */
   public static void addCrossToVector(Tuple3DReadOnly crossTerm1, Tuple3DReadOnly crossTerm2, Vector3DBasics vectorToModify)
   {
      double tempX = vectorToModify.getX();
      double tempY = vectorToModify.getY();
      double tempZ = vectorToModify.getZ();
      vectorToModify.cross(crossTerm1, crossTerm2);
      vectorToModify.add(tempX, tempY, tempZ);
   }

   /**
    * Converts a tuple to tilde form (matrix implementation of cross product).
    *
    * <pre>
    *     /  0 -z  y \
    * M = |  z  0 -x |
    *     \ -y  x  0 /
    * </pre>
    *
    * @param tuple        the tuple to create the tilde form of. Not modified.
    * @param startRow     the row index at which the tilde matrix is to be inserted in
    *                     {@code matrixToPack}.
    * @param startColumn  the column index at which the tilde matrix is to be inserted in
    *                     {@code matrixToPack}.
    * @param matrixToPack the matrix used to store the result. Modified.
    */
   public static void toTildeForm(Tuple3DReadOnly tuple, int startRow, int startColumn, DMatrix matrixToPack)
   {
      toTildeForm(tuple, false, startRow, startColumn, matrixToPack);
   }

   /**
    * Converts a tuple to tilde form (matrix implementation of cross product).
    *
    * <pre>
    *     /  0 -z  y \
    * M = |  z  0 -x |
    *     \ -y  x  0 /
    * </pre>
    *
    * @param tuple        the tuple to create the tilde form of. Not modified.
    * @param transpose    whether the 3-by-3 tilde form is to be transposed before inserting it in
    *                     {@code matrixToPack}.
    * @param startRow     the row index at which the tilde matrix is to be inserted in
    *                     {@code matrixToPack}.
    * @param startColumn  the column index at which the tilde matrix is to be inserted in
    *                     {@code matrixToPack}.
    * @param matrixToPack the matrix used to store the result. Modified.
    */
   public static void toTildeForm(Tuple3DReadOnly tuple, boolean transpose, int startRow, int startColumn, DMatrix matrixToPack)
   {
      toTildeForm(1.0, tuple, transpose, startRow, startColumn, matrixToPack);
   }

   /**
    * Converts a tuple to tilde form (matrix implementation of cross product).
    *
    * <pre>
    *         /  0 -z  y \
    * M = s * |  z  0 -x |
    *         \ -y  x  0 /
    * </pre>
    *
    * @param scale        scale factor to apply to the components of the resulting matrix.
    * @param tuple        the tuple to create the tilde form of. Not modified.
    * @param transpose    whether the 3-by-3 tilde form is to be transposed before inserting it in
    *                     {@code matrixToPack}.
    * @param startRow     the row index at which the tilde matrix is to be inserted in
    *                     {@code matrixToPack}.
    * @param startColumn  the column index at which the tilde matrix is to be inserted in
    *                     {@code matrixToPack}.
    * @param matrixToPack the matrix used to store the result. Modified.
    */
   public static void toTildeForm(double scale, Tuple3DReadOnly tuple, boolean transpose, int startRow, int startColumn, DMatrix matrixToPack)
   {
      double x, y, z;
      if (transpose)
      {
         x = -scale * tuple.getX();
         y = -scale * tuple.getY();
         z = -scale * tuple.getZ();
      }
      else
      {
         x = scale * tuple.getX();
         y = scale * tuple.getY();
         z = scale * tuple.getZ();
      }

      int row = startRow;
      matrixToPack.set(row, startColumn, 0.0);
      matrixToPack.set(row, startColumn + 1, -z);
      matrixToPack.set(row, startColumn + 2, y);

      row++;
      matrixToPack.set(row, startColumn, z);
      matrixToPack.set(row, startColumn + 1, 0.0);
      matrixToPack.set(row, startColumn + 2, -x);

      row++;
      matrixToPack.set(row, startColumn, -y);
      matrixToPack.set(row, startColumn + 1, x);
      matrixToPack.set(row, startColumn + 2, 0.0);
   }

   /**
    * Adds the given {@code matrixToAdd} to {@code matrixToModify}.
    *
    * @param startRow       the first row index in {@code matrixToAdd} to start reading elements.
    * @param startColumn    the first column index in {@code matrixToAdd} to start reading elements.
    * @param matrixToAdd    the matrix to be added. Not modified.
    * @param matrixToModify the matrix to which the {@code DenseMatrix64F} is to be added. Modified.
    */
   public static void addEquals(int startRow, int startColumn, DMatrix matrixToAdd, Matrix3DBasics matrixToModify)
   {
      int row = startRow;
      double m00 = matrixToModify.getM00() + matrixToAdd.get(row, startColumn);
      double m01 = matrixToModify.getM01() + matrixToAdd.get(row, startColumn + 1);
      double m02 = matrixToModify.getM02() + matrixToAdd.get(row, startColumn + 2);
      row++;
      double m10 = matrixToModify.getM10() + matrixToAdd.get(row, startColumn);
      double m11 = matrixToModify.getM11() + matrixToAdd.get(row, startColumn + 1);
      double m12 = matrixToModify.getM12() + matrixToAdd.get(row, startColumn + 2);
      row++;
      double m20 = matrixToModify.getM20() + matrixToAdd.get(row, startColumn);
      double m21 = matrixToModify.getM21() + matrixToAdd.get(row, startColumn + 1);
      double m22 = matrixToModify.getM22() + matrixToAdd.get(row, startColumn + 2);
      matrixToModify.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Subtracts the given {@code matrixToAdd} to {@code matrixToModify}.
    *
    * @param startRow       the first row index in {@code matrixToAdd} to start reading elements.
    * @param startColumn    the first column index in {@code matrixToAdd} to start reading elements.
    * @param matrixToAdd    the matrix to be subtracted. Not modified.
    * @param matrixToModify the matrix to which the {@code DenseMatrix64F} is to be subtracted.
    *                       Modified.
    */
   public static void subEquals(int startRow, int startColumn, DMatrix matrixToAdd, Matrix3DBasics matrixToModify)
   {
      int row = startRow;
      double m00 = matrixToModify.getM00() - matrixToAdd.get(row, startColumn);
      double m01 = matrixToModify.getM01() - matrixToAdd.get(row, startColumn + 1);
      double m02 = matrixToModify.getM02() - matrixToAdd.get(row, startColumn + 2);
      row++;
      double m10 = matrixToModify.getM10() - matrixToAdd.get(row, startColumn);
      double m11 = matrixToModify.getM11() - matrixToAdd.get(row, startColumn + 1);
      double m12 = matrixToModify.getM12() - matrixToAdd.get(row, startColumn + 2);
      row++;
      double m20 = matrixToModify.getM20() - matrixToAdd.get(row, startColumn);
      double m21 = matrixToModify.getM21() - matrixToAdd.get(row, startColumn + 1);
      double m22 = matrixToModify.getM22() - matrixToAdd.get(row, startColumn + 2);
      matrixToModify.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Adds the given {@code vectorToAdd} as a column into {@code matrixToModify}
    * 
    * @param startRow       the first row index in {@code matrixToModify} to start adding.
    * @param column         the column index in {@code matrixToModify} where the vector is added.
    * @param vectorToAdd    the vector to be added. Not modified.
    * @param matrixToModify the matrix to which the vector is added. Modified.
    */
   public static void addEquals(int startRow, int column, SpatialVectorReadOnly vectorToAdd, DMatrixRMaj matrixToModify)
   {
      matrixToModify.add(startRow + 0, column, vectorToAdd.getAngularPartX());
      matrixToModify.add(startRow + 1, column, vectorToAdd.getAngularPartY());
      matrixToModify.add(startRow + 2, column, vectorToAdd.getAngularPartZ());
      matrixToModify.add(startRow + 3, column, vectorToAdd.getLinearPartX());
      matrixToModify.add(startRow + 4, column, vectorToAdd.getLinearPartY());
      matrixToModify.add(startRow + 5, column, vectorToAdd.getLinearPartZ());
   }

   /**
    * Applies a translation to the given moment of inertia.
    * <p>
    * The translation formula for the inertia is as follows:
    *
    * <pre>
    * I -= m ([p]<sub>&times;</sub>[c]<sub>&times;</sub> + [c]<sub>&times;</sub>[p]<sub>&times;</sub> + [p]<sub>&times;</sub>[p]<sub>&times;</sub>)
    * </pre>
    *
    * where <tt>m</tt> the mass of the body, <tt>c</tt> is the body's center of mass position expressed
    * in the same reference frame of the original inertia matrix, <tt>p</tt> is the translation to
    * apply to the inertia matrix, "[.]<sub>&times;</sub>" is the operator that maps a vector to a skew
    * symmetric matrix, for any vector w = [x y z]<sup>T</sup>:
    *
    * <pre>
    *    <sub> </sub>   /  0 -z  y \
    * [w]<sub>&times;</sub> = |  z  0 -x |
    *    <sub> </sub>   \ -y  x  0 /
    * </pre>
    * 
    * The {@code centerOfMass} is assumed to be zero when {@code null} is passed.
    * </p>
    *
    * @param mass                       the mass of the corresponding rigid body.
    * @param centerOfMass               the coordinates of the center of mass expressed in the same
    *                                   frame as the given {@code massMomentOfInertiaToTransform}. Can
    *                                   be {@code null}. Not modified.
    * @param negateTranslation          whether to negate the translation before transforming the
    *                                   moment of inertia.
    * @param translation                the translation to apply to the mass moment of inertia matrix.
    *                                   Not modified.
    * @param momentOfInertiaToTransform the inertia matrix to transform. Modified.
    */
   public static void translateMomentOfInertia(double mass,
                                               Tuple3DReadOnly centerOfMass,
                                               boolean negateTranslation,
                                               Tuple3DReadOnly translation,
                                               Matrix3DBasics momentOfInertiaToTransform)
   {
      double xp = translation.getX();
      double yp = translation.getY();
      double zp = translation.getZ();

      if (negateTranslation)
      {
         xp = -xp;
         yp = -yp;
         zp = -zp;
      }

      double xp_xp = xp * xp;
      double yp_yp = yp * yp;
      double zp_zp = zp * zp;

      double txx, tyy, tzz;
      double txy, txz, tyz;

      if (centerOfMass != null)
      {
         double xc = centerOfMass.getX();
         double yc = centerOfMass.getY();
         double zc = centerOfMass.getZ();

         double two_xc_xp = 2.0 * xc * xp;
         double two_yc_yp = 2.0 * yc * yp;
         double two_zc_zp = 2.0 * zc * zp;

         txx = mass * (two_yc_yp + two_zc_zp + yp_yp + zp_zp);
         tyy = mass * (two_xc_xp + two_zc_zp + xp_xp + zp_zp);
         tzz = mass * (two_xc_xp + two_yc_yp + xp_xp + yp_yp);

         txy = mass * (-xc * yp - yc * xp - xp * yp);
         txz = mass * (-xc * zp - zc * xp - xp * zp);
         tyz = mass * (-yc * zp - zc * yp - yp * zp);
      }
      else
      {
         txx = mass * (yp_yp + zp_zp);
         tyy = mass * (xp_xp + zp_zp);
         tzz = mass * (xp_xp + yp_yp);

         txy = -mass * xp * yp;
         txz = -mass * xp * zp;
         tyz = -mass * yp * zp;
      }

      double m00 = momentOfInertiaToTransform.getM00() + txx;
      double m01 = momentOfInertiaToTransform.getM01() + txy;
      double m02 = momentOfInertiaToTransform.getM02() + txz;
      double m10 = momentOfInertiaToTransform.getM10() + txy;
      double m11 = momentOfInertiaToTransform.getM11() + tyy;
      double m12 = momentOfInertiaToTransform.getM12() + tyz;
      double m20 = momentOfInertiaToTransform.getM20() + txz;
      double m21 = momentOfInertiaToTransform.getM21() + tyz;
      double m22 = momentOfInertiaToTransform.getM22() + tzz;

      momentOfInertiaToTransform.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Using Newton's second law of motion with a rigid-body to compute the net moment from its
    * acceleration.
    * <p>
    * This method assumes that all the arguments are about and expressed at the center of mass. This
    * simplifies greatly the equations.
    * </p>
    * <p>
    * If the given {@code angularAcceleration} or {@code angularVelocity} is {@code null}, it is
    * assumed to be equal to zero.
    * </p>
    *
    * @param momentOfInertia     the moment of inertia of the rigid body expressed at its center of
    *                            mass. Not modified.
    * @param angularAcceleration the angular acceleration of the rigid-body with respect to an inertial
    *                            frame and expressed in the same frame as the inertia matrix. Can be
    *                            {@code null}. Not modified.
    * @param angularVelocity     the angular velocity of the rigid-body with respect to an inertial
    *                            frame and expressed in the same frame as the inertia matrix. Can be
    *                            {@code null}. Not modified.
    * @param dynamicMomentToPack vector used to store the net moment at the center of mass. Modified.
    */
   public static void computeDynamicMomentFast(Matrix3DReadOnly momentOfInertia,
                                               Vector3DReadOnly angularAcceleration,
                                               Vector3DReadOnly angularVelocity,
                                               Vector3DBasics dynamicMomentToPack)
   {
      if (angularVelocity == null)
      {
         dynamicMomentToPack.setToZero();
      }
      else
      {
         // J w
         momentOfInertia.transform(angularVelocity, dynamicMomentToPack);
         // w x J w
         dynamicMomentToPack.cross(angularVelocity, dynamicMomentToPack);
      }

      if (angularAcceleration != null)
      {
         double mx = dynamicMomentToPack.getX();
         double my = dynamicMomentToPack.getY();
         double mz = dynamicMomentToPack.getZ();
         // J wDot
         momentOfInertia.transform(angularAcceleration, dynamicMomentToPack);
         // J wDot + w x J w
         dynamicMomentToPack.add(mx, my, mz);
      }
   }

   /**
    * Using Newton's second law of motion with a rigid-body to compute the net moment from its
    * acceleration.
    * <p>
    * Besides the requirement that all arguments should be expressed the same frame, this method does
    * not make other assumption regarding the said frame.
    * </p>
    * <p>
    * If the given {@code angularAcceleration}, {@code linearAcceleration}, {@code angularVelocity}, or
    * {@code linearVelocity} is {@code null}, it is assumed to be equal to zero.
    * </p>
    *
    * @param momentOfInertia     the moment of inertia of the rigid body. Not modified.
    * @param mass                the rigid body mass.
    * @param centerOfMassOffset  the offset of the center of mass position with respect to the origin
    *                            of the frame in which the inertia is expressed. Not modified.
    * @param angularAcceleration the angular acceleration of the rigid-body with respect to an inertial
    *                            frame and expressed in the same frame as the inertia matrix. Can be
    *                            {@code null}. Not modified.
    * @param linearAcceleration  the linear acceleration of the origin of the frame in which the
    *                            inertia is expressed with respect to an inertial frame and expressed
    *                            in the same frame as the inertia matrix. Can be {@code null}. Not
    *                            modified.
    * @param angularVelocity     the angular velocity of the rigid-body with respect to an inertial
    *                            frame and expressed in the same frame as the inertia matrix. Can be
    *                            {@code null}. Not modified.
    * @param linearVelocity      the linear velocity of the origin of the frame in which the inertia is
    *                            expressed with respect to an inertial frame and expressed in the same
    *                            frame as the inertia matrix. Can be {@code null}. Not modified.
    * @param dynamicMomentToPack vector used to store the net moment at the origin of the frame in
    *                            which the inertia is expressed. Modified.
    */
   public static void computeDynamicMoment(Matrix3DReadOnly momentOfInertia,
                                           double mass,
                                           Vector3DReadOnly centerOfMassOffset,
                                           Vector3DReadOnly angularAcceleration,
                                           Vector3DReadOnly linearAcceleration,
                                           Vector3DReadOnly angularVelocity,
                                           Vector3DReadOnly linearVelocity,
                                           Vector3DBasics dynamicMomentToPack)
   {
      // Temporary variables to store intermediate results.
      double mx, my, mz;

      if (linearAcceleration == null)
      {
         dynamicMomentToPack.setToZero();
      }
      else
      {
         // c x a
         dynamicMomentToPack.cross(centerOfMassOffset, linearAcceleration);
      }

      if (angularVelocity != null)
      {
         if (linearVelocity != null)
         {
            // w . c
            double omega_dot_c = angularVelocity.dot(centerOfMassOffset);
            // v . c
            double v_dot_c = linearVelocity.dot(centerOfMassOffset);

            // c x a + w (v . c) - v (w . c)
            dynamicMomentToPack.addX(angularVelocity.getX() * v_dot_c - linearVelocity.getX() * omega_dot_c);
            dynamicMomentToPack.addY(angularVelocity.getY() * v_dot_c - linearVelocity.getY() * omega_dot_c);
            dynamicMomentToPack.addZ(angularVelocity.getZ() * v_dot_c - linearVelocity.getZ() * omega_dot_c);
            // m ( c x a + w (v . c) - v (w . c) )
            dynamicMomentToPack.scale(mass);
         }

         mx = dynamicMomentToPack.getX();
         my = dynamicMomentToPack.getY();
         mz = dynamicMomentToPack.getZ();

         // J w
         momentOfInertia.transform(angularVelocity, dynamicMomentToPack);
         // w x J w
         dynamicMomentToPack.cross(angularVelocity, dynamicMomentToPack);
         // w x J w + m ( c x a + w (v . c) - v (w . c) )
         mx += dynamicMomentToPack.getX();
         my += dynamicMomentToPack.getY();
         mz += dynamicMomentToPack.getZ();
      }
      else
      {
         mx = dynamicMomentToPack.getX();
         my = dynamicMomentToPack.getY();
         mz = dynamicMomentToPack.getZ();
      }

      if (angularAcceleration != null)
      {
         // J wDot
         momentOfInertia.transform(angularAcceleration, dynamicMomentToPack);
         // J wDot + w x J w + m ( c x a + w (v . c) - v (w . c) )
         dynamicMomentToPack.add(mx, my, mz);
      }
      else
      {
         dynamicMomentToPack.set(mx, my, mz);
      }
   }

   /**
    * Using Newton's second law of motion with a rigid-body to compute the net force from its
    * acceleration.
    * <p>
    * This method assumes that all the arguments are about and expressed at the center of mass. This
    * simplifies greatly the equations.
    * </p>
    * <p>
    * If the given {@code linearAcceleration}, {@code angularVelocity}, or {@code linearVelocity} is
    * {@code null}, it is assumed to be equal to zero.
    * </p>
    *
    * @param mass               the rigid body mass.
    * @param linearAcceleration the linear acceleration of the origin of the frame in which the inertia
    *                           is expressed with respect to an inertial frame and expressed in the
    *                           same frame as the inertia matrix. Can be {@code null}. Not modified.
    * @param angularVelocity    the angular velocity of the rigid-body with respect to an inertial
    *                           frame and expressed in the same frame as the inertia matrix. Can be
    *                           {@code null}. Not modified.
    * @param linearVelocity     the linear velocity of the origin of the frame in which the inertia is
    *                           expressed with respect to an inertial frame and expressed in the same
    *                           frame as the inertia matrix. Can be {@code null}. Not modified.
    * @param dynamicForceToPack vector used to store the net force at the center of mass. Modified.
    */
   public static void computeDynamicForceFast(double mass,
                                              Vector3DReadOnly linearAcceleration,
                                              Vector3DReadOnly angularVelocity,
                                              Vector3DReadOnly linearVelocity,
                                              Vector3DBasics dynamicForceToPack)
   {
      if (angularVelocity != null && linearVelocity != null)
      {
         dynamicForceToPack.cross(angularVelocity, linearVelocity);

         if (linearAcceleration != null)
            dynamicForceToPack.add(linearAcceleration);
         dynamicForceToPack.scale(mass);
      }
      else
      {
         dynamicForceToPack.setToZero();

         if (linearAcceleration != null)
         {
            dynamicForceToPack.add(linearAcceleration);
            dynamicForceToPack.scale(mass);
         }
      }
   }

   /**
    * Using Newton's second law of motion with a rigid-body to compute the net force from its
    * acceleration.
    * <p>
    * Besides the requirement that all arguments should be expressed the same frame, this method does
    * not make other assumption regarding the said frame.
    * </p>
    * <p>
    * If the given {@code angularAcceleration}, {@code linearAcceleration}, {@code angularVelocity}, or
    * {@code linearVelocity} is {@code null}, it is assumed to be equal to zero.
    * </p>
    *
    * @param mass                the rigid body mass.
    * @param centerOfMassOffset  the offset of the center of mass position with respect to the origin
    *                            of the frame in which the inertia is expressed. Not modified.
    * @param angularAcceleration the angular acceleration of the rigid-body with respect to an inertial
    *                            frame and expressed in the same frame as the inertia matrix. Can be
    *                            {@code null}. Not modified.
    * @param linearAcceleration  the linear acceleration of the origin of the frame in which the
    *                            inertia is expressed with respect to an inertial frame and expressed
    *                            in the same frame as the inertia matrix. Can be {@code null}. Not
    *                            modified.
    * @param angularVelocity     the angular velocity of the rigid-body with respect to an inertial
    *                            frame and expressed in the same frame as the inertia matrix. Can be
    *                            {@code null}. Not modified.
    * @param linearVelocity      the linear velocity of the origin of the frame in which the inertia is
    *                            expressed with respect to an inertial frame and expressed in the same
    *                            frame as the inertia matrix. Can be {@code null}. Not modified.
    * @param dynamicForceToPack  vector used to store the net force at the origin of the frame in which
    *                            the inertia is expressed. Modified.
    */
   public static void computeDynamicForce(double mass,
                                          Vector3DReadOnly centerOfMassOffset,
                                          Vector3DReadOnly angularAcceleration,
                                          Vector3DReadOnly linearAcceleration,
                                          Vector3DReadOnly angularVelocity,
                                          Vector3DReadOnly linearVelocity,
                                          Vector3DBasics dynamicForceToPack)
   {
      if (angularVelocity != null)
      {
         // w x (c x w - v)
         dynamicForceToPack.cross(centerOfMassOffset, angularVelocity);
         if (linearVelocity != null)
            dynamicForceToPack.sub(linearVelocity);
         dynamicForceToPack.cross(angularVelocity, dynamicForceToPack);
      }
      else
      {
         dynamicForceToPack.setToZero();
      }

      if (angularAcceleration != null)
      {
         // c x wDot + w x (c x w - v)
         addCrossToVector(centerOfMassOffset, angularAcceleration, dynamicForceToPack);
      }

      // - c x wDot - w x (c x w - v)
      dynamicForceToPack.negate();

      if (linearAcceleration != null)
      {
         // a - c x wDot - w x (c x w - v)
         dynamicForceToPack.add(linearAcceleration);
      }

      dynamicForceToPack.scale(mass);
   }

   /**
    * Calculates the kinetic co-energy as introduced in the Ph.D. thesis of Vincent Duindam entitled
    * <i>"Port-Based Modeling and Control for Efficient Bipedal Walking Robots"</i>, page 38.
    * <p>
    * This method does not require the moment of inertia to be expressed in any particular frame.
    * However, all the arguments should be expressed in the same frame.
    * </p>
    *
    * @param momentOfInertia    the moment of inertia of the rigid body. Not modified.
    * @param mass               the rigid body mass.
    * @param centerOfMassOffset the offset of the center of mass position with respect to the origin of
    *                           the frame in which the inertia is expressed. Not modified.
    * @param angularVelocity    the angular velocity of the rigid-body with respect to an inertial
    *                           frame and expressed in the same frame as the inertia matrix. Not
    *                           modified.
    * @param linearVelocity     the linear velocity of the origin of the frame in which the inertia is
    *                           expressed with respect to an inertial frame and expressed in the same
    *                           frame as the inertia matrix. Not modified.
    * @return the value of the kinetic co-energy.
    */
   public static double computeKineticCoEnergy(Matrix3DReadOnly momentOfInertia,
                                               double mass,
                                               Vector3DReadOnly centerOfMassOffset,
                                               Vector3DReadOnly angularVelocity,
                                               Vector3DReadOnly linearVelocity)
   {
      // m v^2
      double energy = mass * linearVelocity.normSquared();

      double cx = centerOfMassOffset.getX();
      double cy = centerOfMassOffset.getY();
      double cz = centerOfMassOffset.getZ();

      double wx = angularVelocity.getX();
      double wy = angularVelocity.getY();
      double wz = angularVelocity.getZ();

      double vx = linearVelocity.getX();
      double vy = linearVelocity.getY();
      double vz = linearVelocity.getZ();

      double ixx = momentOfInertia.getM00();
      double iyy = momentOfInertia.getM11();
      double izz = momentOfInertia.getM22();

      double ixy = momentOfInertia.getM01();
      double ixz = momentOfInertia.getM02();
      double iyz = momentOfInertia.getM12();

      double iyx = momentOfInertia.getM10();
      double izx = momentOfInertia.getM20();
      double izy = momentOfInertia.getM21();

      // c x v
      double c_cross_v_x = cy * vz - cz * vy;
      double c_cross_v_y = cz * vx - cx * vz;
      double c_cross_v_z = cx * vy - cy * vx;
      // 2 m w . ( c x v )
      energy += 2.0 * mass * (wx * c_cross_v_x + wy * c_cross_v_y + wz * c_cross_v_z);

      // J w
      double j_w_x = ixx * wx + ixy * wy + ixz * wz;
      double j_w_y = iyx * wx + iyy * wy + iyz * wz;
      double j_w_z = izx * wx + izy * wy + izz * wz;
      energy += wx * j_w_x + wy * j_w_y + wz * j_w_z;

      return 0.5 * energy;
   }

   /**
    * Attempts to cast the given joint to the type {@code T}.
    *
    * @param <T>           the desired joint type.
    * @param jointToCast   the joint to be casted to the type {@code T}. Not modified.
    * @param classToCastTo the class the joint is susceptible to be an instance of.
    * @return the joint casted to the type {@code T}.
    */
   @SuppressWarnings("unchecked")
   public static <T extends JointReadOnly> T checkTypeAndCast(JointReadOnly jointToCast, Class<T> classToCastTo)
   {
      if (classToCastTo.isInstance(jointToCast))
         return (T) jointToCast;
      else
         throw new RuntimeException("Cannot cast " + jointToCast.getClass().getSimpleName() + " to " + classToCastTo.getSimpleName());
   }

   /**
    * Creates the list of unit-twists for a planar joint.
    * <p>
    * For each degree of freedom of the joint there is one unit-twist.
    * </p>
    *
    * @param beforeJointFrame the frame right before the joint.
    * @param afterJointFrame  the frame right after the joint.
    * @return the list of unit-twists.
    */
   public static List<TwistReadOnly> computePlanarJointMotionSubspace(ReferenceFrame beforeJointFrame, ReferenceFrame afterJointFrame)
   {
      int nDegreesOfFreedom = PlanarJointReadOnly.NUMBER_OF_DOFS;

      RigidBodyTransform identity = new RigidBodyTransform();

      ReferenceFrame[] intermediateFrames = new ReferenceFrame[nDegreesOfFreedom];
      intermediateFrames[nDegreesOfFreedom - 1] = afterJointFrame;

      for (int dofIndex = intermediateFrames.length - 2; dofIndex >= 0; dofIndex--) // from afterJointFrame to beforeJointFrame
      {
         ReferenceFrame parentFrame = intermediateFrames[dofIndex + 1];
         String frameName = "intermediateFrame" + dofIndex;
         intermediateFrames[dofIndex] = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parentFrame, identity);
      }

      List<TwistReadOnly> unitTwistsInBodyFrame = new ArrayList<>();
      ReferenceFrame previousFrame = beforeJointFrame;
      int[] twistComponentIndex = {1, 3, 5}; // w_y, v_x, v_z

      for (int dofIndex = 0; dofIndex < nDegreesOfFreedom; dofIndex++) // from beforeJointFrame to afterJointFrame
      {
         ReferenceFrame frame = intermediateFrames[dofIndex];

         Twist twist = new Twist(frame, previousFrame, frame);
         twist.setElement(twistComponentIndex[dofIndex], 1.0);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      return Collections.unmodifiableList(unitTwistsInBodyFrame);
   }

   /**
    * Creates the list of unit-twists for a 6-DoF joint.
    * <p>
    * For each degree of freedom of the joint there is one unit-twist.
    * </p>
    *
    * @param beforeJointFrame the frame right before the joint.
    * @param afterJointFrame  the frame right after the joint.
    * @return the list of unit-twists.
    */
   public static List<TwistReadOnly> computeSixDoFJointMotionSubspace(ReferenceFrame beforeJointFrame, ReferenceFrame afterJointFrame)
   {
      int nDegreesOfFreedom = SixDoFJointReadOnly.NUMBER_OF_DOFS;

      RigidBodyTransform identity = new RigidBodyTransform();

      ReferenceFrame[] intermediateFrames = new ReferenceFrame[nDegreesOfFreedom];
      intermediateFrames[nDegreesOfFreedom - 1] = afterJointFrame;

      for (int dofIndex = intermediateFrames.length - 2; dofIndex >= 0; dofIndex--) // from afterJointFrame to beforeJointFrame
      {
         ReferenceFrame parentFrame = intermediateFrames[dofIndex + 1];
         String frameName = "intermediateFrame" + dofIndex;
         intermediateFrames[dofIndex] = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parentFrame, identity);
      }

      List<TwistReadOnly> unitTwistsInBodyFrame = new ArrayList<>();
      ReferenceFrame previousFrame = beforeJointFrame;

      for (int dofIndex = 0; dofIndex < nDegreesOfFreedom; dofIndex++) // from beforeJointFrame to afterJointFrame
      {
         ReferenceFrame frame = intermediateFrames[dofIndex];

         Twist twist = new Twist(frame, previousFrame, frame);
         twist.setElement(dofIndex, 1.0);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      return Collections.unmodifiableList(unitTwistsInBodyFrame);
   }

   /**
    * Creates the list of unit-twists for a spherical joint.
    * <p>
    * For each degree of freedom of the joint there is one unit-twist.
    * </p>
    *
    * @param beforeJointFrame the frame right before the joint.
    * @param afterJointFrame  the frame right after the joint.
    * @return the list of unit-twists.
    */
   public static List<TwistReadOnly> computeSphericalJointMotionSubspace(ReferenceFrame beforeJointFrame, ReferenceFrame afterJointFrame)
   {
      int nDegreesOfFreedom = SphericalJointReadOnly.NUMBER_OF_DOFS;

      RigidBodyTransform identity = new RigidBodyTransform();

      ReferenceFrame[] intermediateFrames = new ReferenceFrame[nDegreesOfFreedom];
      intermediateFrames[nDegreesOfFreedom - 1] = afterJointFrame;

      for (int dofIndex = intermediateFrames.length - 2; dofIndex >= 0; dofIndex--) // from afterJointFrame to beforeJointFrame
      {
         ReferenceFrame parentFrame = intermediateFrames[dofIndex + 1];
         String frameName = "intermediateFrame" + dofIndex;
         intermediateFrames[dofIndex] = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parentFrame, identity);
      }

      ArrayList<Twist> unitTwistsInBodyFrame = new ArrayList<>();
      ReferenceFrame previousFrame = beforeJointFrame;

      for (int dofIndex = 0; dofIndex < nDegreesOfFreedom; dofIndex++) // from beforeJointFrame to afterJointFrame
      {
         ReferenceFrame frame = intermediateFrames[dofIndex];

         Twist twist = new Twist(frame, previousFrame, frame);
         twist.setElement(dofIndex, 1.0);
         unitTwistsInBodyFrame.add(twist);

         previousFrame = frame;
      }

      return Collections.unmodifiableList(unitTwistsInBodyFrame);
   }

   /**
    * Optimized version of the transformation
    * {@link Matrix3DTools#transform(Matrix3DReadOnly, Matrix3DReadOnly, Matrix3DBasics)} for symmetric
    * matrices:
    * <p>
    * {@code symmetricMatrixToRotate} = {@code rotation} * {@code symmetricMatrixToRotate} *
    * {@code rotation}<sup>-1</sup>.
    * </p>
    *
    * @param rotation                the rotation to apply to the given matrix. Not modified.
    * @param symmetricMatrixToRotate the symmetric matrix to be transformed. Modified.
    * @see Matrix3DTools#transform(Matrix3DReadOnly, Matrix3DReadOnly, Matrix3DBasics)
    */
   public static void transformSymmetricMatrix3D(RotationMatrixReadOnly rotation, Matrix3DBasics symmetricMatrixToRotate)
   {
      Matrix3DTools.multiply(rotation, symmetricMatrixToRotate, symmetricMatrixToRotate);

      double m00 = symmetricMatrixToRotate.getM00() * rotation.getM00() + symmetricMatrixToRotate.getM01() * rotation.getM01()
            + symmetricMatrixToRotate.getM02() * rotation.getM02();
      double m01 = symmetricMatrixToRotate.getM00() * rotation.getM10() + symmetricMatrixToRotate.getM01() * rotation.getM11()
            + symmetricMatrixToRotate.getM02() * rotation.getM12();
      double m02 = symmetricMatrixToRotate.getM00() * rotation.getM20() + symmetricMatrixToRotate.getM01() * rotation.getM21()
            + symmetricMatrixToRotate.getM02() * rotation.getM22();
      double m11 = symmetricMatrixToRotate.getM10() * rotation.getM10() + symmetricMatrixToRotate.getM11() * rotation.getM11()
            + symmetricMatrixToRotate.getM12() * rotation.getM12();
      double m12 = symmetricMatrixToRotate.getM10() * rotation.getM20() + symmetricMatrixToRotate.getM11() * rotation.getM21()
            + symmetricMatrixToRotate.getM12() * rotation.getM22();
      double m22 = symmetricMatrixToRotate.getM20() * rotation.getM20() + symmetricMatrixToRotate.getM21() * rotation.getM21()
            + symmetricMatrixToRotate.getM22() * rotation.getM22();
      symmetricMatrixToRotate.set(m00, m01, m02, m01, m11, m12, m02, m12, m22);
   }

   /**
    * Optimized version of the transformation
    * {@link Matrix3DTools#inverseTransform(Matrix3DReadOnly, Matrix3DReadOnly, Matrix3DBasics)} for
    * symmetric matrices:
    * <p>
    * {@code symmetricMatrixToRotate} = {@code rotation}<sup>-1</sup> * {@code symmetricMatrixToRotate}
    * * {@code rotation}.
    * </p>
    *
    * @param rotation                the rotation to apply to the given matrix. Not modified.
    * @param symmetricMatrixToRotate the symmetric matrix to be transformed. Modified.
    * @see Matrix3DTools#inverseTransform(Matrix3DReadOnly, Matrix3DReadOnly, Matrix3DBasics)
    */
   public static void inverseTransformSymmetricMatrix3D(RotationMatrixReadOnly rotation, Matrix3DBasics symmetricMatrixToRotate)
   {
      Matrix3DTools.multiplyTransposeLeft(rotation, symmetricMatrixToRotate, symmetricMatrixToRotate);

      double m00 = symmetricMatrixToRotate.getM00() * rotation.getM00() + symmetricMatrixToRotate.getM01() * rotation.getM10()
            + symmetricMatrixToRotate.getM02() * rotation.getM20();
      double m01 = symmetricMatrixToRotate.getM00() * rotation.getM01() + symmetricMatrixToRotate.getM01() * rotation.getM11()
            + symmetricMatrixToRotate.getM02() * rotation.getM21();
      double m02 = symmetricMatrixToRotate.getM00() * rotation.getM02() + symmetricMatrixToRotate.getM01() * rotation.getM12()
            + symmetricMatrixToRotate.getM02() * rotation.getM22();
      double m11 = symmetricMatrixToRotate.getM10() * rotation.getM01() + symmetricMatrixToRotate.getM11() * rotation.getM11()
            + symmetricMatrixToRotate.getM12() * rotation.getM21();
      double m12 = symmetricMatrixToRotate.getM10() * rotation.getM02() + symmetricMatrixToRotate.getM11() * rotation.getM12()
            + symmetricMatrixToRotate.getM12() * rotation.getM22();
      double m22 = symmetricMatrixToRotate.getM20() * rotation.getM02() + symmetricMatrixToRotate.getM21() * rotation.getM12()
            + symmetricMatrixToRotate.getM22() * rotation.getM22();
      symmetricMatrixToRotate.set(m00, m01, m02, m01, m11, m12, m02, m12, m22);
   }
}
