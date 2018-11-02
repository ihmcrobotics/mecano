package us.ihmc.mecano.algorithms;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * This class provides tools for the articulated-body inertia algorithm.
 * 
 * @author Sylvain Bertrand
 */
public class ArticulatedBodyInertiaAlorigthmTools
{

   /**
    * Applies a translation to the given moment of inertia.
    * <p>
    * Compared to
    * {@link MecanoTools#translateMomentOfInertia(double, Tuple3DReadOnly, boolean, Tuple3DReadOnly, Matrix3DBasics)}
    * this method doesn't assume the linear inertia and cross inertia to have properties allowing
    * for simplification of the transformation.
    * </p>
    * 
    * @param negateTranslation whether to negate the translation before transforming the angular inertia.
    * @param translation the translation to be applied. Not modified.
    * @param linearInertia the corresponding linear part of the inertia. Not modified.
    * @param crossInertia the corresponding cross part of the inertia. Not modified.
    * @param angularInertiaToTranslate the angular part to translate. Modified.
    */
   public static void translateAngularInertia(boolean negateTranslation, Tuple3DReadOnly translation, Matrix3DReadOnly linearInertia,
                                              Matrix3DReadOnly crossInertia, Matrix3DBasics angularInertiaToTranslate)
   {
      double x, y, z;
      if (negateTranslation)
      {
         x = -translation.getX();
         y = -translation.getY();
         z = -translation.getZ();
      }
      else
      {
         x = translation.getX();
         y = translation.getY();
         z = translation.getZ();
      }

      double xx = x * x;
      double yy = y * y;
      double zz = z * z;

      double xy = x * y;
      double xz = x * z;
      double yz = y * z;

      double mxx = linearInertia.getM00();
      double myy = linearInertia.getM11();
      double mzz = linearInertia.getM22();

      double mxy = linearInertia.getM01();
      double mxz = linearInertia.getM02();
      double myz = linearInertia.getM12();

      double ixx = angularInertiaToTranslate.getM00();
      double iyy = angularInertiaToTranslate.getM11();
      double izz = angularInertiaToTranslate.getM22();

      double ixy = angularInertiaToTranslate.getM01();
      double ixz = angularInertiaToTranslate.getM02();
      double iyz = angularInertiaToTranslate.getM12();

      double c00 = crossInertia.getM00();
      double c01 = crossInertia.getM01();
      double c02 = crossInertia.getM02();
      double c10 = crossInertia.getM10();
      double c11 = crossInertia.getM11();
      double c12 = crossInertia.getM12();
      double c20 = crossInertia.getM20();
      double c21 = crossInertia.getM21();
      double c22 = crossInertia.getM22();

      // Step 1: add the translated linear inertia:
      ixx += yy * mzz + zz * myy - 2.0 * yz * myz;
      iyy += xx * mzz + zz * mxx - 2.0 * xz * mxz;
      izz += xx * myy + yy * mxx - 2.0 * xy * mxy;

      ixy += -zz * mxy - xy * mzz + xz * myz + yz * mxz;
      ixz += -yy * mxz + xy * myz - xz * myy + yz * mxy;
      iyz += -xx * myz + xy * mxz + xz * mxy - yz * mxx;

      // Step 2: add the translated cross inertia:
      ixx += 2.0 * (y * c02 - z * c01);
      iyy += 2.0 * (-x * c12 + z * c10);
      izz += 2.0 * (x * c21 - y * c20);

      ixy += -x * c02 + y * c12 + z * (c00 - c11);
      ixz += x * c01 - y * (c00 - c22) - z * c21;
      iyz += x * (c11 - c22) - y * c10 + z * c20;

      angularInertiaToTranslate.set(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz);
   }

   /**
    * Applies a translation to the cross part of a complex inertia.
    * 
    * @param negateTranslation whether to negate the translation before transforming the cross inertia.
    * @param translation the translation to be applied. Not modified.
    * @param linearInertia the corresponding linear part of the inertia. Not modified.
    * @param crossInertiaToTranslate the cross inertia to transform. Modified.
    */
   public static void translateCrossInertia(boolean negateTranslation, Tuple3DReadOnly translation, Matrix3DReadOnly linearInertia,
                                            Matrix3DBasics crossInertiaToTranslate)
   {
      double x, y, z;
      if (negateTranslation)
      {
         x = -translation.getX();
         y = -translation.getY();
         z = -translation.getZ();
      }
      else
      {
         x = translation.getX();
         y = translation.getY();
         z = translation.getZ();
      }

      double mxx = linearInertia.getM00();
      double myy = linearInertia.getM11();
      double mzz = linearInertia.getM22();

      double mxy = linearInertia.getM01();
      double mxz = linearInertia.getM02();
      double myz = linearInertia.getM12();

      double c00 = crossInertiaToTranslate.getM00();
      double c01 = crossInertiaToTranslate.getM01();
      double c02 = crossInertiaToTranslate.getM02();

      double c10 = crossInertiaToTranslate.getM10();
      double c11 = crossInertiaToTranslate.getM11();
      double c12 = crossInertiaToTranslate.getM12();

      double c20 = crossInertiaToTranslate.getM20();
      double c21 = crossInertiaToTranslate.getM21();
      double c22 = crossInertiaToTranslate.getM22();

      c00 += y * mxz - z * mxy;
      c11 += -x * myz + z * mxy;
      c22 += x * myz - y * mxz;

      c01 += y * myz - z * myy;
      c02 += y * mzz - z * myz;
      c12 += -x * mzz + z * mxz;

      c10 += -x * mxz + z * mxx;
      c20 += x * mxy - y * mxx;
      c21 += x * myy - y * mxy;

      crossInertiaToTranslate.set(c00, c01, c02, c10, c11, c12, c20, c21, c22);
   }
}
