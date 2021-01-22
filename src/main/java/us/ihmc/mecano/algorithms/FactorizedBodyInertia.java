package us.ihmc.mecano.algorithms;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Factorized body inertia is a 6-by-6 matrix that is used to represent:
 *
 * <pre>
 *     / &omega;&times; v&times; \
 * B = |       | I
 *     \ 0  &omega;&times; /
 * </pre>
 *
 * where <tt>&omega;</tt> and <tt>v</tt> are the angular and linear velocities of a rigid-body,
 * <tt>I</tt> is the spatial inertia of the same rigid-body, and <tt>(.)&times;</tt> is the mapping
 * from vector to 3-by-3 matrix as follows:
 *
 * <pre>
 * / x \    /  0 -z  y \
 * | y |&times; = |  z  0 -x |
 * \ z /    \ -y  x  0 /
 * </pre>
 * <p>
 * This class is used as part of {@link CompositeRigidBodyMassMatrixCalculator} for calculating the
 * Coriolis and centrifugal matrix.
 * </p>
 *
 * @author Sylvain Bertrand
 */
class FactorizedBodyInertia implements GeometryObject<FactorizedBodyInertia>, ReferenceFrameHolder, FrameChangeable
{
   private ReferenceFrame referenceFrame = null;
   private final Matrix3D angularInertia = new Matrix3D();
   private final Matrix3D linearInertia = new Matrix3D();
   private final Matrix3D topRightInertia = new Matrix3D();
   private final Matrix3D bottomLeftInertia = new Matrix3D();

   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   public FactorizedBodyInertia()
   {
   }

   public FactorizedBodyInertia(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public boolean containsNaN()
   {
      return angularInertia.containsNaN() || linearInertia.containsNaN() || topRightInertia.containsNaN() || bottomLeftInertia.containsNaN();
   }

   @Override
   public void setToNaN()
   {
      angularInertia.setToNaN();
      linearInertia.setToNaN();
      topRightInertia.setToNaN();
      bottomLeftInertia.setToNaN();
   }

   public void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   @Override
   public void setToZero()
   {
      angularInertia.setToZero();
      linearInertia.setToZero();
      topRightInertia.setToZero();
      bottomLeftInertia.setToZero();
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   @Override
   public void set(FactorizedBodyInertia other)
   {
      checkReferenceFrameMatch(other);
      angularInertia.set(other.angularInertia);
      linearInertia.set(other.linearInertia);
      topRightInertia.set(other.topRightInertia);
      bottomLeftInertia.set(other.bottomLeftInertia);
   }

   public void setIncludingFrame(FactorizedBodyInertia other)
   {
      setReferenceFrame(other.referenceFrame);
      angularInertia.set(other.angularInertia);
      linearInertia.set(other.linearInertia);
      topRightInertia.set(other.topRightInertia);
      bottomLeftInertia.set(other.bottomLeftInertia);
   }

   public void setIncludingFrame(SpatialInertiaReadOnly spatialInertia, TwistReadOnly bodyTwist)
   {
      spatialInertia.checkBodyFrameMatch(bodyTwist.getBodyFrame());
      spatialInertia.checkReferenceFrameMatch(bodyTwist.getReferenceFrame());
      setReferenceFrame(spatialInertia.getReferenceFrame());

      // w x J - m v x c x
      tildeTimesTilde(bodyTwist.getLinearPart(), spatialInertia.getCenterOfMassOffset(), angularInertia);
      angularInertia.scale(-spatialInertia.getMass());
      addTildeTimesMatrix(bodyTwist.getAngularPart(), spatialInertia.getMomentOfInertia(), angularInertia);

      // -m w x c x
      tildeTimesTilde(bodyTwist.getAngularPart(), spatialInertia.getCenterOfMassOffset(), bottomLeftInertia);
      bottomLeftInertia.scale(-spatialInertia.getMass());

      // m v x + m w x c x
      topRightInertia.setToTildeForm(bodyTwist.getLinearPart());
      topRightInertia.scale(spatialInertia.getMass());
      topRightInertia.sub(bottomLeftInertia);

      // m w x
      linearInertia.setToTildeForm(bodyTwist.getAngularPart());
      linearInertia.scale(spatialInertia.getMass());
   }

   public void add(FactorizedBodyInertia other)
   {
      checkReferenceFrameMatch(other);
      angularInertia.add(other.angularInertia);
      linearInertia.add(other.linearInertia);
      topRightInertia.add(other.topRightInertia);
      bottomLeftInertia.add(other.bottomLeftInertia);
   }

   public void add(DMatrix matrix)
   {
      MecanoTools.addEquals(0, 0, matrix, angularInertia);
      MecanoTools.addEquals(3, 3, matrix, linearInertia);
      MecanoTools.addEquals(0, 3, matrix, topRightInertia);
      MecanoTools.addEquals(3, 0, matrix, bottomLeftInertia);
   }

   public void sub(FactorizedBodyInertia other)
   {
      checkReferenceFrameMatch(other);
      angularInertia.sub(other.angularInertia);
      linearInertia.sub(other.linearInertia);
      topRightInertia.sub(other.topRightInertia);
      bottomLeftInertia.sub(other.bottomLeftInertia);
   }

   public void sub(DMatrix matrix)
   {
      MecanoTools.subEquals(0, 0, matrix, angularInertia);
      MecanoTools.subEquals(3, 3, matrix, linearInertia);
      MecanoTools.subEquals(0, 3, matrix, topRightInertia);
      MecanoTools.subEquals(3, 0, matrix, bottomLeftInertia);
   }

   /** {@inheritDoc} */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == getReferenceFrame())
         return;

      getReferenceFrame().getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      setReferenceFrame(desiredFrame);
   }

   public void transform(SpatialVectorReadOnly vectorOriginal, SpatialVectorBasics vectorTransformed)
   {
      if (vectorOriginal == vectorTransformed)
         throw new UnsupportedOperationException("In-place transformation is not supported.");

      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(referenceFrame);
      angularInertia.transform(vectorOriginal.getAngularPart(), vectorTransformed.getAngularPart());
      topRightInertia.addTransform(vectorOriginal.getLinearPart(), vectorTransformed.getAngularPart());

      bottomLeftInertia.transform(vectorOriginal.getAngularPart(), vectorTransformed.getLinearPart());
      linearInertia.addTransform(vectorOriginal.getLinearPart(), vectorTransformed.getLinearPart());
   }

   public void addTransform(SpatialVectorReadOnly vectorOriginal, SpatialVectorBasics vectorTransformed)
   {
      if (vectorOriginal == vectorTransformed)
         throw new UnsupportedOperationException("In-place transformation is not supported.");

      checkReferenceFrameMatch(vectorOriginal);
      checkReferenceFrameMatch(vectorTransformed);
      angularInertia.addTransform(vectorOriginal.getAngularPart(), vectorTransformed.getAngularPart());
      topRightInertia.addTransform(vectorOriginal.getLinearPart(), vectorTransformed.getAngularPart());

      bottomLeftInertia.addTransform(vectorOriginal.getAngularPart(), vectorTransformed.getLinearPart());
      linearInertia.addTransform(vectorOriginal.getLinearPart(), vectorTransformed.getLinearPart());
   }

   public void transposeTransform(SpatialVectorReadOnly vectorOriginal, SpatialVectorBasics vectorTransformed)
   {
      if (vectorOriginal == vectorTransformed)
         throw new UnsupportedOperationException("In-place transformation is not supported.");

      checkReferenceFrameMatch(vectorOriginal);
      vectorTransformed.setReferenceFrame(referenceFrame);
      transposeTransform(angularInertia, vectorOriginal.getAngularPart(), vectorTransformed.getAngularPart());
      addTransposeTransform(bottomLeftInertia, vectorOriginal.getLinearPart(), vectorTransformed.getAngularPart());

      transposeTransform(topRightInertia, vectorOriginal.getAngularPart(), vectorTransformed.getLinearPart());
      addTransposeTransform(linearInertia, vectorOriginal.getLinearPart(), vectorTransformed.getLinearPart());
   }

   static void transposeTransform(Matrix3DReadOnly matrix, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM10() * tupleOriginal.getY() + matrix.getM20() * tupleOriginal.getZ();
      double y = matrix.getM01() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY() + matrix.getM21() * tupleOriginal.getZ();
      double z = matrix.getM02() * tupleOriginal.getX() + matrix.getM12() * tupleOriginal.getY() + matrix.getM22() * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   static void addTransposeTransform(Matrix3DReadOnly matrix, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM10() * tupleOriginal.getY() + matrix.getM20() * tupleOriginal.getZ();
      double y = matrix.getM01() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY() + matrix.getM21() * tupleOriginal.getZ();
      double z = matrix.getM02() * tupleOriginal.getX() + matrix.getM12() * tupleOriginal.getY() + matrix.getM22() * tupleOriginal.getZ();
      tupleTransformed.add(x, y, z);
   }

   public Matrix3D getAngularInertia()
   {
      return angularInertia;
   }

   public Matrix3D getLinearInertia()
   {
      return linearInertia;
   }

   public Matrix3D getTopRightInertia()
   {
      return topRightInertia;
   }

   public Matrix3D getBottomLeftInertia()
   {
      return bottomLeftInertia;
   }

   public void get(DMatrix matrixToPack)
   {
      angularInertia.get(0, 0, matrixToPack);
      linearInertia.get(3, 3, matrixToPack);
      topRightInertia.get(0, 3, matrixToPack);
      bottomLeftInertia.get(3, 0, matrixToPack);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransform)
         applyTransform((RigidBodyTransform) transform);
      else
         throw new UnsupportedOperationException("The feature applyTransform is not supported for the transform of the type: "
               + transform.getClass().getSimpleName());
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      if (transform instanceof RigidBodyTransform)
         applyInverseTransform((RigidBodyTransform) transform);
      else
         throw new UnsupportedOperationException("The feature applyInverseTransform is not supported for the transform of the type: "
               + transform.getClass().getSimpleName());
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      if (transform.hasRotation())
      {
         transform.getRotation().transform(angularInertia);
         transform.getRotation().transform(linearInertia);
         transform.getRotation().transform(topRightInertia);
         transform.getRotation().transform(bottomLeftInertia);
      }

      if (transform.hasTranslation())
      {
         addTildeTimesMatrix(transform.getTranslation(), linearInertia, topRightInertia);
         addTildeTimesMatrix(transform.getTranslation(), bottomLeftInertia, angularInertia);
         subMatrixTimesTilde(topRightInertia, transform.getTranslation(), angularInertia);
         subMatrixTimesTilde(linearInertia, transform.getTranslation(), bottomLeftInertia);
      }
   }

   public void applyInverseTransform(RigidBodyTransform transform)
   {
      // TODO Evaluate inverse transform w/o inverting the transform.
      transformToDesiredFrame.setAndInvert(transform);
      applyTransform(transformToDesiredFrame);
   }

   /**
    * <pre>
    *                  /  0 -z  y \
    * resultToPack +=  |  z  0 -x | * m
    *                  \ -y  x  0 /
    * </pre>
    */
   static void addTildeTimesMatrix(Tuple3DReadOnly p, Matrix3DReadOnly m, Matrix3DBasics resultToPack)
   {
      double c00 = -p.getZ() * m.getM10() + p.getY() * m.getM20();
      double c01 = -p.getZ() * m.getM11() + p.getY() * m.getM21();
      double c02 = -p.getZ() * m.getM12() + p.getY() * m.getM22();
      double c10 = p.getZ() * m.getM00() - p.getX() * m.getM20();
      double c11 = p.getZ() * m.getM01() - p.getX() * m.getM21();
      double c12 = p.getZ() * m.getM02() - p.getX() * m.getM22();
      double c20 = -p.getY() * m.getM00() + p.getX() * m.getM10();
      double c21 = -p.getY() * m.getM01() + p.getX() * m.getM11();
      double c22 = -p.getY() * m.getM02() + p.getX() * m.getM12();

      resultToPack.addM00(c00);
      resultToPack.addM01(c01);
      resultToPack.addM02(c02);
      resultToPack.addM10(c10);
      resultToPack.addM11(c11);
      resultToPack.addM12(c12);
      resultToPack.addM20(c20);
      resultToPack.addM21(c21);
      resultToPack.addM22(c22);
   }

   /**
    * <pre>
    *                      /  0 -z  y \
    * resultToPack -=  m * |  z  0 -x |
    *                      \ -y  x  0 /
    * </pre>
    */
   static void subMatrixTimesTilde(Matrix3DReadOnly m, Tuple3DReadOnly p, Matrix3DBasics resultToPack)
   {
      double c00 = m.getM01() * p.getZ() - m.getM02() * p.getY();
      double c01 = m.getM02() * p.getX() - m.getM00() * p.getZ();
      double c02 = m.getM00() * p.getY() - m.getM01() * p.getX();
      double c10 = m.getM11() * p.getZ() - m.getM12() * p.getY();
      double c11 = m.getM12() * p.getX() - m.getM10() * p.getZ();
      double c12 = m.getM10() * p.getY() - m.getM11() * p.getX();
      double c20 = m.getM21() * p.getZ() - m.getM22() * p.getY();
      double c21 = m.getM22() * p.getX() - m.getM20() * p.getZ();
      double c22 = m.getM20() * p.getY() - m.getM21() * p.getX();

      resultToPack.subM00(c00);
      resultToPack.subM01(c01);
      resultToPack.subM02(c02);
      resultToPack.subM10(c10);
      resultToPack.subM11(c11);
      resultToPack.subM12(c12);
      resultToPack.subM20(c20);
      resultToPack.subM21(c21);
      resultToPack.subM22(c22);
   }

   /**
    * <pre>
    *                  /   0 -z1  y1 \   /   0 -z2  y2 \
    * resultToPack -=  |  z1   0 -x1 | * |  z2   0 -x2 |
    *                  \ -y1  x1   0 /   \ -y2  x2   0 /
    * </pre>
    */
   static void tildeTimesTilde(Tuple3DReadOnly p1, Tuple3DReadOnly p2, Matrix3DBasics resultToPack)
   {
      double c00 = -p1.getZ() * p2.getZ() - p1.getY() * p2.getY();
      double c01 = p1.getY() * p2.getX();
      double c02 = p1.getZ() * p2.getX();
      double c10 = p1.getX() * p2.getY();
      double c11 = -p1.getZ() * p2.getZ() - p1.getX() * p2.getX();
      double c12 = p1.getZ() * p2.getY();
      double c20 = p1.getX() * p2.getZ();
      double c21 = p1.getY() * p2.getZ();
      double c22 = -p1.getY() * p2.getY() - p1.getX() * p2.getX();
      resultToPack.set(c00, c01, c02, c10, c11, c12, c20, c21, c22);
   }

   @Override
   public boolean epsilonEquals(FactorizedBodyInertia other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!getAngularInertia().epsilonEquals(other.getAngularInertia(), epsilon))
         return false;
      if (!getLinearInertia().epsilonEquals(other.getLinearInertia(), epsilon))
         return false;
      if (!getTopRightInertia().epsilonEquals(other.getTopRightInertia(), epsilon))
         return false;
      if (!getBottomLeftInertia().epsilonEquals(other.getBottomLeftInertia(), epsilon))
         return false;
      return true;
   }

   @Override
   public boolean geometricallyEquals(FactorizedBodyInertia other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return epsilonEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      DMatrixRMaj matrix = new DMatrixRMaj(6, 6);
      get(matrix);
      return "Articulated-body inertia expressed in " + referenceFrame + "\n" + MecanoIOTools.getDMatrixString(matrix);
   }
}
