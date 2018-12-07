package us.ihmc.mecano.spatial;

import static us.ihmc.robotics.Assert.*;

import java.util.Arrays;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MecanoTools;

public class SpatialInertiaTest
{
   private static final double EPSILON = 1.0e-12;
   private static final int ITERATIONS = 1000;

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(654);

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing applyInverseTransform against applyTransform
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame expressedInFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialInertia expected = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);
         SpatialInertia actual = new SpatialInertia(expected);
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);

         MecanoTestTools.assertSpatialInertiaEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testComputeKineticCoEnergy() throws Exception
   {
      Random random = new Random(334523);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test that the class is using the method from RigidBodyDynamicsTools.
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame expressedInFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, worldFrame, expressedInFrame);

         double expected = MecanoTools.computeKineticCoEnergy(inertia.getMomentOfInertia(), inertia.getMass(), inertia.getCenterOfMassOffset(),
                                                                         twist.getAngularPart(), twist.getLinearPart());
         double actual = inertia.computeKineticCoEnergy(twist);

         assertEquals(expected, actual, EPSILON);
      }

      {// Test exceptions:
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame expressedInFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame nonStationaryFrame = new ReferenceFrame("nonStationaryFrame", worldFrame, false, false)
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
            }
         };
         ReferenceFrame anotherFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, nonStationaryFrame, expressedInFrame);

         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeKineticCoEnergy(twist), RuntimeException.class);

         twist.setToZero(anotherFrame, worldFrame, expressedInFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeKineticCoEnergy(twist), ReferenceFrameMismatchException.class);

         twist.setToZero(bodyFrame, worldFrame, anotherFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeKineticCoEnergy(twist), ReferenceFrameMismatchException.class);

         twist.setToZero(bodyFrame, worldFrame, bodyFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeKineticCoEnergy(twist), ReferenceFrameMismatchException.class);
      }
   }

   @Test
   public void testComputeDynamicWrenchFast() throws Exception
   {
      Random random = new Random(345346);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test that the class is using the method from RigidBodyDynamicsTools.
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, bodyFrame);
         inertia.getCenterOfMassOffset().setToZero();
         SpatialAcceleration acceleration = MecanoRandomTools.nextSpatialAcceleration(random, bodyFrame, worldFrame, bodyFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, worldFrame, bodyFrame);

         Wrench expected = new Wrench(bodyFrame, bodyFrame);
         MecanoTools.computeDynamicMomentFast(inertia.getMomentOfInertia(), acceleration.getAngularPart(), twist.getAngularPart(),
                                                         expected.getAngularPart());
         MecanoTools.computeDynamicForceFast(inertia.getMass(), acceleration.getLinearPart(), twist.getAngularPart(), twist.getLinearPart(),
                                                        expected.getLinearPart());

         Wrench actual = new Wrench(worldFrame, worldFrame);
         inertia.computeDynamicWrenchFast(acceleration, twist, actual);

         MecanoTestTools.assertWrenchEquals(expected, actual, EPSILON);
      }

      { // Tests some exceptions
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame anotherFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame nonStationaryFrame = new ReferenceFrame("nonStationaryFrame", worldFrame, false, false)
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
            }
         };

         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, bodyFrame);
         SpatialAcceleration acceleration = MecanoRandomTools.nextSpatialAcceleration(random, bodyFrame, worldFrame, bodyFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, worldFrame, bodyFrame);

         Wrench wrench = new Wrench(worldFrame, worldFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeDynamicWrenchFast(acceleration, twist, wrench), RuntimeException.class);
         inertia.getCenterOfMassOffset().setToZero();
         inertia.computeDynamicWrenchFast(acceleration, twist, wrench);

         inertia.setReferenceFrame(anotherFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeDynamicWrenchFast(acceleration, twist, wrench),
                                                            ReferenceFrameMismatchException.class);
         inertia.setReferenceFrame(bodyFrame);
         inertia.computeDynamicWrenchFast(acceleration, twist, wrench);

         twist.setBaseFrame(nonStationaryFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeDynamicWrenchFast(acceleration, twist, wrench), RuntimeException.class);
         twist.setBaseFrame(worldFrame);
         inertia.computeDynamicWrenchFast(acceleration, twist, wrench);

         acceleration.setBaseFrame(nonStationaryFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeDynamicWrenchFast(acceleration, twist, wrench), RuntimeException.class);
         acceleration.setBaseFrame(worldFrame);
         inertia.computeDynamicWrenchFast(acceleration, twist, wrench);

         twist.setReferenceFrame(anotherFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeDynamicWrenchFast(acceleration, twist, wrench),
                                                            ReferenceFrameMismatchException.class);
         twist.setReferenceFrame(bodyFrame);
         inertia.computeDynamicWrenchFast(acceleration, twist, wrench);

         acceleration.setReferenceFrame(anotherFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeDynamicWrenchFast(acceleration, twist, wrench),
                                                            ReferenceFrameMismatchException.class);
         acceleration.setReferenceFrame(bodyFrame);
         inertia.computeDynamicWrenchFast(acceleration, twist, wrench);

         twist.setBodyFrame(anotherFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeDynamicWrenchFast(acceleration, twist, wrench),
                                                            ReferenceFrameMismatchException.class);
         twist.setBodyFrame(bodyFrame);
         inertia.computeDynamicWrenchFast(acceleration, twist, wrench);

         acceleration.setBodyFrame(anotherFrame);
         MecanoTestTools.assertExceptionIsThrown(() -> inertia.computeDynamicWrenchFast(acceleration, twist, wrench),
                                                            ReferenceFrameMismatchException.class);
         acceleration.setBodyFrame(bodyFrame);
         inertia.computeDynamicWrenchFast(acceleration, twist, wrench);
      }
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43);

      for (int i = 0; i < ITERATIONS; i++)
      {
         /*
          * This test uses the fact that the kinetic co-energy is frame invariant. So it effectively
          * allows to compare the changeFrame method for SpatialInertiaMatrix against the much
          * simpler changeFrame method for Twist.
          */
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame baseFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame finalFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, initialFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, baseFrame, initialFrame);

         assertTrue(inertia.getReferenceFrame() == initialFrame);

         double expected = computeKineticCoEnergy(twist, inertia);
         double initialMass = inertia.getMass();

         inertia.changeFrame(finalFrame);
         twist.changeFrame(finalFrame);

         assertTrue(inertia.getReferenceFrame() == finalFrame);

         double actual = computeKineticCoEnergy(twist, inertia);
         double finalMass = inertia.getMass();

         assertEquals(expected, actual, EPSILON);

         // Check that the mass does actually not change:
         assertEquals(initialMass, finalMass, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         /*
          * This test verifies Newton's law "F = ma" computed in different frames. To make things
          * easier, we assume zero velocity just acceleration.
          */
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame baseFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame finalFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         SpatialAcceleration acceleration = MecanoRandomTools.nextSpatialAcceleration(random, bodyFrame, baseFrame, initialFrame);
         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, initialFrame);
         Wrench expected = computeWrench(acceleration, inertia);

         acceleration.changeFrame(finalFrame);
         inertia.changeFrame(finalFrame);
         Wrench actual = computeWrench(acceleration, inertia);
         actual.changeFrame(initialFrame);

         MecanoTestTools.assertWrenchEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against the matrix equation using the adjoint.
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame finalFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, initialFrame, initialFrame);

         SimpleMatrix inertiaInitial = new SimpleMatrix(6, 6);
         inertia.get(inertiaInitial.getMatrix());

         RigidBodyTransform finalToInitial = finalFrame.getTransformToDesiredFrame(initialFrame);
         SimpleMatrix finalToInitialAdjoint = SimpleMatrix.wrap(adjoint(finalToInitial));
         SimpleMatrix finalToInitialAdjointTranspose = finalToInitialAdjoint.transpose();
         DenseMatrix64F expected = finalToInitialAdjointTranspose.mult(inertiaInitial).mult(finalToInitialAdjoint).getMatrix();

         inertia.changeFrame(finalFrame);
         DenseMatrix64F actual = new DenseMatrix64F(6, 6);
         inertia.get(actual);

         assertTrue(MatrixFeatures.isEquals(expected, actual, EPSILON));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test that a rotation does not change the center of mass offset.
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame finalFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("finalFrame", initialFrame,
                                                                                                       new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random),
                                                                                                                              new Point3D()));

         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, initialFrame, initialFrame);
         double expected = inertia.getCenterOfMassOffset().length();

         inertia.changeFrame(finalFrame);
         double actual = inertia.getCenterOfMassOffset().length();

         assertEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Sanity test on the moment of inertia
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame finalFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("finalFrame", initialFrame,
                                                                                                       new RigidBodyTransform(EuclidCoreRandomTools.nextQuaternion(random),
                                                                                                                              new Point3D()));

         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, initialFrame, initialFrame);
         inertia.getMomentOfInertia().set(MecanoRandomTools.nextSymmetricPositiveDefiniteMatrix3D(random));
         inertia.getCenterOfMassOffset().setToZero();

         Matrix3D initialMomentOfInertia = new Matrix3D(inertia.getMomentOfInertia());
         inertia.changeFrame(finalFrame);
         Matrix3D finalMomentOfInertia = new Matrix3D(inertia.getMomentOfInertia());
         assertEigenValuesPositiveAndEqual(initialMomentOfInertia, finalMomentOfInertia, EPSILON);
      }
   }

   private static void assertEigenValuesPositiveAndEqual(Matrix3D matrix1, Matrix3D matrix2, double epsilon)
   {
      DenseMatrix64F denseMatrix1 = new DenseMatrix64F(3, 3);
      matrix1.get(denseMatrix1);

      DenseMatrix64F denseMatrix2 = new DenseMatrix64F(3, 3);
      matrix2.get(denseMatrix2);

      EigenDecomposition<DenseMatrix64F> eig1 = DecompositionFactory.eig(3, false);
      eig1.decompose(denseMatrix1);
      double[] eigRealParts1 = new double[3];
      for (int i = 0; i < eig1.getNumberOfEigenvalues(); i++)
         eigRealParts1[i] = eig1.getEigenvalue(i).getReal();
      Arrays.sort(eigRealParts1);

      EigenDecomposition<DenseMatrix64F> eig2 = DecompositionFactory.eig(3, false);
      eig2.decompose(denseMatrix2);
      double[] eigRealParts2 = new double[3];
      for (int i = 0; i < eig2.getNumberOfEigenvalues(); i++)
         eigRealParts2[i] = eig2.getEigenvalue(i).getReal();
      Arrays.sort(eigRealParts2);

      for (int i = 0; i < eig1.getNumberOfEigenvalues(); i++)
      {
         assertEquals(0.0, eig1.getEigenvalue(i).getImaginary(), epsilon);
         assertEquals(0.0, eig2.getEigenvalue(i).getImaginary(), epsilon);
         assertEquals(eigRealParts2[0], eigRealParts2[0], epsilon);
      }
   }

   /**
    * The kinetic co-energy U is calculated as follows:
    * 
    * <pre>
    * U = 1/2 T<sup>T</sup> I T
    * </pre>
    * 
    * where T is a twist and I a spatial inertia matrix.
    */
   private static double computeKineticCoEnergy(Twist twist, SpatialInertia inertia)
   {
      DenseMatrix64F twistMatrix = new DenseMatrix64F(6, 1);
      DenseMatrix64F inertiaMatrix = new DenseMatrix64F(6, 6);
      twist.get(twistMatrix);
      inertia.get(inertiaMatrix);

      DenseMatrix64F intermediate = new DenseMatrix64F(1, 6);
      CommonOps.multTransA(twistMatrix, inertiaMatrix, intermediate);

      DenseMatrix64F result = new DenseMatrix64F(1, 1);
      CommonOps.mult(intermediate, twistMatrix, result);

      return 0.5 * result.get(0);
   }

   private static Wrench computeWrench(SpatialAcceleration acceleration, SpatialInertia inertia)
   {
      acceleration.getReferenceFrame().checkReferenceFrameMatch(inertia.getReferenceFrame());

      DenseMatrix64F accelerationMatrix = new DenseMatrix64F(6, 1);
      DenseMatrix64F inertiaMatrix = new DenseMatrix64F(6, 6);

      acceleration.get(accelerationMatrix);
      inertia.get(inertiaMatrix);

      DenseMatrix64F wrenchMatrix = new DenseMatrix64F(6, 1);
      CommonOps.mult(inertiaMatrix, accelerationMatrix, wrenchMatrix);
      return new Wrench(acceleration.getBodyFrame(), acceleration.getReferenceFrame(), wrenchMatrix);
   }

   private static DenseMatrix64F adjoint(RigidBodyTransform transform)
   {
      Matrix3D translationTilde = new Matrix3D();
      translationTilde.setToTildeForm(transform.getTranslationVector());
      DenseMatrix64F translationTildeDense = new DenseMatrix64F(3, 3);
      translationTilde.get(translationTildeDense);

      DenseMatrix64F rotationDense = new DenseMatrix64F(3, 3);
      transform.getRotationMatrix().get(rotationDense);

      DenseMatrix64F adjointMatrix = new DenseMatrix64F(6, 6);

      // upper left:
      CommonOps.insert(rotationDense, adjointMatrix, 0, 0);

      // lower left:

      DenseMatrix64F lowerLeft = new DenseMatrix64F(translationTildeDense.getNumRows(), rotationDense.getNumCols());
      CommonOps.mult(translationTildeDense, rotationDense, lowerLeft);
      CommonOps.insert(lowerLeft, adjointMatrix, 3, 0);

      // lower right:
      CommonOps.insert(rotationDense, adjointMatrix, 3, 3);

      return adjointMatrix;
   }
}
