package us.ihmc.mecano.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrix3x3;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;

public class MecanoToolsTest
{
   private static final double EPSILON = 1.0e-12;
   private static final int ITERATIONS = 1000;

   @Test
   public void testStringMethod()
   { // Test empty input
      String emptyString = new String();
      assertEquals(emptyString, MecanoTools.capitalize(emptyString));
      assertEquals(null, MecanoTools.capitalize(null));

      int stringSize = 10;
      String str = new String();
      Random random = new Random(1243);
      int randomInt = random.nextInt();
      for (int i = 0; i < ITERATIONS; ++i)
      { // Test capitalization
         randomInt = random.nextInt();
         boolean upper = (randomInt % 2 == 0) ? true : false;
         char c;
         for (int j = 0; j < stringSize; ++j)
         {
            randomInt = random.nextInt();
            if (true == upper)
               c = (char) ((int) 'A' + randomInt % 26);
            else
               c = (char) ((int) 'a' + randomInt % 26);
            str += c;
         }
         if (true == upper)
         { // Should return original string
            assertEquals(str, MecanoTools.capitalize(str));
         }
         else
         { // Should return first letter capitalized string
            assertEquals(MecanoTools.capitalize(str), Character.toUpperCase(str.charAt(0)) + str.substring(1, str.length()));
         }
         str = "";
      }
   }

   @Test
   public void testMatrix3DChecks()
   {
      Random random = new Random(1234);
      for (int i = 0; i < ITERATIONS; ++i)
      { // Test symmetric matrix boolean feature
         double a = random.nextDouble();
         double b = random.nextDouble();
         double c = random.nextDouble();
         Matrix3D symmetricMatrix3D = EuclidCoreRandomTools.nextMatrix3D(random);
         assertFalse(MecanoTools.isMatrix3DSymmetric(symmetricMatrix3D, EPSILON));
         symmetricMatrix3D.setM01(a);
         symmetricMatrix3D.setM10(a);
         assertFalse(MecanoTools.isMatrix3DSymmetric(symmetricMatrix3D, EPSILON));
         symmetricMatrix3D.setM02(b);
         symmetricMatrix3D.setM20(b);
         assertFalse(MecanoTools.isMatrix3DSymmetric(symmetricMatrix3D, EPSILON));
         symmetricMatrix3D.setM12(c);
         symmetricMatrix3D.setM21(c);
         assertTrue(MecanoTools.isMatrix3DSymmetric(symmetricMatrix3D, EPSILON));

         // Test diagonal matrix boolean feature
         int randomInt = random.nextInt();
         Matrix3D diagonalMatrix3D = EuclidCoreRandomTools.nextMatrix3D(random);
         boolean diagonal = randomInt % 2 == 0 ? true : false;
         if (true == diagonal)
         {
            diagonalMatrix3D.setM01(0);
            diagonalMatrix3D.setM02(0);
            diagonalMatrix3D.setM12(0);
            diagonalMatrix3D.setM10(0);
            diagonalMatrix3D.setM20(0);
            diagonalMatrix3D.setM21(0);
            assertTrue(MecanoTools.isMatrix3DDiagonal(diagonalMatrix3D, EPSILON));
         }
         else
         {
            assertFalse(MecanoTools.isMatrix3DDiagonal(diagonalMatrix3D, EPSILON));
         }

      }
   }

   @Test
   public void testTildeForm()
   {
      Random random = new Random(5432);
      int min = -100000;
      int max = 100000;
      int dimension = 0;
      for (int i = 0; i < ITERATIONS; ++i)
      { // Test vector - > Skew Symmetric Matrix (tilde form for use in cross product)
        // Let's test this by asserting C = A X B == tilde(A) * B
        // Also use random size matrix as input and random position to put the resulting sub-matrix in it.
         Vector3D vector3D_A = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D vector3D_B = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D expectedCrossProduct = new Vector3D();
         expectedCrossProduct.cross(vector3D_A, vector3D_B);
         int randomInt = random.nextInt();
         switch (randomInt % 4)
         {
            case 0:
               dimension = 3;
            case 1:
               dimension = 4;
            case 2:
               dimension = 5;
            case 3:
               dimension = 6;
         }
         DMatrixRMaj random_matrix = EuclidCoreRandomTools.nextDMatrixRMaj(random, dimension, dimension, min, max);
         int row = Math.abs(randomInt) % (dimension - 2);
         randomInt = random.nextInt();
         int col = Math.abs(randomInt) % (dimension - 2);
         MecanoTools.toTildeForm(vector3D_A, row, col, random_matrix);
         DMatrixRMaj tildeForm_A = CommonOps_DDRM.extract(random_matrix, row, row + 3, col, col + 3);

         Vector3D toBeTestedCrossProduct = new Vector3D(tildeForm_A.get(0, 0) * vector3D_B.getX() + tildeForm_A.get(0, 1) * vector3D_B.getY()
               + tildeForm_A.get(0, 2) * vector3D_B.getZ(),
                                                        tildeForm_A.get(1, 0) * vector3D_B.getX() + tildeForm_A.get(1, 1) * vector3D_B.getY()
                                                              + tildeForm_A.get(1, 2) * vector3D_B.getZ(),
                                                        tildeForm_A.get(2, 0) * vector3D_B.getX() + tildeForm_A.get(2, 1) * vector3D_B.getY()
                                                              + tildeForm_A.get(2, 2) * vector3D_B.getZ());
         assertEquals(expectedCrossProduct, toBeTestedCrossProduct);
      }
   }

   @Test
   public void testAddSubEquals()
   {
      Random random = new Random(1234);
      for (int i = 0; i < ITERATIONS; ++i)
      {
         // set up matrices for testing.
         Matrix3D operatorMatrix3D = EuclidCoreRandomTools.nextMatrix3D(random);
         DMatrix3x3 operatorMatrix3x3 = new DMatrix3x3();
         operatorMatrix3D.get(operatorMatrix3x3);
         Matrix3D expectedMatrix3D = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D toBeTestedMatrix3D = new Matrix3D(expectedMatrix3D);

         // Test addEquals method in MecanoTools.
         expectedMatrix3D.add(operatorMatrix3D);
         MecanoTools.addEquals(0, 0, operatorMatrix3x3, toBeTestedMatrix3D);
         assertEquals(expectedMatrix3D, toBeTestedMatrix3D);

         // Test subEquals method in MecanoTools.

         expectedMatrix3D.sub((operatorMatrix3D));
         MecanoTools.subEquals(0, 0, operatorMatrix3x3, toBeTestedMatrix3D);
         assertEquals(expectedMatrix3D, toBeTestedMatrix3D);

      }
   }

   @Test
   public void testTranslateMomentOfInertia()
   {
      Random random = new Random(2342);

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's test translating one way and then back:
         double mass = random.nextDouble();
         Vector3D centerOfMass = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         Matrix3D inertiaOriginal = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3D expected = new Matrix3D(inertiaOriginal);

         Matrix3D actual = new Matrix3D(inertiaOriginal);
         MecanoTools.translateMomentOfInertia(mass, centerOfMass, false, translation, actual);
         centerOfMass.add(translation);
         translation.negate();
         MecanoTools.translateMomentOfInertia(mass, centerOfMass, false, translation, actual);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's test against the matrix expression:
         double mass = random.nextDouble();
         Vector3D centerOfMass = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         Matrix3D inertiaOriginal = EuclidCoreRandomTools.nextMatrix3D(random);

         Matrix3D expected = new Matrix3D();
         {
            Matrix3D cTilde = new Matrix3D();
            cTilde.setToTildeForm(centerOfMass);
            Matrix3D pTilde = new Matrix3D();
            pTilde.setToTildeForm(translation);

            Matrix3D cTilde_pTilde = new Matrix3D(cTilde);
            cTilde_pTilde.multiply(pTilde);

            Matrix3D pTilde_cTilde = new Matrix3D(pTilde);
            pTilde_cTilde.multiply(cTilde);

            Matrix3D pTilde_pTilde = new Matrix3D(pTilde);
            pTilde_pTilde.multiply(pTilde);

            expected.set(cTilde_pTilde);
            expected.add(pTilde_cTilde);
            expected.add(pTilde_pTilde);
            expected.scale(-mass);
            expected.add(inertiaOriginal);
         }

         Matrix3D actual = new Matrix3D(inertiaOriginal);
         MecanoTools.translateMomentOfInertia(mass, centerOfMass, false, translation, actual);

         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; ++i)
      { // Test negate translation.
         double mass = random.nextDouble();
         Vector3D centerOfMass = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D translation = EuclidCoreRandomTools.nextVector3D(random);
         Matrix3D inertiaOriginal = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D pre_negatedTensor = new Matrix3D(inertiaOriginal); // EXPECTED
         Matrix3D negationTestTensor = new Matrix3D(inertiaOriginal); // TO BE TESTED
         MecanoTools.translateMomentOfInertia(mass, centerOfMass, true, translation, negationTestTensor);
         translation.negate();
         MecanoTools.translateMomentOfInertia(mass, centerOfMass, false, translation, pre_negatedTensor);
         EuclidCoreTestTools.assertMatrix3DEquals(pre_negatedTensor, negationTestTensor, EPSILON);
      }
   }

   @Test
   public void testComputeDynamicForce() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's compare computeDynamicForce(...) against
        // computeDynamicForceFast(...)
         double mass = random.nextDouble();
         Vector3D centerOfMassOffset = new Vector3D(); // It has to be zero for the comparison to be relevant.
         Vector3D angularAcceleration = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D linearAcceleration = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D linearVelocity = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D dynamicForce1 = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D dynamicForce2 = EuclidCoreRandomTools.nextVector3D(random);

         MecanoTools.computeDynamicForce(mass, centerOfMassOffset, angularAcceleration, linearAcceleration, angularVelocity, linearVelocity, dynamicForce1);
         MecanoTools.computeDynamicForceFast(mass, linearAcceleration, angularVelocity, linearVelocity, dynamicForce2);

         EuclidCoreTestTools.assertEquals(dynamicForce1, dynamicForce2, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's assert that when the acceleration & velocity are zero, the
        // resulting force is also zero
         double mass = random.nextDouble();
         // This should not affect the result.
         Vector3D centerOfMassOffset = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D angularAcceleration = new Vector3D();
         Vector3D linearAcceleration = new Vector3D();
         Vector3D angularVelocity = new Vector3D();
         Vector3D linearVelocity = new Vector3D();
         Vector3D expected = new Vector3D();
         Vector3D dynamicForce1 = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D dynamicForce2 = EuclidCoreRandomTools.nextVector3D(random);

         MecanoTools.computeDynamicForce(mass, centerOfMassOffset, angularAcceleration, linearAcceleration, angularVelocity, linearVelocity, dynamicForce1);
         MecanoTools.computeDynamicForceFast(mass, linearAcceleration, angularVelocity, linearVelocity, dynamicForce2);

         EuclidCoreTestTools.assertEquals(expected, dynamicForce1, EPSILON);
         EuclidCoreTestTools.assertEquals(expected, dynamicForce2, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's assert that when the acceleration is zero, the resulting force
        // is also zero
         double mass = random.nextDouble();
         // This should not affect the result.
         Vector3D centerOfMassOffset = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D angularAcceleration = new Vector3D();
         Vector3D linearAcceleration = new Vector3D();
         Vector3D angularVelocity = new Vector3D();
         // Rotate around the CoM offset axis to prevent generating centrifugal forces
         angularVelocity.setAndScale(EuclidCoreRandomTools.nextDouble(random, 10.0), centerOfMassOffset);
         Vector3D linearVelocity = new Vector3D();
         // By aligning the angular and linear velocities we prevent generating Coriolis
         // accelerations
         linearVelocity.setAndScale(EuclidCoreRandomTools.nextDouble(random, 10.0), angularVelocity);
         Vector3D expected = new Vector3D();
         Vector3D dynamicForce1 = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D dynamicForce2 = EuclidCoreRandomTools.nextVector3D(random);

         MecanoTools.computeDynamicForceFast(mass, linearAcceleration, angularVelocity, linearVelocity, dynamicForce1);
         MecanoTools.computeDynamicForce(mass, centerOfMassOffset, angularAcceleration, linearAcceleration, angularVelocity, linearVelocity, dynamicForce2);

         EuclidCoreTestTools.assertEquals(expected, dynamicForce1, EPSILON);
         EuclidCoreTestTools.assertEquals(expected, dynamicForce2, EPSILON);
      }
   }

   @Test
   public void testComputeDynamicMoment() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's compare computeDynamicMoment(...) against
        // computeDynamicMomentFast(...)
         Matrix3D momentOfInertia = EuclidCoreRandomTools.nextMatrix3D(random);
         double mass = random.nextDouble();
         Vector3D centerOfMassOffset = new Vector3D(); // It has to be zero for the comparison to be relevant.
         Vector3D angularAcceleration = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D linearAcceleration = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D linearVelocity = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D dynamicMoment1 = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D dynamicMoment2 = EuclidCoreRandomTools.nextVector3D(random);

         MecanoTools.computeDynamicMoment(momentOfInertia,
                                          mass,
                                          centerOfMassOffset,
                                          angularAcceleration,
                                          linearAcceleration,
                                          angularVelocity,
                                          linearVelocity,
                                          dynamicMoment1);
         MecanoTools.computeDynamicMomentFast(momentOfInertia, angularAcceleration, angularVelocity, dynamicMoment2);

         EuclidCoreTestTools.assertEquals(dynamicMoment1, dynamicMoment2, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's assert that when the acceleration & velocity are zero, the
        // resulting moment is also zero
         Matrix3D momentOfInertia = EuclidCoreRandomTools.nextMatrix3D(random);
         double mass = random.nextDouble();
         // This should not affect the result.
         Vector3D centerOfMassOffset = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D angularAcceleration = new Vector3D();
         Vector3D linearAcceleration = new Vector3D();
         Vector3D angularVelocity = new Vector3D();
         Vector3D linearVelocity = new Vector3D();
         Vector3D expected = new Vector3D();
         Vector3D dynamicMoment1 = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D dynamicMoment2 = EuclidCoreRandomTools.nextVector3D(random);

         MecanoTools.computeDynamicMoment(momentOfInertia,
                                          mass,
                                          centerOfMassOffset,
                                          angularAcceleration,
                                          linearAcceleration,
                                          angularVelocity,
                                          linearVelocity,
                                          dynamicMoment1);
         MecanoTools.computeDynamicMomentFast(momentOfInertia, angularAcceleration, angularVelocity, dynamicMoment2);

         EuclidCoreTestTools.assertEquals(expected, dynamicMoment1, EPSILON);
         EuclidCoreTestTools.assertEquals(expected, dynamicMoment2, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's assert that when the acceleration is zero, the resulting moment
        // is also zero
        // Need to make the matrix diagonal with all elements equal to get rid
        // of Coriolis effect.
         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setIdentity();
         momentOfInertia.scale(random.nextDouble());
         double mass = random.nextDouble();
         // This should not affect the result.
         Vector3D centerOfMassOffset = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D angularAcceleration = new Vector3D();
         Vector3D linearAcceleration = new Vector3D();
         Vector3D angularVelocity = new Vector3D();
         // Rotate around the CoM offset axis to prevent generating centrifugal forces
         angularVelocity.setAndScale(EuclidCoreRandomTools.nextDouble(random, 10.0), centerOfMassOffset);
         Vector3D linearVelocity = new Vector3D();
         // By aligning the angular and linear velocities we prevent generating Coriolis
         // accelerations
         linearVelocity.setAndScale(EuclidCoreRandomTools.nextDouble(random, 10.0), angularVelocity);
         Vector3D expected = new Vector3D();
         Vector3D dynamicMoment1 = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D dynamicMoment2 = EuclidCoreRandomTools.nextVector3D(random);

         MecanoTools.computeDynamicMomentFast(momentOfInertia, angularAcceleration, angularVelocity, dynamicMoment1);
         MecanoTools.computeDynamicMoment(momentOfInertia,
                                          mass,
                                          centerOfMassOffset,
                                          angularAcceleration,
                                          linearAcceleration,
                                          angularVelocity,
                                          linearVelocity,
                                          dynamicMoment2);

         EuclidCoreTestTools.assertEquals(expected, dynamicMoment1, EPSILON);
         EuclidCoreTestTools.assertEquals(expected, dynamicMoment2, EPSILON);
      }

   }

   @Test
   public void testComputeDynamicWrench() throws Exception
   {
      Random random = new Random(3453);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's compute the dynamic wrench in different frames for a sphere.
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         // Let's create the inertia in body frame assuming the CoM is it the bodyFrame's
         // origin.
         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, bodyFrame, 1.0, 1.0, 0.0);
         // Set the inertia of a sphere.
         inertia.getMomentOfInertia().setIdentity();
         double rotationalInertia = random.nextDouble();
         inertia.getMomentOfInertia().scale(rotationalInertia);
         SpatialAcceleration acceleration = MecanoRandomTools.nextSpatialAcceleration(random, bodyFrame, worldFrame, bodyFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, worldFrame, bodyFrame);
         // Need to make the linear velocity colinear with the angular velocity to
         // prevent generating Coriolis forces.
         twist.getLinearPart().setAndScale(EuclidCoreRandomTools.nextDouble(random), twist.getAngularPart());

         // The wrench for a sphere is easy to calculate
         Wrench expectedWrenchAtBody = new Wrench(bodyFrame, bodyFrame);
         expectedWrenchAtBody.getAngularPart().setAndScale(rotationalInertia, acceleration.getAngularPart());
         expectedWrenchAtBody.getLinearPart().setAndScale(inertia.getMass(), acceleration.getLinearPart());

         Wrench wrenchAtBody = new Wrench(bodyFrame, bodyFrame);
         Wrench wrenchAtFrameA = new Wrench(bodyFrame, frameA);

         // Compute the wrench at the bodyFrame, with the center of mass offset set to
         // zero.
         MecanoTools.computeDynamicMomentFast(inertia.getMomentOfInertia(),
                                              acceleration.getAngularPart(),
                                              twist.getAngularPart(),
                                              wrenchAtBody.getAngularPart());
         MecanoTools.computeDynamicForceFast(inertia.getMass(),
                                             acceleration.getLinearPart(),
                                             twist.getAngularPart(),
                                             twist.getLinearPart(),
                                             wrenchAtBody.getLinearPart());

         // Compute the wrench at the frameA, with the center of mass offset non-zero.
         inertia.changeFrame(frameA);
         acceleration.changeFrame(frameA);
         twist.changeFrame(frameA);
         MecanoTools.computeDynamicMoment(inertia.getMomentOfInertia(),
                                          inertia.getMass(),
                                          inertia.getCenterOfMassOffset(),
                                          acceleration.getAngularPart(),
                                          acceleration.getLinearPart(),
                                          twist.getAngularPart(),
                                          twist.getLinearPart(),
                                          wrenchAtFrameA.getAngularPart());
         MecanoTools.computeDynamicForce(inertia.getMass(),
                                         inertia.getCenterOfMassOffset(),
                                         acceleration.getAngularPart(),
                                         acceleration.getLinearPart(),
                                         twist.getAngularPart(),
                                         twist.getLinearPart(),
                                         wrenchAtFrameA.getLinearPart());

         // Let's change frames to bodyFrame and compare the wrenches
         wrenchAtFrameA.changeFrame(bodyFrame);
         MecanoTestTools.assertWrenchEquals("Iteration: " + i, expectedWrenchAtBody, wrenchAtBody, EPSILON);
         MecanoTestTools.assertWrenchEquals("Iteration: " + i, expectedWrenchAtBody, wrenchAtFrameA, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Let's compute the dynamic wrench in different frames and asserts that
        // the result is actually the same.
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame(random);
         // Let's create the inertia in body frame assuming the CoM is it the bodyFrame's
         // origin.
         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, bodyFrame, 1.0, 1.0, 0.0);
         SpatialAcceleration acceleration = MecanoRandomTools.nextSpatialAcceleration(random, bodyFrame, worldFrame, bodyFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, worldFrame, bodyFrame);

         Wrench wrenchAtBody = new Wrench(bodyFrame, bodyFrame);
         Wrench wrenchAtFrameA = new Wrench(bodyFrame, frameA);
         Wrench wrenchAtFrameB = new Wrench(bodyFrame, frameB);

         // Compute the wrench at the bodyFrame, with the center of mass offset set to
         // zero.
         MecanoTools.computeDynamicMomentFast(inertia.getMomentOfInertia(),
                                              acceleration.getAngularPart(),
                                              twist.getAngularPart(),
                                              wrenchAtBody.getAngularPart());
         MecanoTools.computeDynamicForceFast(inertia.getMass(),
                                             acceleration.getLinearPart(),
                                             twist.getAngularPart(),
                                             twist.getLinearPart(),
                                             wrenchAtBody.getLinearPart());

         // Compute the wrench at the frameA, with the center of mass offset non-zero.
         inertia.changeFrame(frameA);
         acceleration.changeFrame(frameA);
         twist.changeFrame(frameA);
         MecanoTools.computeDynamicMoment(inertia.getMomentOfInertia(),
                                          inertia.getMass(),
                                          inertia.getCenterOfMassOffset(),
                                          acceleration.getAngularPart(),
                                          acceleration.getLinearPart(),
                                          twist.getAngularPart(),
                                          twist.getLinearPart(),
                                          wrenchAtFrameA.getAngularPart());
         MecanoTools.computeDynamicForce(inertia.getMass(),
                                         inertia.getCenterOfMassOffset(),
                                         acceleration.getAngularPart(),
                                         acceleration.getLinearPart(),
                                         twist.getAngularPart(),
                                         twist.getLinearPart(),
                                         wrenchAtFrameA.getLinearPart());

         // Compute the wrench at the frameB, with the center of mass offset non-zero.
         inertia.changeFrame(frameB);
         acceleration.changeFrame(frameB);
         twist.changeFrame(frameB);
         MecanoTools.computeDynamicMoment(inertia.getMomentOfInertia(),
                                          inertia.getMass(),
                                          inertia.getCenterOfMassOffset(),
                                          acceleration.getAngularPart(),
                                          acceleration.getLinearPart(),
                                          twist.getAngularPart(),
                                          twist.getLinearPart(),
                                          wrenchAtFrameB.getAngularPart());
         MecanoTools.computeDynamicForce(inertia.getMass(),
                                         inertia.getCenterOfMassOffset(),
                                         acceleration.getAngularPart(),
                                         acceleration.getLinearPart(),
                                         twist.getAngularPart(),
                                         twist.getLinearPart(),
                                         wrenchAtFrameB.getLinearPart());

         // Let's change frame from frameA to bodyFrame and compare the wrenches
         wrenchAtFrameA.changeFrame(bodyFrame);
         MecanoTestTools.assertWrenchEquals(wrenchAtBody, wrenchAtFrameA, EPSILON);

         // Let's change frame from bodyFrame to frameB and compare the wrenches
         wrenchAtBody.changeFrame(frameB);
         MecanoTestTools.assertWrenchEquals(wrenchAtFrameB, wrenchAtBody, EPSILON);
      }
      for (int i = 0; i < ITERATIONS; ++i)
      { // Assert dynamic moment is set to zeros when angular velocity is not
        // provided.
         Matrix3D momentOfInertia = EuclidCoreRandomTools.nextMatrix3D(random);
         Vector3D dynamicMoment = EuclidCoreRandomTools.nextVector3D(random);
         MecanoTools.computeDynamicMomentFast(momentOfInertia, null, null, dynamicMoment);
         assertEquals(EuclidCoreTools.zeroVector3D, dynamicMoment);

      }
   }

   @Test
   public void testComputeKineticCoEnergy() throws Exception
   {
      Random random = new Random(4636);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test against the matrix multiplication: U = 1/2 ( T^T I T )
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame expressedInFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         SpatialInertia inertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, expressedInFrame);
         Twist twist = MecanoRandomTools.nextTwist(random, bodyFrame, worldFrame, expressedInFrame);

         double actual = MecanoTools.computeKineticCoEnergy(inertia.getMomentOfInertia(),
                                                            inertia.getMass(),
                                                            inertia.getCenterOfMassOffset(),
                                                            twist.getAngularPart(),
                                                            twist.getLinearPart());
         double expected;
         { // Compute the matrix form
            DMatrixRMaj inertiaMatrix = new DMatrixRMaj(6, 6);
            DMatrixRMaj twistMatrix = new DMatrixRMaj(6, 1);

            twist.get(twistMatrix);
            inertia.get(inertiaMatrix);

            DMatrixRMaj intermediate = new DMatrixRMaj(1, 6);
            CommonOps_DDRM.multTransA(twistMatrix, inertiaMatrix, intermediate);

            DMatrixRMaj energyMatrix = new DMatrixRMaj(1, 1);
            CommonOps_DDRM.mult(intermediate, twistMatrix, energyMatrix);

            expected = 0.5 * energyMatrix.get(0);
         }

         assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testTransformSymmetricMatrix3D() throws Exception
   {
      Random random = new Random(4363);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         expected.setM10(expected.getM01());
         expected.setM20(expected.getM02());
         expected.setM21(expected.getM12());
         Matrix3D actual = new Matrix3D(expected);

         rotationMatrix.transform(expected);
         MecanoTools.transformSymmetricMatrix3D(rotationMatrix, actual);
      }
   }

   @Test
   public void testInverseTransformSymmetricMatrix3D() throws Exception
   {
      Random random = new Random(4363);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         Matrix3D expected = EuclidCoreRandomTools.nextMatrix3D(random);
         expected.setM10(expected.getM01());
         expected.setM20(expected.getM02());
         expected.setM21(expected.getM12());
         Matrix3D actual = new Matrix3D(expected);

         rotationMatrix.inverseTransform(expected);
         MecanoTools.inverseTransformSymmetricMatrix3D(rotationMatrix, actual);
      }
   }

   @Test
   public void testAddEquals()
   {
      Random random = new Random(3424);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addEquals(int startRow, int column, SpatialVectorReadOnly vectorToAdd, DMatrixRMaj matrixToModify)
         int numRow = 6 + random.nextInt(10);
         int numCol = 1 + random.nextInt(10);
         DMatrixRMaj original = RandomMatrices_DDRM.rectangle(numRow, numCol, -10.0, 10.0, random);
         SpatialVector vector = MecanoRandomTools.nextSpatialVector(random, ReferenceFrame.getWorldFrame());

         int startRow = numRow == 6 ? 0 : random.nextInt(numRow - 6);
         int column = numCol == 1 ? 0 : random.nextInt(numCol - 1);

         DMatrixRMaj expected = new DMatrixRMaj(original.getNumRows(), original.getNumCols());
         vector.get(startRow, column, expected);
         CommonOps_DDRM.addEquals(expected, original);

         DMatrixRMaj actual = new DMatrixRMaj(original);
         MecanoTools.addEquals(startRow, column, vector, actual);

         MecanoTestTools.assertDMatrixEquals(expected, actual, 1.0e-12);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addEquals(int startRow, int startColumn, DMatrix matrixToAdd, Matrix3DBasics matrixToModify)
         int numRow = 3 + random.nextInt(10);
         int numCol = 3 + random.nextInt(10);
         DMatrixRMaj matrixToAdd = RandomMatrices_DDRM.rectangle(numRow, numCol, -10.0, 10.0, random);
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);

         int startRow = numRow == 3 ? 0 : random.nextInt(numRow - 3);
         int startCol = numCol == 3 ? 0 : random.nextInt(numCol - 3);

         Matrix3D expected = new Matrix3D();
         expected.set(startRow, startCol, matrixToAdd);
         expected.add(original);

         Matrix3D actual = new Matrix3D(original);
         MecanoTools.addEquals(startRow, startCol, matrixToAdd, actual);

         EuclidCoreTestTools.assertEquals(expected, actual, 1.0e-12);
      }
   }
}
