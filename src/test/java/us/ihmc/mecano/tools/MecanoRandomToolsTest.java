package us.ihmc.mecano.tools;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.SymmetricEigenDecomposition3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

import static org.junit.jupiter.api.Assertions.*;

public class MecanoRandomToolsTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testNextSpatialInertia()
   {
      Random random = new Random(349857);

      // Attempting to construct with negative mass should throw an exception
      ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      double inertiaMax = 1.0;
      double massMax = -random.nextDouble();
      double centerOfMassOffsetMinMax = 1.0;
      try
      {
         SpatialInertiaReadOnly spatialInertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, bodyFrame,
                                                                                      inertiaMax, massMax, centerOfMassOffsetMinMax);
      }
      catch (IllegalArgumentException e)
      {
         // Good
      }

      // Attempting to construct with negative inertia should throw an exception
      inertiaMax = -random.nextDouble();
      massMax = 1.0;
      centerOfMassOffsetMinMax = 1.0;
      try
      {
         SpatialInertiaReadOnly spatialInertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, bodyFrame,
                                                                                      inertiaMax, massMax, centerOfMassOffsetMinMax);
      }
      catch (IllegalArgumentException e)
      {
         // Good
      }

      // Otherwise, the resulting spatial inertia should be "fully physically consistent":
      // 1. mass > 0
      // 2. moment of inertia is positive definite
      // 3. principal moments of inertia (the diagonal entries of the diagonalized moment of inertia matrix) satisfy the triangle inequality
      for (int i = 0; i < ITERATIONS; i++)
      {
         bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         inertiaMax = 1.0;
         massMax = 1.0;
         centerOfMassOffsetMinMax = 1.0;
         SpatialInertiaReadOnly spatialInertia = MecanoRandomTools.nextSpatialInertia(random, bodyFrame, bodyFrame,
                                                                                         inertiaMax, massMax, centerOfMassOffsetMinMax);

         assertTrue(isFullyPhysicallyConsistent(spatialInertia));
      }
   }

   /**
    * Check whether the input {@code spatialInertia} is fully physically consistent.
    * <p>
    * A spatial inertia is fully physically consistent if and only if it is physically consistent (see {@link #isPhysicallyConsistent(SpatialInertiaReadOnly)})
    * and its principal moments of inertia (the diagonal entries of the diagonalized moment of inertia matrix) satisfy the triangle inequality.
    * </p>
    * @param spatialInertia the spatial inertia to check.
    * @return {@code true} if the spatial inertia is fully physically consistent, {@code false} otherwise.
    */
   private static boolean isFullyPhysicallyConsistent(SpatialInertiaReadOnly spatialInertia)
   {
      DMatrixRMaj momentOfInertia = new DMatrixRMaj(3, 3);
      SymmetricEigenDecomposition3D eigenDecomposition = new SymmetricEigenDecomposition3D();
      eigenDecomposition.decompose(spatialInertia.getMomentOfInertia());
      Vector3D eigenvalues = eigenDecomposition.getEigenValues();

      boolean isTriangleInequalitySatisfied = eigenvalues.getX() < eigenvalues.getY() + eigenvalues.getZ() &&
                                              eigenvalues.getY() < eigenvalues.getX() + eigenvalues.getZ() &&
                                              eigenvalues.getZ() < eigenvalues.getX() + eigenvalues.getY();

      return isPhysicallyConsistent(spatialInertia) && isTriangleInequalitySatisfied;

   }

   /**
    * Check whether the input {@code spatialInertia} is physically consistent.
    * <p>
    * A spatial inertia is physically consistent if and only if mass is positive, and the moment of inertia is positive definite, the latter of which is
    * checked using Sylvester's criterion.
    * </p>
    * @param spatialInertia the spatial inertia to check.
    * @return {@code true} if the spatial inertia is physically consistent, {@code false} otherwise.
    */
   private static boolean isPhysicallyConsistent(SpatialInertiaReadOnly spatialInertia)
   {
      Matrix3DReadOnly momentOfInertia = spatialInertia.getMomentOfInertia();
      // Sylvester's criterion: A matrix is positive definite if and only if all its principal minors are positive
      double firstPrincipalMinor = momentOfInertia.getM00();
      double secondPrincipalMinor = momentOfInertia.getM00() * momentOfInertia.getM11() - momentOfInertia.getM01() * momentOfInertia.getM10();
      double thirdPrincipalMinor = momentOfInertia.determinant();

      return spatialInertia.getMass() > 0.0 && firstPrincipalMinor > 0.0 && secondPrincipalMinor > 0.0 && thirdPrincipalMinor > 0.0;
   }
}
