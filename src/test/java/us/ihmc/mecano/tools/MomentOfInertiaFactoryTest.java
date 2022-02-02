package us.ihmc.mecano.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import java.util.Random;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class MomentOfInertiaFactoryTest
{
   
   private static final int ITERATIONS = 1000;
   private static final int MAX_VALUE = 10;
   
   @Test
   public void testSolidCylinder() throws Exception
   {
      Random random = new Random(130375);
      String type = "Solid Cylinder";
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         double randomMass = Math.abs(random.nextDouble());
         double randomRadius = Math.abs(random.nextDouble());
         double randomHeight = Math.abs(random.nextDouble());
         Vector3DReadOnly randomAxisOfCylinder = EuclidCoreRandomTools.nextElementIn(random, Axis3D.values);
                  
         Matrix3D momentOfInertia = MomentOfInertiaFactory.solidCylinder(randomMass, randomRadius, randomHeight, randomAxisOfCylinder);
         
         assertGeneralInertiaProperties(momentOfInertia, type);
         assertThrowSolidCylinderNegativeValue(randomMass, -1.0*randomRadius, randomHeight, randomAxisOfCylinder, type + " :negative radius did not throw Illegal Argument Exception");
         assertThrowSolidCylinderNegativeValue(randomMass, randomRadius, -1.0*randomHeight, randomAxisOfCylinder, type + " :negative height did not throw Illegal Argument Exception");
         assertThrowSolidCylinderNegativeValue(-1.0*randomMass, randomRadius, randomHeight, randomAxisOfCylinder, type + " :negative mass did not throw Illegal Argument Exception");
      }
   }

   @Test
   public void testSolidEllipsoid() throws Exception
   {
      Random random = new Random(175521);
      String type = "Solid Ellipsoid";

      for (int i = 0; i < ITERATIONS; i++)
      {
         double randomMass = Math.abs(random.nextDouble());
         double randomXRadius = Math.abs(random.nextDouble());
         double randomYRadius = Math.abs(random.nextDouble());
         double randomZRadius = Math.abs(random.nextDouble());
      
         Matrix3D momentOfInertia = MomentOfInertiaFactory.solidEllipsoid(randomMass, randomXRadius, randomYRadius, randomZRadius);
         
         assertEquals(momentOfInertia.getM00(), (0.2 * randomMass) * (randomYRadius * randomYRadius + randomZRadius * randomZRadius));
         assertEquals(momentOfInertia.getM11(), (0.2 * randomMass) * (randomZRadius * randomZRadius + randomXRadius * randomXRadius));
         assertEquals(momentOfInertia.getM22(), (0.2 * randomMass) * (randomXRadius * randomXRadius + randomYRadius * randomYRadius));
         
         assertGeneralInertiaProperties(momentOfInertia, type);
         assertThrowSolidEllipsoidNegativeValues(randomMass, -1.0*randomXRadius, randomYRadius, randomZRadius, type + " :negative X radius did not throw Illegal Argument Exception");
         assertThrowSolidEllipsoidNegativeValues(randomMass, randomXRadius, -1.0*randomYRadius, randomZRadius, type + " :negative Y radius did not throw Illegal Argument Exception");
         assertThrowSolidEllipsoidNegativeValues(randomMass, randomXRadius, randomYRadius, -1.0*randomZRadius, type + " :negative Z radius did not throw Illegal Argument Exception");
         assertThrowSolidEllipsoidNegativeValues(-1.0*randomMass, randomXRadius, randomYRadius, randomZRadius, type + " :negative mass did not throw Illegal Argument Exception");
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         double randomMass = Math.abs(random.nextDouble());
         Vector3D randomRadii = EuclidCoreRandomTools.nextVector3D(random, 0.0, MAX_VALUE);
         
         Matrix3D momentOfInertia = MomentOfInertiaFactory.solidEllipsoid(randomMass, randomRadii);
         
         assertEquals(momentOfInertia.getM00(), (0.2 * randomMass) * (randomRadii.getY() * randomRadii.getY() + randomRadii.getZ() * randomRadii.getZ()));
         assertEquals(momentOfInertia.getM11(), (0.2 * randomMass) * (randomRadii.getZ() * randomRadii.getZ() + randomRadii.getX() * randomRadii.getX()));
         assertEquals(momentOfInertia.getM22(), (0.2 * randomMass) * (randomRadii.getX() * randomRadii.getX() + randomRadii.getY() * randomRadii.getY()));
         
         assertGeneralInertiaProperties(momentOfInertia, type);
         assertThrowSolidEllipsoidNegativeValues(-1.0*randomMass, randomRadii, type + " :negative mass did not throw Illegal Argument Exception");
         Vector3D negRadii = new Vector3D(-1.0*randomRadii.getX(), randomRadii.getY(), randomRadii.getZ());
         assertThrowSolidEllipsoidNegativeValues(randomMass, negRadii, type + " :negative X radii did not throw Illegal Argument Exception");
         negRadii = new Vector3D(randomRadii.getX(), -1.0*randomRadii.getY(), randomRadii.getZ());
         assertThrowSolidEllipsoidNegativeValues(randomMass, negRadii, type + " :negative Y radii did not throw Illegal Argument Exception");
         negRadii = new Vector3D(randomRadii.getX(), randomRadii.getY(), -1.0*randomRadii.getZ());
         assertThrowSolidEllipsoidNegativeValues(randomMass, negRadii, type + " :negative Y radii did not throw Illegal Argument Exception");
      }
   }
  
   @Test
   public void testSolidSphere() throws Exception
   {
      Random random = new Random(130125);
      String type = "Solid Sphere";

      for (int i = 0; i < ITERATIONS; i++)
      {
         double randomMass = Math.abs(random.nextDouble());
         double randomRadius = Math.abs(random.nextDouble());
      
         Matrix3D momentOfInertia = MomentOfInertiaFactory.solidSphere(randomMass, randomRadius);

         assertEquals(momentOfInertia.getM00(), 0.4 * randomMass * randomRadius * randomRadius);
         assertTrue(momentOfInertia.getM00() == momentOfInertia.getM11() && momentOfInertia.getM00() == momentOfInertia.getM22());

         assertGeneralInertiaProperties(momentOfInertia, type);
         assertThrowSolidSphereNegativeValues(randomMass, -1.0*randomRadius, type + " :negative radius did not throw Illegal Argument Exception");
         assertThrowSolidSphereNegativeValues(-1.0*randomMass, randomRadius, type + " :negative mass did not throw Illegal Argument Exception");
      }
   }
   
   @Test
   public void testSolidBox() throws Exception
   {
      Random random = new Random(160355);
      String type = "Solid Box";

      for (int i = 0; i < ITERATIONS; i++)
      {
         double randomMass = Math.abs(random.nextDouble());
         double randomXSize = Math.abs(random.nextDouble());
         double randomYSize = Math.abs(random.nextDouble());
         double randomZSize = Math.abs(random.nextDouble());
      
         Matrix3D momentOfInertia = MomentOfInertiaFactory.solidBox(randomMass, randomXSize, randomYSize, randomZSize);

         assertEquals(momentOfInertia.getM00(), (randomMass / 12.0) * (randomYSize * randomYSize + randomZSize * randomZSize));
         assertEquals(momentOfInertia.getM11(), (randomMass / 12.0) * (randomZSize * randomZSize + randomXSize * randomXSize));
         assertEquals(momentOfInertia.getM22(), (randomMass / 12.0) * (randomXSize * randomXSize + randomYSize * randomYSize));
         
         assertGeneralInertiaProperties(momentOfInertia, type);
         assertThrowSolidBoxNegativeValues(randomMass, -1.0*randomXSize, randomYSize, randomZSize, type + " :negative X radus did not throw Illegal Argument Exception");
         assertThrowSolidBoxNegativeValues(randomMass, randomXSize, -1.0*randomYSize, randomZSize, type + " :negative Y radus did not throw Illegal Argument Exception");
         assertThrowSolidBoxNegativeValues(randomMass, randomXSize, randomYSize, -1.0*randomZSize, type + " :negative Z radus did not throw Illegal Argument Exception");
         assertThrowSolidBoxNegativeValues(-1.0*randomMass, randomXSize, randomYSize, randomZSize, type + " :negative mass did not throw Illegal Argument Exception");
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         double randomMass = Math.abs(random.nextDouble());
         Vector3D randomRadii = EuclidCoreRandomTools.nextVector3D(random, 0.0, MAX_VALUE);
         
         Matrix3D momentOfInertia = MomentOfInertiaFactory.solidBox(randomMass, randomRadii);
         
         assertEquals(momentOfInertia.getM00(), (randomMass / 12.0) * (randomRadii.getY() * randomRadii.getY() + randomRadii.getZ() * randomRadii.getZ()));
         assertEquals(momentOfInertia.getM11(), (randomMass / 12.0) * (randomRadii.getZ() * randomRadii.getZ() + randomRadii.getX() * randomRadii.getX()));
         assertEquals(momentOfInertia.getM22(), (randomMass / 12.0) * (randomRadii.getX() * randomRadii.getX() + randomRadii.getY() * randomRadii.getY()));
         
         assertGeneralInertiaProperties(momentOfInertia, type);
         assertThrowSolidBoxNegativeValues(-1.0*randomMass, randomRadii, type + " :negative mass did not throw Illegal Argument Exception");
         Vector3D negRadii = new Vector3D(-1.0*randomRadii.getX(), randomRadii.getY(), randomRadii.getZ());
         assertThrowSolidBoxNegativeValues(randomMass, negRadii, type + " :negative X radii did not throw Illegal Argument Exception");
         negRadii = new Vector3D(randomRadii.getX(), -1.0*randomRadii.getY(), randomRadii.getZ());
         assertThrowSolidBoxNegativeValues(randomMass, negRadii, type + " :negative Y radii did not throw Illegal Argument Exception");
         negRadii = new Vector3D(randomRadii.getX(), randomRadii.getY(), -1.0*randomRadii.getZ());
         assertThrowSolidBoxNegativeValues(randomMass, negRadii, type + " :negative Z radii did not throw Illegal Argument Exception");
      }
   }

   @Test
   public void testFromMassAndRadiiOfGyration() throws Exception
   {
      Random random = new Random(12745);
      String type = "From Mass and Radii of Gyration";

      for (int i = 0; i < ITERATIONS; i++)
      {
         double randomMass = Math.abs(random.nextDouble());
         double randomRadiusOfGyrationX = Math.abs(random.nextDouble());
         double randomRadiusOfGyrationY = Math.abs(random.nextDouble());
         double randomRadiusOfGyrationZ = Math.abs(random.nextDouble());
      
         Matrix3D momentOfInertia = MomentOfInertiaFactory.fromMassAndRadiiOfGyration(randomMass, randomRadiusOfGyrationX, randomRadiusOfGyrationY, randomRadiusOfGyrationZ);
 
         assertEquals(momentOfInertia.getM00(), randomMass * (randomRadiusOfGyrationY * randomRadiusOfGyrationY + randomRadiusOfGyrationZ * randomRadiusOfGyrationZ));
         assertEquals(momentOfInertia.getM11(), randomMass * (randomRadiusOfGyrationZ * randomRadiusOfGyrationZ + randomRadiusOfGyrationX * randomRadiusOfGyrationX));
         assertEquals(momentOfInertia.getM22(), randomMass * (randomRadiusOfGyrationX * randomRadiusOfGyrationX + randomRadiusOfGyrationY * randomRadiusOfGyrationY));
         
         assertGeneralInertiaProperties(momentOfInertia, type);
         assertThrowFromMassAndRadiiOfGyrationNegativeValues(randomMass, -1.0*randomRadiusOfGyrationX, randomRadiusOfGyrationY, randomRadiusOfGyrationZ, type + " :negative X Radius of Gyration did not throw Illegal Argument Exception");
         assertThrowFromMassAndRadiiOfGyrationNegativeValues(randomMass, randomRadiusOfGyrationX, -1.0*randomRadiusOfGyrationY, randomRadiusOfGyrationZ, type + " :negative Y Radius of Gyration did not throw Illegal Argument Exception");
         assertThrowFromMassAndRadiiOfGyrationNegativeValues(randomMass, randomRadiusOfGyrationX, randomRadiusOfGyrationY, -1.0*randomRadiusOfGyrationZ, type + " :negative Z Radius of Gyration did not throw Illegal Argument Exception");
         assertThrowFromMassAndRadiiOfGyrationNegativeValues(-1.0*randomMass, randomRadiusOfGyrationX, randomRadiusOfGyrationY, randomRadiusOfGyrationZ, type + " :negative mass did not throw Illegal Argument Exception");
      }
   }

   //Common Assert Methods
   private static void assertGeneralInertiaProperties(Matrix3DReadOnly momentOfInertia, String type)
   {
      //To be a valid Moment of Inertia Matrix, the following must be true:
      //  M00 + M11 >= M22
      //  M22 + M00 >= M11
      //  M11 + M22 >= M00
      //  M00, M11, and M22 must all be positive
      
      assertTrue(momentOfInertia.getM00() + momentOfInertia.getM11() >= momentOfInertia.getM22(), type + " :General Inertia Property failed: M00 + M11 is not greater than or equal to M22.");
      assertTrue(momentOfInertia.getM22() + momentOfInertia.getM00() >= momentOfInertia.getM11(), type + " :General Inertia Property failed: M22 + M00 is not greater than or equal to M11.");
      assertTrue(momentOfInertia.getM11() + momentOfInertia.getM22() >= momentOfInertia.getM00(), type + " :General Inertia Property failed: M11 + M22 is not greater than or equal to M00.");
      assertTrue(momentOfInertia.getM00() >= 0 && momentOfInertia.getM11() >=0 && momentOfInertia.getM22() >=0, type + " :General Inertia Property failed: M00 or M11 or M22 is negative.");

      //Verify values other than M00, M11, and M22 are all zero
      assertTrue(momentOfInertia.getM01() == 0 && momentOfInertia.getM02() == 0 && momentOfInertia.getM10() == 0 && momentOfInertia.getM12() == 0 && momentOfInertia.getM20() == 0 && momentOfInertia.getM21() == 0, type + " :One of the following values is not zero - M01, M02, M10, M12, M20, or M21");
   }
   
   private static void assertThrowSolidCylinderNegativeValue(double randomMass, double randomRadius, double randomHeight, Vector3DReadOnly randomAxisOfCylinder, String errorMessage)
   {
      Assertions.assertThrows(IllegalArgumentException.class, () ->
      {
         @SuppressWarnings("unused")
         Matrix3D momentOfInertiaTestNeg = MomentOfInertiaFactory.solidCylinder(randomMass, randomRadius, randomHeight, randomAxisOfCylinder);
      }, errorMessage);
   }
   
   private static void assertThrowSolidEllipsoidNegativeValues(double randomMass, double randomXRadius, double randomYRadius, double randomZRadius, String errorMessage)
   {
      Assertions.assertThrows(IllegalArgumentException.class, () ->
      {
         @SuppressWarnings("unused")
         Matrix3D momentOfInertiaTestNeg = MomentOfInertiaFactory.solidEllipsoid(randomMass, randomXRadius, randomYRadius, randomZRadius);
      }, errorMessage);
   }

   private static void assertThrowSolidEllipsoidNegativeValues(double randomMass, Vector3DReadOnly randomRadii, String errorMessage)
   {
      Assertions.assertThrows(IllegalArgumentException.class, () ->
      {
         @SuppressWarnings("unused")
         Matrix3D momentOfInertiaTestNeg = MomentOfInertiaFactory.solidEllipsoid(randomMass, randomRadii);
      }, errorMessage);
   }

   private static void assertThrowSolidSphereNegativeValues(double randomMass, double randomRadius, String errorMessage)
   {
      Assertions.assertThrows(IllegalArgumentException.class, () ->
      {
         @SuppressWarnings("unused")
         Matrix3D momentOfInertiaTestNeg = MomentOfInertiaFactory.solidSphere(randomMass, randomRadius);
      }, errorMessage);
   }

   private static void assertThrowSolidBoxNegativeValues(double randomMass, Vector3DReadOnly randomRadii, String errorMessage)
   {
      Assertions.assertThrows(IllegalArgumentException.class, () ->
      {
         @SuppressWarnings("unused")
         Matrix3D momentOfInertiaTestNeg = MomentOfInertiaFactory.solidBox(randomMass, randomRadii);
      }, errorMessage);
   }

   private static void assertThrowSolidBoxNegativeValues(double randomMass, double randomXSize, double randomYSize, double randomZSize, String errorMessage)
   {
      Assertions.assertThrows(IllegalArgumentException.class, () ->
      {
         @SuppressWarnings("unused")
         Matrix3D momentOfInertiaTestNeg = MomentOfInertiaFactory.solidBox(randomMass, randomXSize, randomYSize, randomZSize);
      }, errorMessage);
   }
   
   private static void assertThrowFromMassAndRadiiOfGyrationNegativeValues(double randomMass,
                                                                           double randomRadiusOfGyrationX,
                                                                           double randomRadiusOfGyrationY,
                                                                           double randomRadiusOfGyrationZ,
                                                                           String errorMessage)
   {
      Assertions.assertThrows(IllegalArgumentException.class, () ->
      {
         @SuppressWarnings("unused")
         Matrix3D momentOfInertiaTestNeg = MomentOfInertiaFactory.fromMassAndRadiiOfGyration(randomMass, randomRadiusOfGyrationX, randomRadiusOfGyrationY, randomRadiusOfGyrationZ);
      }, errorMessage);
   }
}
