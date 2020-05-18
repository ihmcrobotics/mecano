package us.ihmc.mecano.tools;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class contains tools to obtain the moment of inertia matrix of primitive shapes.
 */
public class MomentOfInertiaFactory
{
   /**
    * Computes the moment of inertia matrix for a solid cylinder.
    *
    * @param mass           the cylinder mass.
    * @param radius         the radius of the cylinder.
    * @param height         the height, or length, of the cylinder.
    * @param axisOfCylinder the revolution axis of the cylinder. Not modified.
    * @return the moment of inertia of the cylinder.
    */
   public static Matrix3D solidCylinder(double mass, double radius, double height, Vector3DReadOnly axisOfCylinder)
   {
      checkMassAndDimensions(mass, radius, height);

      double IalongAxis = 0.5 * mass * radius * radius;
      double IcrossAxis = mass * (3.0 * radius * radius + height * height) / 12.0;

      Vector3D principalInertia = new Vector3D(1.0, 1.0, 1.0);
      principalInertia.sub(axisOfCylinder);
      principalInertia.scale(IcrossAxis);
      principalInertia.scaleAdd(IalongAxis, axisOfCylinder, principalInertia);

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(principalInertia.getX());
      momentOfInertia.setM11(principalInertia.getY());
      momentOfInertia.setM22(principalInertia.getZ());
      return momentOfInertia;

   }

   /**
    * Computes the moment of inertia matrix for a solid ellipsoid.
    *
    * @param mass  the ellipsoid mass.
    * @param radii the three radii of the ellipsoid. Not modified.
    * @return the moment of inertia of the ellipsoid.
    */
   public static Matrix3D solidEllipsoid(double mass, Tuple3DReadOnly radii)
   {
      return solidEllipsoid(mass, radii.getX(), radii.getY(), radii.getZ());
   }

   /**
    * Computes the moment of inertia matrix for a solid ellipsoid.
    *
    * @param mass    the ellipsoid mass.
    * @param xRadius radius of the ellipsoid along the x-axis.
    * @param yRadius radius of the ellipsoid along the y-axis.
    * @param zRadius radius of the ellipsoid along the z-axis.
    * @return the moment of inertia of the ellipsoid.
    */
   public static Matrix3D solidEllipsoid(double mass, double xRadius, double yRadius, double zRadius)
   {
      checkMassAndDimensions(mass, xRadius, yRadius, zRadius);
      Matrix3D momentOfInertia = new Matrix3D();
      double scale = 0.2 * mass;
      momentOfInertia.setM00(scale * (yRadius * yRadius + zRadius * zRadius));
      momentOfInertia.setM11(scale * (zRadius * zRadius + xRadius * xRadius));
      momentOfInertia.setM22(scale * (xRadius * xRadius + yRadius * yRadius));
      return momentOfInertia;
   }

   /**
    * Computes the moment of inertia matrix for a solid sphere.
    *
    * @param mass   the sphere mass.
    * @param radius radius of the sphere.
    * @return the moment of inertia of the sphere.
    */
   public static Matrix3D solidSphere(double mass, double radius)
   {
      checkMassAndDimensions(mass, radius);
      Matrix3D momentOfInertia = new Matrix3D();
      double inertia = 0.4 * mass * radius * radius;
      momentOfInertia.setM00(inertia);
      momentOfInertia.setM11(inertia);
      momentOfInertia.setM22(inertia);
      return momentOfInertia;
   }

   /**
    * Computes the moment of inertia matrix for a solid box.
    *
    * @param mass the box mass.
    * @param size 3D size of the box along each axis.
    * @return the moment of inertia of the box.
    */
   public static Matrix3D solidBox(double mass, Tuple3DReadOnly size)
   {
      return solidBox(mass, size.getX(), size.getY(), size.getZ());
   }

   /**
    * Computes the moment of inertia matrix for a solid box.
    *
    * @param mass  the box mass.
    * @param xSize size of the box along the x-axis.
    * @param ySize size of the box along the y-axis.
    * @param zSize size of the box along the z-axis.
    * @return the moment of inertia of the box.
    */
   public static Matrix3D solidBox(double mass, double xSize, double ySize, double zSize)
   {
      checkMassAndDimensions(mass, xSize, ySize, zSize);
      Matrix3D momentOfInertia = new Matrix3D();
      double scale = mass / 12.0;
      momentOfInertia.setM00(scale * (ySize * ySize + zSize * zSize));
      momentOfInertia.setM11(scale * (zSize * zSize + xSize * xSize));
      momentOfInertia.setM22(scale * (xSize * xSize + ySize * ySize));
      return momentOfInertia;
   }

   /**
    * Compute the moment of inertia of are computed as follows:
    *
    * <pre>
    * Ixx = mass * (radiusOfGyrationY * radiusOfGyrationY + radiusOfGyrationZ * radiusOfGyrationY)
    * Iyy = mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationZ * radiusOfGyrationZ)
    * Izz = mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationY * radiusOfGyrationY)
    * </pre>
    *
    * This is equivalent to the mass being concentrated on the surface of a thin ellipsoid with the
    * given radii of gyration.
    *
    * @param mass              Mass of the body.
    * @param radiusOfGyrationX Radius of gyration in the x direction.
    * @param radiusOfGyrationY Radius of gyration in the y direction.
    * @param radiusOfGyrationZ Radius of gyration in the z direction.
    * @return the moment of inertia matrix.
    */
   public static Matrix3D fromMassAndRadiiOfGyration(double mass, double radiusOfGyrationX, double radiusOfGyrationY, double radiusOfGyrationZ)
   {
      checkMassAndDimensions(mass, radiusOfGyrationX, radiusOfGyrationY, radiusOfGyrationZ);

      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(mass * (radiusOfGyrationY * radiusOfGyrationY + radiusOfGyrationZ * radiusOfGyrationZ));
      momentOfInertia.setM11(mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationZ * radiusOfGyrationZ));
      momentOfInertia.setM22(mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationY * radiusOfGyrationY));
      return momentOfInertia;
   }

   private static void checkMassAndDimensions(double mass, double... dimensions)
   {
      if (mass < 0.0)
         throw new IllegalArgumentException("can not pass in negative mass values");

      for (double dimension : dimensions)
      {
         if (dimension < 0.0)
            throw new IllegalArgumentException("can not pass in negative dimensions");
      }
   }
}
