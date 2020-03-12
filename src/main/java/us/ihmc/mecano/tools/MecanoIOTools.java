package us.ihmc.mecano.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.*;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

/**
 * {@code RigidBodyDynamicsIOTools}, as {@link EuclidCoreIOTools}, is intended to gather the input &
 * output tools for printing, saving, and loading spatial vectors.
 * <p>
 * At this time, only a few print tools are offered, additional features will come in future
 * releases.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class MecanoIOTools
{
   /**
    * Gets a representative {@code String} of {@code spatialVector} as follows:
    *
    * <pre>
    * Spatial Vector: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param spatialVector the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialVectorString(SpatialVectorReadOnly spatialVector)
   {
      return getSpatialVectorString(DEFAULT_FORMAT, spatialVector);
   }

   /**
    * Gets a representative {@code String} of {@code spatialVector} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial Vector: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param spatialVector the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialVectorString(String format, SpatialVectorReadOnly spatialVector)
   {
      if (spatialVector == null)
         return "null";
      else
         return getSpatialVectorString(format, spatialVector.getReferenceFrame(), spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   /**
    * Gets a representative {@code String} of {@code spatialVector} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial Vector: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param expressedInFrame the reference frame in which the spatial vector is expressed.
    * @param angularPart      the angular part of the spatial vector to get the {@code String} of. Not
    *                         modified.
    * @param linearPart       the linear part of the spatial vector to get the {@code String} of. Not
    *                         modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialVectorString(String format, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      return "Spatial Vector: [angular = " + getTuple3DString(format, angularPart) + ", linear = " + getTuple3DString(format, linearPart) + "] - "
            + expressedInFrame;
   }

   /**
    * Gets a representative {@code String} of {@code spatialForce} as follows:
    *
    * <pre>
    * Spatial Force: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param spatialForce the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialForceVectorString(SpatialForceReadOnly spatialForce)
   {
      return getSpatialForceString(DEFAULT_FORMAT, spatialForce);
   }

   /**
    * Gets a representative {@code String} of {@code spatialForce} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial Force: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format       the format to use for each number.
    * @param spatialForce the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialForceString(String format, SpatialForceReadOnly spatialForce)
   {
      if (spatialForce == null)
         return "null";
      else
         return getSpatialForceVectorString(format, spatialForce.getReferenceFrame(), spatialForce.getAngularPart(), spatialForce.getLinearPart());
   }

   /**
    * Gets a representative {@code String} of {@code spatialForceVector} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial Force: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param expressedInFrame the reference frame in which the spatial force vector is expressed.
    * @param angularPart      the angular part of the spatial force vector to get the {@code String}
    *                         of. Not modified.
    * @param linearPart       the linear part of the spatial force vector to get the {@code String} of.
    *                         Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialForceVectorString(String format, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      return "Spatial Force: [angular = " + getTuple3DString(format, angularPart) + ", linear = " + getTuple3DString(format, linearPart) + "] - "
            + expressedInFrame;
   }

   /**
    * Gets a representative {@code String} of {@code wrench} as follows:
    *
    * <pre>
    * Wrench exerted on bodyFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param wrench the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getWrenchString(WrenchReadOnly wrench)
   {
      return getWrenchString(DEFAULT_FORMAT, wrench);
   }

   /**
    * Gets a representative {@code String} of {@code wrench} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Wrench exerted on bodyFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param wrench the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getWrenchString(String format, WrenchReadOnly wrench)
   {
      if (wrench == null)
         return "null";
      else
         return getWrenchString(format, wrench.getBodyFrame(), wrench.getReferenceFrame(), wrench.getAngularPart(), wrench.getLinearPart());
   }

   /**
    * Gets a representative {@code String} of {@code wrench} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Wrench exerted on bodyFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param bodyFrame        the frame of the body on which the wrench is exerted.
    * @param expressedInFrame the reference frame in which the wrench is expressed.
    * @param angularPart      the angular part of the wrench to get the {@code String} of. Not
    *                         modified.
    * @param linearPart       the linear part of the wrench to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getWrenchString(String format, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart,
                                        Vector3DReadOnly linearPart)
   {
      return "Wrench exerted on " + bodyFrame + ": [angular = " + getTuple3DString(format, angularPart) + ", linear = " + getTuple3DString(format, linearPart)
            + "] - " + expressedInFrame;
   }

   /**
    * Gets a representative {@code String} of {@code spatialImpulse} as follows:
    *
    * <pre>
    * Spatial impulse exerted on bodyFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param spatialImpulse the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialImpulseString(SpatialImpulseReadOnly spatialImpulse)
   {
      return getSpatialImpulseString(DEFAULT_FORMAT, spatialImpulse);
   }

   /**
    * Gets a representative {@code String} of {@code spatialImpulse} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial impulse exerted on bodyFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format         the format to use for each number.
    * @param spatialImpulse the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialImpulseString(String format, SpatialImpulseReadOnly spatialImpulse)
   {
      if (spatialImpulse == null)
         return "null";
      else
         return getSpatialImpulseString(format,
                                        spatialImpulse.getBodyFrame(),
                                        spatialImpulse.getReferenceFrame(),
                                        spatialImpulse.getAngularPart(),
                                        spatialImpulse.getLinearPart());
   }

   /**
    * Gets a representative {@code String} of {@code spatialImpulse} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial impulse exerted on bodyFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param bodyFrame        the frame of the body on which the wrench is exerted.
    * @param expressedInFrame the reference frame in which the wrench is expressed.
    * @param angularPart      the angular part of the impulse to get the {@code String} of. Not
    *                         modified.
    * @param linearPart       the linear part of the impulse to get the {@code String} of. Not
    *                         modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialImpulseString(String format, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart,
                                                Vector3DReadOnly linearPart)
   {
      return "Spatial impulse exerted on " + bodyFrame + ": [angular = " + getTuple3DString(format, angularPart) + ", linear = "
            + getTuple3DString(format, linearPart) + "] - " + expressedInFrame;
   }

   /**
    * Gets a representative {@code String} of {@code twist} as follows:
    *
    * <pre>
    * Twist of bodyFrame, with respect to baseFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param twist the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTwistString(TwistReadOnly twist)
   {
      return getTwistString(DEFAULT_FORMAT, twist);
   }

   /**
    * Gets a representative {@code String} of {@code twist} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Twist of bodyFrame, with respect to baseFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param twist  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTwistString(String format, TwistReadOnly twist)
   {
      if (twist == null)
         return "null";
      else
         return getTwistString(format, twist.getBodyFrame(), twist.getBaseFrame(), twist.getReferenceFrame(), twist.getAngularPart(), twist.getLinearPart());
   }

   /**
    * Gets a representative {@code String} of {@code twist} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Twist of bodyFrame, with respect to baseFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param bodyFrame        what we are specifying the twist of.
    * @param baseFrame        with respect to what we are specifying the twist.
    * @param expressedInFrame in which reference frame the twist is expressed.
    * @param angularPart      the angular part of the twist to get the {@code String} of. Not modified.
    * @param linearPart       the linear part of the twist to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTwistString(String format, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
                                       Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      return "Twist of " + bodyFrame + ", with respect to " + baseFrame + ": [angular = " + getTuple3DString(format, angularPart) + ", linear = "
            + getTuple3DString(format, linearPart) + "] - " + expressedInFrame;
   }

   /**
    * Gets a representative {@code String} of {@code spatialAccelerationVector} as follows:
    *
    * <pre>
    * Spatial acceleration of bodyFrame, with respect to baseFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    *
    * @param spatialAcceleration the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialAccelerationString(SpatialAccelerationReadOnly spatialAcceleration)
   {
      return getSpatialAccelerationString(DEFAULT_FORMAT, spatialAcceleration);
   }

   /**
    * Gets a representative {@code String} of {@code spatialAcceleration} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial acceleration of bodyFrame, with respect to baseFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format              the format to use for each number.
    * @param spatialAcceleration the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialAccelerationString(String format, SpatialAccelerationReadOnly spatialAcceleration)
   {
      if (spatialAcceleration == null)
         return "null";
      else
         return getSpatialAccelerationString(format,
                                             spatialAcceleration.getBodyFrame(),
                                             spatialAcceleration.getBaseFrame(),
                                             spatialAcceleration.getReferenceFrame(),
                                             spatialAcceleration.getAngularPart(),
                                             spatialAcceleration.getLinearPart());
   }

   /**
    * Gets a representative {@code String} of {@code spatialAcceleration} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial acceleration of bodyFrame, with respect to baseFrame: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param bodyFrame        what we are specifying the acceleration of.
    * @param baseFrame        with respect to what we are specifying the acceleration.
    * @param expressedInFrame in which reference frame the acceleration is expressed.
    * @param angularPart      the angular part of the spatial acceleration to get the {@code String}
    *                         of. Not modified.
    * @param linearPart       the linear part of the spatial acceleration to get the {@code String} of.
    *                         Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialAccelerationString(String format, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
                                                     Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      return "Spatial acceleration of " + bodyFrame + ", with respect to " + baseFrame + ": [angular = " + getTuple3DString(format, angularPart) + ", linear = "
            + getTuple3DString(format, linearPart) + "] - " + expressedInFrame;
   }

   /**
    * Gets a representative {@code String} of {@code momentum} as follows:
    *
    * <pre>
    * Momentum: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param momentum the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getMomentumString(MomentumReadOnly momentum)
   {
      return getMomentumString(DEFAULT_FORMAT, momentum);
   }

   /**
    * Gets a representative {@code String} of {@code momentum} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Momentum: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format   the format to use for each number.
    * @param momentum the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getMomentumString(String format, MomentumReadOnly momentum)
   {
      if (momentum == null)
         return "null";
      else
         return getMomentumString(format, momentum.getReferenceFrame(), momentum.getAngularPart(), momentum.getLinearPart());
   }

   /**
    * Gets a representative {@code String} of {@code momentum} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Momentum: [angular = ( 0.174,  0.732, -0.222 ), linear = ( 0.174,  0.732, -0.222 )] - expressedInFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param expressedInFrame the reference frame in which the momentum is expressed.
    * @param angularMomentum  the angular part of the momentum to get the {@code String} of. Not
    *                         modified.
    * @param linearMomentum   the linear part of the momentum to get the {@code String} of. Not
    *                         modified.
    * @return the representative {@code String}.
    */
   public static String getMomentumString(String format, ReferenceFrame expressedInFrame, Vector3DReadOnly angularMomentum, Vector3DReadOnly linearMomentum)
   {
      return "Spatial Force Vector: [angular = " + getTuple3DString(format, angularMomentum) + ", linear = " + getTuple3DString(format, linearMomentum) + "] - "
            + expressedInFrame;
   }

   /**
    * Gets a representative {@code String} of {@code spatialInertia} as follows:
    * 
    * <pre>
    * Spatial inertia of bodyFrame expressed in World:
    * / 0.253,  0.000,  0.000,  0.000,  0.554, -0.247 \
    * | 0.000,  0.994,  0.000, -0.554,  0.000,  0.387 |
    * | 0.000,  0.000,  0.084,  0.247, -0.387,  0.000 |
    * | 0.000, -0.554,  0.247,  0.773,  0.000,  0.000 |
    * | 0.554,  0.000, -0.387,  0.000,  0.773,  0.000 |
    * \-0.247,  0.387,  0.000,  0.000,  0.000,  0.773 /
    * </pre>
    * 
    * @param spatialInertia the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialInertiaString(SpatialInertiaReadOnly spatialInertia)
   {
      return getSpatialInertiaString(DEFAULT_FORMAT, spatialInertia);
   }

   /**
    * Gets a representative {@code String} of {@code spatialInertiaMatrix} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial inertia of bodyFrame expressed in World:
    * / 0.253,  0.000,  0.000,  0.000,  0.554, -0.247 \
    * | 0.000,  0.994,  0.000, -0.554,  0.000,  0.387 |
    * | 0.000,  0.000,  0.084,  0.247, -0.387,  0.000 |
    * | 0.000, -0.554,  0.247,  0.773,  0.000,  0.000 |
    * | 0.554,  0.000, -0.387,  0.000,  0.773,  0.000 |
    * \-0.247,  0.387,  0.000,  0.000,  0.000,  0.773 /
    * </pre>
    * </p>
    * 
    * @param format         the format to use for each number.
    * @param spatialInertia the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialInertiaString(String format, SpatialInertiaReadOnly spatialInertia)
   {
      if (spatialInertia == null)
         return "null";
      else
      {
         DenseMatrix64F inertiaMatrix = new DenseMatrix64F(6, 6);
         spatialInertia.get(inertiaMatrix);
         return "Spatial inertia of " + spatialInertia.getBodyFrame() + " expressed in " + spatialInertia.getReferenceFrame() + ":\n"
               + getDenseMatrix64FString(format, inertiaMatrix);
      }
   }

   /**
    * Gets a representative {@code String} of {@code spatialInertia} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Spatial inertia of bodyFrame expressed in World:
    * / 0.253,  0.000,  0.000,  0.000,  0.554, -0.247 \
    * | 0.000,  0.994,  0.000, -0.554,  0.000,  0.387 |
    * | 0.000,  0.000,  0.084,  0.247, -0.387,  0.000 |
    * | 0.000, -0.554,  0.247,  0.773,  0.000,  0.000 |
    * | 0.554,  0.000, -0.387,  0.000,  0.773,  0.000 |
    * \-0.247,  0.387,  0.000,  0.000,  0.000,  0.773 /
    * </pre>
    * </p>
    * 
    * @param format              the format to use for each number.
    * @param bodyFrame           what we are specifying the inertia of.
    * @param expressedInFrame    the reference frame in which the inertia is expressed.
    * @param mass                the mass part of the spatial inertia.
    * @param centerOfMassOffset  the offset from the frame's origin of the center of mass. Not
    *                            modified.
    * @param massMomentOfInertia the mass moment of inertia part. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSpatialInertiaString(String format, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double mass,
                                                Vector3DReadOnly centerOfMassOffset, Matrix3DReadOnly massMomentOfInertia)
   {
      DenseMatrix64F inertiaMatrix = new DenseMatrix64F(6, 6);
      massMomentOfInertia.get(inertiaMatrix);
      MecanoTools.toTildeForm(mass, centerOfMassOffset, false, 0, 3, inertiaMatrix);
      MecanoTools.toTildeForm(mass, centerOfMassOffset, true, 3, 0, inertiaMatrix);
      for (int i = 3; i < 6; i++)
         inertiaMatrix.set(i, i, mass);
      return "Spatial inertia of " + bodyFrame + " expressed in " + expressedInFrame + ":\n" + getDenseMatrix64FString(format, inertiaMatrix);
   }

   /**
    * Gets a representative {@code String} of {@code denseMatrix64F} as follows:
    *
    * <pre>
    * / 0.253,  0.000,  0.000,  0.000,  0.554, -0.247 \
    * | 0.000,  0.994,  0.000, -0.554,  0.000,  0.387 |
    * | 0.000,  0.000,  0.084,  0.247, -0.387,  0.000 |
    * | 0.000, -0.554,  0.247,  0.773,  0.000,  0.000 |
    * | 0.554,  0.000, -0.387,  0.000,  0.773,  0.000 |
    * \-0.247,  0.387,  0.000,  0.000,  0.000,  0.773 /
    * </pre>
    * 
    * @param denseMatrix64F the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getDenseMatrix64FString(DenseMatrix64F denseMatrix64F)
   {
      return getDenseMatrix64FString(DEFAULT_FORMAT, denseMatrix64F);
   }

   /**
    * Gets a representative {@code String} of {@code denseMatrix64F} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * / 0.253,  0.000,  0.000,  0.000,  0.554, -0.247 \
    * | 0.000,  0.994,  0.000, -0.554,  0.000,  0.387 |
    * | 0.000,  0.000,  0.084,  0.247, -0.387,  0.000 |
    * | 0.000, -0.554,  0.247,  0.773,  0.000,  0.000 |
    * | 0.554,  0.000, -0.387,  0.000,  0.773,  0.000 |
    * \-0.247,  0.387,  0.000,  0.000,  0.000,  0.773 /
    * </pre>
    * </p>
    * 
    * @param format         the format to use for each number.
    * @param denseMatrix64F the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getDenseMatrix64FString(String format, DenseMatrix64F denseMatrix64F)
   {
      String ret = "";

      double[] rowValues = new double[denseMatrix64F.getNumCols()];

      String separator = ", ";

      for (int row = 0; row < denseMatrix64F.getNumRows(); row++)
      {
         for (int col = 0; col < denseMatrix64F.getNumCols(); col++)
            rowValues[col] = denseMatrix64F.get(row, col);

         String prefix, suffix;

         if (row == 0)
         {
            prefix = "/";
            suffix = " \\\n";
         }
         else if (row == denseMatrix64F.getNumRows() - 1)
         {
            prefix = "\\";
            suffix = " /";
         }
         else
         {
            prefix = "|";
            suffix = " |\n";
         }

         ret += getStringOf(prefix, suffix, separator, format, rowValues);
      }

      return ret;
   }
}
