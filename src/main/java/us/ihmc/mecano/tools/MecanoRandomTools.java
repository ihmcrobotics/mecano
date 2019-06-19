package us.ihmc.mecano.tools;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.*;

import java.util.Random;
import java.util.stream.IntStream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;

/**
 * This class provides random generators to generate random spatial vectors.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class MecanoRandomTools
{
   /**
    * Generates a moving reference frame with random transform and twist to world frame.
    *
    * @param random the random generator to use.
    * @return the new random moving reference frame.
    */
   public static MovingReferenceFrame nextMovingReferenceFrame(Random random)
   {
      return nextMovingReferenceFrame(random, false);
   }

   /**
    * Generates a moving reference frame with random transform and twist to world frame.
    *
    * @param random         the random generator to use.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *                       random frame.
    * @return the new random moving reference frame.
    */
   public static MovingReferenceFrame nextMovingReferenceFrame(Random random, boolean use2DTransform)
   {
      return nextMovingReferenceFrame(random, ReferenceFrame.getWorldFrame(), use2DTransform);
   }

   /**
    * Generates a moving reference frame with random transform and twist to its parent frame.
    *
    * @param random      the random generator to use.
    * @param parentFrame the parent frame of the new moving reference frame.
    * @return the new random moving reference frame.
    */
   public static MovingReferenceFrame nextMovingReferenceFrame(Random random, ReferenceFrame parentFrame)
   {
      return nextMovingReferenceFrame(random, parentFrame, false);
   }

   /**
    * Generates a moving reference frame with random transform and twist to its parent frame.
    *
    * @param random         the random generator to use.
    * @param parentFrame    the parent frame of the new moving reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *                       random frame.
    * @return the new random moving reference frame.
    */
   public static MovingReferenceFrame nextMovingReferenceFrame(Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      return nextMovingReferenceFrame("randomFrame" + random.nextInt(), random, parentFrame, use2DTransform);
   }

   /**
    * Generates a moving reference frame with random transform and twist to its parent frame.
    *
    * @param frameName   the name of the new frame.
    * @param random      the random generator to use.
    * @param parentFrame the parent frame of the new moving reference frame.
    * @return the new random moving reference frame.
    */
   public static MovingReferenceFrame nextMovingReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame)
   {
      return nextMovingReferenceFrame(frameName, random, parentFrame, false);
   }

   /**
    * Generates a moving reference frame with random transform and twist to its parent frame.
    *
    * @param frameName      the name of the new frame.
    * @param random         the random generator to use.
    * @param parentFrame    the parent frame of the new moving reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *                       random frame.
    * @return the new random moving reference frame.
    */
   public static MovingReferenceFrame nextMovingReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      RigidBodyTransform transform;
      Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D linearVelocity = EuclidCoreRandomTools.nextVector3D(random);

      if (use2DTransform)
      {
         transform = EuclidCoreRandomTools.nextRigidBodyTransform2D(random);
         angularVelocity.setX(0.0);
         angularVelocity.setY(0.0);
         linearVelocity.setZ(0.0);
      }
      else
      {
         transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      }
      return new MovingReferenceFrame(frameName, parentFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transform);
         }

         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            twistRelativeToParentToPack.set(angularVelocity, linearVelocity);
         }
      };
   }

   /**
    * Creates a tree structure of 20 random moving reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random the random generator to use.
    * @return the array containing the random moving reference frames.
    */
   public static MovingReferenceFrame[] nextMovingReferenceFrameTree(Random random)
   {
      return nextMovingReferenceFrameTree(random, false);
   }

   /**
    * Creates a tree structure of 20 random moving reference frames start off
    * {@link ReferenceFrame#getWorldFrame()}.
    * 
    * @param random          the random generator to use.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *                        random frames.
    * @return the array containing the random moving reference frames.
    */
   public static MovingReferenceFrame[] nextMovingReferenceFrameTree(Random random, boolean use2DTransforms)
   {
      return nextMovingReferenceFrameTree(random, 20, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random                        the random generator to use.
    * @param numberOfMovingReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frames.
    */
   public static MovingReferenceFrame[] nextMovingReferenceFrameTree(Random random, int numberOfMovingReferenceFrames)
   {
      return nextMovingReferenceFrameTree(random, numberOfMovingReferenceFrames, false);
   }

   /**
    * Creates a tree structure of random moving reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random                        the random generator to use.
    * @param numberOfMovingReferenceFrames the number of moving reference frames to be created.
    * @param use2DTransforms               whether to use a 2D or 3D rotation for the transform used to
    *                                      create the random frames.
    * @return the array containing the random moving reference frames.
    */
   public static MovingReferenceFrame[] nextMovingReferenceFrameTree(Random random, int numberOfMovingReferenceFrames, boolean use2DTransforms)
   {
      return nextMovingReferenceFrameTree("randomFrame", random, ReferenceFrame.getWorldFrame(), numberOfMovingReferenceFrames, use2DTransforms);
   }

   /**
    * Creates a tree structure of random moving reference frames starting off the given
    * {@code rootFrame}.
    *
    * @param frameNamePrefix               prefix to use when creating each random moving reference
    *                                      frame.
    * @param random                        the random generator to use.
    * @param rootFrame                     the base frame from which the tree is to be expanded.
    * @param numberOfMovingReferenceFrames the number of moving reference frames to be created.
    * @return the array containing the random moving reference frames.
    */
   public static MovingReferenceFrame[] nextMovingReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame,
                                                                     int numberOfMovingReferenceFrames)
   {
      return nextMovingReferenceFrameTree(frameNamePrefix, random, rootFrame, numberOfMovingReferenceFrames, false);
   }

   /**
    * Creates a tree structure of random moving reference frames starting off the given
    * {@code rootFrame}.
    *
    * @param frameNamePrefix               prefix to use when creating each random moving reference
    *                                      frame.
    * @param random                        the random generator to use.
    * @param rootFrame                     the base frame from which the tree is to be expanded.
    * @param numberOfMovingReferenceFrames the number of moving reference frames to be created.
    * @param use2DTransforms               whether to use a 2D or 3D rotation for the transform used to
    *                                      create the random frames.
    * @return the array containing the random moving reference frames.
    */
   public static MovingReferenceFrame[] nextMovingReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame,
                                                                     int numberOfMovingReferenceFrames, boolean use2DTransforms)
   {
      ReferenceFrame[] referenceFrames = new ReferenceFrame[numberOfMovingReferenceFrames + 1];
      MovingReferenceFrame[] movingReferenceFrames = new MovingReferenceFrame[numberOfMovingReferenceFrames];
      referenceFrames[0] = rootFrame;

      for (int i = 0; i < numberOfMovingReferenceFrames; i++)
      {
         int parentFrameIndex = random.nextInt(i + 1);
         ReferenceFrame parentFrame = referenceFrames[parentFrameIndex];
         MovingReferenceFrame nextMovingReferenceFrame = nextMovingReferenceFrame(frameNamePrefix + i, random, parentFrame, use2DTransforms);
         referenceFrames[i + 1] = nextMovingReferenceFrame;
         movingReferenceFrames[i] = nextMovingReferenceFrame;
      }

      return movingReferenceFrames;
   }

   /**
    * Generates an array filled with random vectors with fixed length.
    * 
    * @param random        the random generator to use.
    * @param arrayLength   the length of the array to generate.
    * @param vectorsLength the length each random vector should have.
    * @return the array of random vectors.
    */
   public static Vector3D[] nextVector3DArray(Random random, int arrayLength, double vectorsLength)
   {
      return IntStream.range(0, arrayLength).mapToObj(i -> EuclidCoreRandomTools.nextVector3DWithFixedLength(random, vectorsLength)).toArray(Vector3D[]::new);
   }

   /**
    * Generates a random spatial vector.
    * <p>
    * {@code spatialVector}<sub>i=0,5</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param expressedInFrame the frame in which the generated vector is expressed.
    * @return the random spatial vector.
    */
   public static SpatialVector nextSpatialVector(Random random, ReferenceFrame expressedInFrame)
   {
      return nextSpatialVector(random, expressedInFrame, 1.0, 1.0);
   }

   /**
    * Generates a random spatial vector.
    * <p>
    * {@code spatialVector}<sub>i=0,2</sub> &in; [-{@code angularPartMinMax};
    * {@code angularPartMinMax}].<br>
    * {@code spatialVector}<sub>i=3,5</sub> &in; [-{@code linearPartMinMax}; {@code linearPartMinMax}].
    * </p>
    *
    * @param random            the random generator to use.
    * @param expressedInFrame  the frame in which the generated vector is expressed.
    * @param angularPartMinMax the maximum absolute value for each component of the angular part.
    * @param linearPartMinMax  the maximum absolute value for each component of the linear part.
    * @return the random spatial vector.
    */
   public static SpatialVector nextSpatialVector(Random random, ReferenceFrame expressedInFrame, double angularPartMinMax, double linearPartMinMax)
   {
      return new SpatialVector(expressedInFrame,
                               nextVector3D(random, -angularPartMinMax, angularPartMinMax),
                               nextVector3D(random, -linearPartMinMax, linearPartMinMax));
   }

   /**
    * Generates a random spatial force vector.
    * <p>
    * {@code spatialForceVector}<sub>i=0,5</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param expressedInFrame the frame in which the generated vector is expressed.
    * @return the random spatial force vector.
    */
   public static SpatialForce nextSpatialForce(Random random, ReferenceFrame expressedInFrame)
   {
      return nextSpatialForce(random, expressedInFrame, 1.0, 1.0);
   }

   /**
    * Generates a random spatial force vector.
    * <p>
    * {@code spatialForceVector}<sub>i=0,2</sub> &in; [-{@code angularPartMinMax};
    * {@code angularPartMinMax}].<br>
    * {@code spatialForceVector}<sub>i=3,5</sub> &in; [-{@code linearPartMinMax};
    * {@code linearPartMinMax}].
    * </p>
    *
    * @param random            the random generator to use.
    * @param expressedInFrame  the frame in which the generated vector is expressed.
    * @param angularPartMinMax the maximum absolute value for each component of the angular part.
    * @param linearPartMinMax  the maximum absolute value for each component of the linear part.
    * @return the random spatial force vector.
    */
   public static SpatialForce nextSpatialForce(Random random, ReferenceFrame expressedInFrame, double angularPartMinMax, double linearPartMinMax)
   {
      return new SpatialForce(expressedInFrame,
                              nextVector3D(random, -angularPartMinMax, angularPartMinMax),
                              nextVector3D(random, -linearPartMinMax, linearPartMinMax));
   }

   /**
    * Generates a random wrench.
    * <p>
    * {@code wrench}<sub>i=0,5</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param bodyFrame        the frame of the body on which the wrench is exerted.
    * @param expressedInFrame the frame in which the generated vector is expressed.
    * @return the random wrench.
    */
   public static Wrench nextWrench(Random random, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      return nextWrench(random, bodyFrame, expressedInFrame, 1.0, 1.0);
   }

   /**
    * Generates a random wrench.
    * <p>
    * {@code wrench}<sub>i=0,2</sub> &in; [-{@code angularPartMinMax}; {@code angularPartMinMax}].<br>
    * {@code wrench}<sub>i=3,5</sub> &in; [-{@code linearPartMinMax}; {@code linearPartMinMax}].
    * </p>
    *
    * @param random            the random generator to use.
    * @param bodyFrame         the frame of the body on which the wrench is exerted.
    * @param expressedInFrame  the frame in which the generated vector is expressed.
    * @param angularPartMinMax the maximum absolute value for each component of the angular part.
    * @param linearPartMinMax  the maximum absolute value for each component of the linear part.
    * @return the random wrench.
    */
   public static Wrench nextWrench(Random random, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double angularPartMinMax, double linearPartMinMax)
   {
      return new Wrench(bodyFrame,
                        expressedInFrame,
                        nextVector3D(random, -angularPartMinMax, angularPartMinMax),
                        nextVector3D(random, -linearPartMinMax, linearPartMinMax));
   }

   /**
    * Generates a random twist.
    * <p>
    * {@code twist}<sub>i=0,5</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param bodyFrame        what we are specifying the twist of.
    * @param baseFrame        with respect to what we are specifying the twist.
    * @param expressedInFrame in which reference frame the twist is expressed.
    * @return the random twist.
    */
   public static Twist nextTwist(Random random, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      return nextTwist(random, bodyFrame, baseFrame, expressedInFrame, 1.0, 1.0);
   }

   /**
    * Generates a random twist.
    * <p>
    * {@code twist}<sub>i=0,2</sub> &in; [-{@code angularPartMinMax}; {@code angularPartMinMax}].<br>
    * {@code twist}<sub>i=3,5</sub> &in; [-{@code linearPartMinMax}; {@code linearPartMinMax}].
    * </p>
    *
    * @param random            the random generator to use.
    * @param bodyFrame         what we are specifying the twist of.
    * @param baseFrame         with respect to what we are specifying the twist.
    * @param expressedInFrame  in which reference frame the twist is expressed.
    * @param angularPartMinMax the maximum absolute value for each component of the angular part.
    * @param linearPartMinMax  the maximum absolute value for each component of the linear part.
    * @return the random twist.
    */
   public static Twist nextTwist(Random random, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, double angularPartMinMax,
                                 double linearPartMinMax)
   {
      return new Twist(bodyFrame,
                       baseFrame,
                       expressedInFrame,
                       nextVector3D(random, -linearPartMinMax, linearPartMinMax),
                       nextVector3D(random, -angularPartMinMax, angularPartMinMax));
   }

   /**
    * Generates a random spatial acceleration vector.
    * <p>
    * {@code spatialAccelerationVector}<sub>i=0,5</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param bodyFrame        what we are specifying the acceleration of.
    * @param baseFrame        with respect to what we are specifying the acceleration.
    * @param expressedInFrame in which reference frame the acceleration is expressed.
    * @return the random spatial acceleration vector.
    */
   public static SpatialAcceleration nextSpatialAcceleration(Random random, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      return nextSpatialAcceleration(random, bodyFrame, baseFrame, expressedInFrame, 1.0, 1.0);
   }

   /**
    * Generates a random spatial acceleration vector.
    * <p>
    * {@code spatialAccelerationVector}<sub>i=0,2</sub> &in; [-{@code angularPartMinMax};
    * {@code angularPartMinMax}].<br>
    * {@code spatialAccelerationVector}<sub>i=3,5</sub> &in; [-{@code linearPartMinMax};
    * {@code linearPartMinMax}].
    * </p>
    *
    * @param random            the random generator to use.
    * @param bodyFrame         what we are specifying the acceleration of.
    * @param baseFrame         with respect to what we are specifying the acceleration.
    * @param expressedInFrame  in which reference frame the acceleration is expressed.
    * @param angularPartMinMax the maximum absolute value for each component of the angular part.
    * @param linearPartMinMax  the maximum absolute value for each component of the linear part.
    * @return the random spatial acceleration vector.
    */
   public static SpatialAcceleration nextSpatialAcceleration(Random random, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame,
                                                             double angularPartMinMax, double linearPartMinMax)
   {
      return new SpatialAcceleration(bodyFrame,
                                     baseFrame,
                                     expressedInFrame,
                                     nextVector3D(random, -angularPartMinMax, angularPartMinMax),
                                     nextVector3D(random, -linearPartMinMax, linearPartMinMax));
   }

   /**
    * Generates a random momentum.
    * <p>
    * {@code momentum}<sub>i=0,5</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param expressedInFrame the frame in which the generated vector is expressed.
    * @return the random momentum.
    */
   public static Momentum nextMomentum(Random random, ReferenceFrame expressedInFrame)
   {
      return nextMomentum(random, expressedInFrame, 1.0, 1.0);
   }

   /**
    * Generates a random momentum.
    * <p>
    * {@code momentum}<sub>i=0,2</sub> &in; [-{@code angularPartMinMax};
    * {@code angularPartMinMax}].<br>
    * {@code momentum}<sub>i=3,5</sub> &in; [-{@code linearPartMinMax}; {@code linearPartMinMax}].
    * </p>
    *
    * @param random            the random generator to use.
    * @param expressedInFrame  the frame in which the generated vector is expressed.
    * @param angularPartMinMax the maximum absolute value for each component of the angular part.
    * @param linearPartMinMax  the maximum absolute value for each component of the linear part.
    * @return the random momentum.
    */
   public static Momentum nextMomentum(Random random, ReferenceFrame expressedInFrame, double angularPartMinMax, double linearPartMinMax)
   {
      return new Momentum(expressedInFrame,
                          nextVector3D(random, -angularPartMinMax, angularPartMinMax),
                          nextVector3D(random, -linearPartMinMax, linearPartMinMax));
   }

   /**
    * Generates a random spatial inertia matrix.
    * <p>
    * {@code mass} &in; [0; 1.0]<br>
    * {@code massMomemtOfInertia}(i, i) &in; [0; 1.0], &forall; i & [0,2].<br>
    * {@code centerOfMassOffset}<sub>i=0,2</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random           the random generator to use.
    * @param bodyFrame        what we are specifying the inertia of.
    * @param expressedInFrame in which reference frame the inertia is expressed.
    * @return the random spatial inertia matrix.
    */
   public static SpatialInertia nextSpatialInertia(Random random, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      return nextSpatialInertia(random, bodyFrame, expressedInFrame, 1.0, 1.0, 1.0);
   }

   /**
    * Generates a random spatial inertia matrix.
    * <p>
    * {@code mass} &in; [0; {@code massMax}]<br>
    * {@code massMomemtOfInertia}(i, i) &in; [0; {@code inertiaMax}], &forall; i & [0,2].<br>
    * {@code centerOfMassOffset}<sub>i=0,2</sub> &in; [-{@code centerOfMassOffsetMinMax};
    * {@code centerOfMassOffsetMinMax}].
    * </p>
    *
    * @param random                   the random generator to use.
    * @param bodyFrame                what we are specifying the inertia of.
    * @param expressedInFrame         in which reference frame the inertia is expressed.
    * @param inertiaMax               the maximum positive value for each coefficients of the mass
    *                                 moment of inertia's diagonal.
    * @param massMax                  the maximum positive value for the mass.
    * @param centerOfMassOffsetMinMax the maximum absolute value for each coordinate of the center of
    *                                 mass offset.
    * @return the random spatial inertia matrix.
    */
   public static SpatialInertia nextSpatialInertia(Random random, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double inertiaMax, double massMax,
                                                   double centerOfMassOffsetMinMax)
   {
      if (inertiaMax < 0.0)
         throw new IllegalArgumentException("The inertia cannot be negative.");
      if (massMax < 0.0)
         throw new IllegalArgumentException("The mass cannot be negative.");

      return new SpatialInertia(bodyFrame,
                                expressedInFrame,
                                nextDiagonalMatrix3D(random, 0.0, inertiaMax),
                                massMax * random.nextDouble(),
                                EuclidCoreRandomTools.nextPoint3D(random, centerOfMassOffsetMinMax));
   }

   /**
    * Generates a random symmetric matrix 3D which eigen values are all real and strictly positive.
    * 
    * @param random the random generator to use.
    * @return the random symmetric and positive definite matrix.
    */
   public static Matrix3D nextSymmetricPositiveDefiniteMatrix3D(Random random)
   {
      return nextSymmetricPositiveDefiniteMatrix3D(random, 1.0e-6, 2.0, 0.5);
   }

   /**
    * Generates a random symmetric matrix 3D which eigen values are all real and strictly positive.
    * 
    * @param random            the random generator to use.
    * @param minDiagonal       the minimum value for each element on the diagonal.
    * @param maxDiagonal       the maximum value for each element on the diagonal.
    * @param minMaxOffDiagonal the maximum absolute value for each element that is not on the diagonal.
    * @return the random symmetric and positive definite matrix.
    * @throws RuntimeException if {@code minDiagonal > maxDiagonal}, if {@code minDiagonal} is
    *                          negative, or if {@code minMaxOffDiagonal} is negative.
    */
   public static Matrix3D nextSymmetricPositiveDefiniteMatrix3D(Random random, double minDiagonal, double maxDiagonal, double minMaxOffDiagonal)
   {
      if (minDiagonal < 0.0)
         throw new RuntimeException("minDiagonal has to be positive.");
      // Wikipedia: a Hermitian (special case: symmetric) matrix is positive definite if:
      // There exists a unique lower triangular matrix L, with strictly positive diagonal elements, that allows the factorization of M into M = LL*.

      Matrix3D lowerTriangular = new Matrix3D();
      lowerTriangular.setM00(EuclidCoreRandomTools.nextDouble(random, minDiagonal, maxDiagonal));

      lowerTriangular.setM10(EuclidCoreRandomTools.nextDouble(random, minMaxOffDiagonal));
      lowerTriangular.setM11(EuclidCoreRandomTools.nextDouble(random, minDiagonal, maxDiagonal));

      lowerTriangular.setM20(EuclidCoreRandomTools.nextDouble(random, minMaxOffDiagonal));
      lowerTriangular.setM21(EuclidCoreRandomTools.nextDouble(random, minMaxOffDiagonal));
      lowerTriangular.setM22(EuclidCoreRandomTools.nextDouble(random, minDiagonal, maxDiagonal));

      Matrix3D symmetricPositiveDefiniteMatrix3D = new Matrix3D(lowerTriangular);
      symmetricPositiveDefiniteMatrix3D.multiplyTransposeOther(lowerTriangular);

      MecanoTools.checkIfMatrix3DIsSymmetric(symmetricPositiveDefiniteMatrix3D, 1.0e-8);
      checkEigenValuesRealAndPositive(symmetricPositiveDefiniteMatrix3D, 1e-10);

      return symmetricPositiveDefiniteMatrix3D;
   }

   private static void checkEigenValuesRealAndPositive(Matrix3D matrix, double epsilon)
   {
      DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
      matrix.get(denseMatrix);
      EigenDecomposition<DenseMatrix64F> eig = DecompositionFactory.eig(3, false);
      eig.decompose(denseMatrix);

      for (int i = 0; i < eig.getNumberOfEigenvalues(); i++)
      {
         double eigImaginaryPart = eig.getEigenvalue(i).getImaginary();
         double eigRealPart = eig.getEigenvalue(i).getReal();

         if (!EuclidCoreTools.epsilonEquals(0.0, eigImaginaryPart, epsilon))
            throw new RuntimeException("The matrix has at least one imaginary eigen value.");
         if (eigRealPart <= 0.0)
            throw new RuntimeException("The matrix has at least one non-positive eigen value.");
      }
   }
}
