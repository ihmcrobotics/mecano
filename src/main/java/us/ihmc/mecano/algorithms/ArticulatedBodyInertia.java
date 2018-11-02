package us.ihmc.mecano.algorithms;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.MecanoFactories;
import us.ihmc.mecano.tools.MecanoIOTools;
import us.ihmc.mecano.tools.MecanoTools;

/**
 * Class representing the articulated-body inertia as introduced in Featherstone - Rigid Body
 * Dynamics Algorithms (2008): <a href=
 * "https://books.google.com/books?id=GJRGBQAAQBAJ&lpg=PR5&ots=XoFXvnJZLH&dq=rigid%20body%20dynamics%20algorithms&lr&pg=PR1#v=onepage&q=rigid%20body%20dynamics%20algorithms&f=false">link</a>.
 * <p>
 * The main purpose of this class is for the {@link ForwardDynamicsCalculator}, classical body
 * inertia representation should use {@link SpatialInertia}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class ArticulatedBodyInertia implements GeometryObject<ArticulatedBodyInertia>, ReferenceFrameHolder, FrameChangeable
{
   /** The frame in which this inertia is expressed. */
   private ReferenceFrame expressedInFrame = null;
   /**
    * The angular part of this inertia, i.e. the 3-by-3 top left block of the 6-by-6 spatial matrix.
    */
   private final Matrix3D angularInertia = new Matrix3D();
   /**
    * The linear part of this inertia, i.e. the 3-by-3 bottom right block of the 6-by-6 spatial
    * matrix.
    */
   private final Matrix3D linearInertia = new Matrix3D();
   /**
    * The cross part of this inertia, i.e. the 3-by-3 top right block of the 6-by-6 spatial matrix.
    */
   private final Matrix3D crossInertia = new Matrix3D();
   /**
    * The transpose of the cross part of this inertia, i.e. the 3-by-3 bottom left block of the
    * 6-by-6 spatial matrix.
    */
   private final Matrix3DReadOnly crossInertiaTranspose = MecanoFactories.createTransposeLinkedMatrix3DReadOnly(crossInertia);
   /** Variable to store intermediate results for garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new inertia with its components set to zero and its reference frame set to
    * {@code null}.
    */
   public ArticulatedBodyInertia()
   {
   }

   /**
    * Creates a new inertia matrix with its components set to zero and initializes its reference
    * frame.
    *
    * @param expressedInFrame in which reference frame the inertia is expressed.
    */
   public ArticulatedBodyInertia(ReferenceFrame expressedInFrame)
   {
      setToZero(expressedInFrame);
   }

   /**
    * Copy constructor.
    *
    * @param other the other inertia to copy. Not modified.
    */
   public ArticulatedBodyInertia(ArticulatedBodyInertia other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      this.expressedInFrame = expressedInFrame;
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return angularInertia.containsNaN() || linearInertia.containsNaN() || crossInertia.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      angularInertia.setToNaN();
      linearInertia.setToNaN();
      crossInertia.setToNaN();
   }

   /**
    * Sets all the coefficients of this inertia to {@link Double#NaN} and sets the reference frame
    * in which it is expressed.
    * 
    * @param expressedInFrame the new reference frame for this inertia.
    */
   public void setToNaN(ReferenceFrame expressedInFrame)
   {
      setReferenceFrame(expressedInFrame);
      setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      angularInertia.setToZero();
      linearInertia.setToZero();
      crossInertia.setToZero();
   }

   /**
    * Sets all the coefficients of this inertia to zero and sets the reference frame in which it is
    * expressed.
    * 
    * @param expressedInFrame the new reference frame for this inertia.
    */
   public void setToZero(ReferenceFrame expressedInFrame)
   {
      setReferenceFrame(expressedInFrame);
      setToZero();
   }

   /**
    * {@inheritDoc}
    * 
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public void set(ArticulatedBodyInertia other)
   {
      checkReferenceFrameMatch(other);
      angularInertia.set(other.getAngularInertia());
      linearInertia.set(other.getLinearInertia());
      crossInertia.set(other.getCrossInertia());
   }

   /**
    * Sets this inertia to {@code other}.
    * 
    * @param other the other inertia to copy values and reference frames from. Not modified.
    */
   public void setIncludingFrame(ArticulatedBodyInertia other)
   {
      setReferenceFrame(other.expressedInFrame);
      angularInertia.set(other.getAngularInertia());
      linearInertia.set(other.getLinearInertia());
      crossInertia.set(other.getCrossInertia());
   }

   /**
    * Sets this inertia to be equivalent to the given {@code spatialInertiaReadOnly}.
    * 
    * @param spatialInertiaReadOnly the spatial inertia to copy. Not modified.
    */
   public void setIncludingFrame(SpatialInertiaReadOnly spatialInertiaReadOnly)
   {
      expressedInFrame = spatialInertiaReadOnly.getReferenceFrame();
      angularInertia.set(spatialInertiaReadOnly.getMomentOfInertia());
      linearInertia.setToZero();
      linearInertia.setM00(spatialInertiaReadOnly.getMass());
      linearInertia.setM11(spatialInertiaReadOnly.getMass());
      linearInertia.setM22(spatialInertiaReadOnly.getMass());
      crossInertia.setToTildeForm(spatialInertiaReadOnly.getCenterOfMassOffset());
      crossInertia.scale(spatialInertiaReadOnly.getMass());
   }

   /**
    * Adds {@code other} to this inertia.
    * 
    * @param other the other inertia to add. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public void add(ArticulatedBodyInertia other)
   {
      checkReferenceFrameMatch(other);
      angularInertia.add(other.getAngularInertia());
      linearInertia.add(other.getLinearInertia());
      crossInertia.add(other.getCrossInertia());
   }

   /**
    * Assuming {@code matrix} represents a proper 6-by-6 inertia matrix, it is added to this
    * inertia.
    * 
    * @param matrix the inertia matrix to add. Not modified.
    */
   public void add(DenseMatrix64F matrix)
   {
      MecanoTools.addEquals(0, 0, matrix, angularInertia);
      MecanoTools.addEquals(3, 3, matrix, linearInertia);
      MecanoTools.addEquals(0, 3, matrix, crossInertia);
   }

   /**
    * Subtracts {@code other} to this inertia.
    * 
    * @param other the other inertia to subtract. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public void sub(ArticulatedBodyInertia other)
   {
      checkReferenceFrameMatch(other);
      angularInertia.sub(other.getAngularInertia());
      linearInertia.sub(other.getLinearInertia());
      crossInertia.sub(other.getCrossInertia());
   }

   /**
    * Assuming {@code matrix} represents a proper 6-by-6 inertia matrix, it is subtracted to this
    * inertia.
    * 
    * @param matrix the inertia matrix to subtract. Not modified.
    */
   public void sub(DenseMatrix64F matrix)
   {
      MecanoTools.subEquals(0, 0, matrix, angularInertia);
      MecanoTools.subEquals(3, 3, matrix, linearInertia);
      MecanoTools.subEquals(0, 3, matrix, crossInertia);
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

   /**
    * Gets the angular part of this articulated-body inertia.
    * <p>
    * In the 6-by-6 inertia matrix, the angular part is the top left 3-by-3 block.
    * </p>
    * 
    * @return the angular part of this inertia.
    */
   public Matrix3D getAngularInertia()
   {
      return angularInertia;
   }

   /**
    * Gets the linear part of this articulated-body inertia.
    * <p>
    * In the 6-by-6 inertia matrix, the linear part is the bottom right 3-by-3 block.
    * </p>
    * 
    * @return the linear part of this inertia.
    */
   public Matrix3D getLinearInertia()
   {
      return linearInertia;
   }

   /**
    * Gets the cross part of this articulated-body inertia.
    * <p>
    * In the 6-by-6 inertia matrix, the cross part is the top right 3-by-3 block.
    * </p>
    * 
    * @return the cross part of this inertia.
    */
   public Matrix3D getCrossInertia()
   {
      return crossInertia;
   }

   /**
    * Gets the transpose of the cross part of this articulated-body inertia.
    * <p>
    * In the 6-by-6 inertia matrix, the transpose of the cross part is the bottom left 3-by-3 block.
    * </p>
    * 
    * @return the transpose of the cross part of this inertia.
    */
   public Matrix3DReadOnly getCrossInertiaTranspose()
   {
      return crossInertiaTranspose;
   }

   /**
    * Gets this articulated-body inertia as a 6-by-6 matrix.
    * 
    * @param matrixToPack the matrix in which this inertia is stored. Modified.
    */
   public void get(DenseMatrix64F matrixToPack)
   {
      angularInertia.get(0, 0, matrixToPack);
      linearInertia.get(3, 3, matrixToPack);

      crossInertia.get(0, 3, matrixToPack);
      crossInertiaTranspose.get(3, 0, matrixToPack);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return expressedInFrame;
   }

   /** {@inheritDoc} */
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

   /**
    * Transforms this articulated-body inertia using the given transform.
    * <p>
    * See the Word&trade; document located in the document folder of this project for more
    * information about the transformation rule for spatial inertia.
    * </p>
    * 
    * @param transform the transform to use on this. Not modified.
    */
   public void applyTransform(RigidBodyTransform transform)
   {
      if (transform.hasRotation())
      {
         RotationMatrixReadOnly rotationMatrix = transform.getRotationMatrix();
         MecanoTools.transformSymmetricMatrix3D(rotationMatrix, angularInertia);
         MecanoTools.transformSymmetricMatrix3D(rotationMatrix, linearInertia);
         rotationMatrix.transform(crossInertia);
      }

      if (transform.hasTranslation())
      {
         Vector3DReadOnly translationVector = transform.getTranslationVector();
         ArticulatedBodyInertiaAlorigthmTools.translateAngularInertia(false, translationVector, linearInertia, crossInertia, angularInertia);
         ArticulatedBodyInertiaAlorigthmTools.translateCrossInertia(false, translationVector, linearInertia, crossInertia);
      }
   }

   /**
    * Transforms this articulated-body inertia by the inverse of the given transform.
    * <p>
    * See the Word&trade; document located in the document folder of this project for more
    * information about the transformation rule for spatial inertia.
    * </p>
    * 
    * @param transform the transform to use on this. Not modified.
    */
   public void applyInverseTransform(RigidBodyTransform transform)
   {
      if (transform.hasTranslation())
      {
         Vector3DReadOnly translationVector = transform.getTranslationVector();
         ArticulatedBodyInertiaAlorigthmTools.translateAngularInertia(true, translationVector, linearInertia, crossInertia, angularInertia);
         ArticulatedBodyInertiaAlorigthmTools.translateCrossInertia(true, translationVector, linearInertia, crossInertia);
      }

      if (transform.hasRotation())
      {
         RotationMatrixReadOnly rotationMatrix = transform.getRotationMatrix();
         MecanoTools.inverseTransformSymmetricMatrix3D(rotationMatrix, angularInertia);
         MecanoTools.inverseTransformSymmetricMatrix3D(rotationMatrix, linearInertia);
         rotationMatrix.inverseTransform(crossInertia);
      }
   }

   /** {@inheritDoc} */
   @Override
   public boolean epsilonEquals(ArticulatedBodyInertia other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!getAngularInertia().epsilonEquals(other.getAngularInertia(), epsilon))
         return false;
      if (!getLinearInertia().epsilonEquals(other.getLinearInertia(), epsilon))
         return false;
      if (!getCrossInertia().epsilonEquals(other.getCrossInertia(), epsilon))
         return false;
      return true;
   }

   /**
    * Checks that {@code other} is expressed in the same reference frame as this and then returns
    * {@link #epsilonEquals(ArticulatedBodyInertia, double)}.
    * 
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   @Override
   public boolean geometricallyEquals(ArticulatedBodyInertia other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return epsilonEquals(other, epsilon);
   }

   /**
    * Returns a representative for this articulated-body inertia as follows:
    * 
    * <pre>
    * Articulated-body inertia expressed in frameName
    * / 0.253,  0.000,  0.000,  0.000,  0.554, -0.247 \
    * | 0.000,  0.994,  0.000, -0.554,  0.000,  0.387 |
    * | 0.000,  0.000,  0.084,  0.247, -0.387,  0.000 |
    * | 0.000, -0.554,  0.247,  0.773,  0.000,  0.000 |
    * | 0.554,  0.000, -0.387,  0.000,  0.773,  0.000 |
    * \-0.247,  0.387,  0.000,  0.000,  0.000,  0.773 /
    * </pre>
    * 
    * @return the representative string for this inertia.
    */
   @Override
   public String toString()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(6, 6);
      get(matrix);
      return "Articulated-body inertia expressed in " + expressedInFrame + "\n" + MecanoIOTools.getDenseMatrix64FString(matrix);
   }
}
