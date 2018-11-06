package us.ihmc.mecano.spatial;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;

public class SpatialImpulse implements SpatialImpulseBasics, GeometryObject<SpatialImpulse>
{
   private ReferenceFrame bodyFrame;
   private final SpatialForce spatialForceVector = new SpatialForce();

   public SpatialImpulse()
   {
      this(null, null);
   }

   public SpatialImpulse(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setToZero(bodyFrame, expressedInFrame);
   }

   public SpatialImpulse(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, angularPart, linearPart);
   }

   public SpatialImpulse(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, matrix);
   }

   public SpatialImpulse(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      setIncludingFrame(bodyFrame, expressedInFrame, array);
   }

   public SpatialImpulse(SpatialImpulseReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(SpatialImpulse other)
   {
      SpatialImpulseBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setBodyFrame(ReferenceFrame bodyFrame)
   {
      this.bodyFrame = bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame expressedInFrame)
   {
      spatialForceVector.setReferenceFrame(expressedInFrame);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return spatialForceVector.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getAngularPart()
   {
      return spatialForceVector.getAngularPart();
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameVector3DBasics getLinearPart()
   {
      return spatialForceVector.getLinearPart();
   }

   /**
    * See {@link SpatialForce#changeFrame(ReferenceFrame)}.
    */
   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      spatialForceVector.changeFrame(desiredFrame);
   }

   /**
    * See {@link SpatialForce#applyTransform(Transform)}.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      spatialForceVector.applyTransform(transform);
   }

   /**
    * See {@link SpatialForce#applyInverseTransform(Transform)}.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      spatialForceVector.applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(SpatialImpulse other, double epsilon)
   {
      return SpatialImpulseBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(SpatialImpulse other, double epsilon)
   {
      return SpatialImpulseBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof WrenchReadOnly)
         return SpatialImpulseBasics.super.equals((WrenchReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return MecanoIOTools.getSpatialImpulseString(this);
   }
}
