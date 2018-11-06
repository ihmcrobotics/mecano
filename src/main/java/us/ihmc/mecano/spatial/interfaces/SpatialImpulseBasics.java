package us.ihmc.mecano.spatial.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface SpatialImpulseBasics extends FixedFrameSpatialImpulseBasics, SpatialForceBasics
{
   void setBodyFrame(ReferenceFrame bodyFrame);

   default void setToZero(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setBodyFrame(bodyFrame);
      setToZero(expressedInFrame);
   }

   default void setToNaN(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame)
   {
      setBodyFrame(bodyFrame);
      setToNaN(expressedInFrame);
   }

   default void setIncludingFrame(SpatialImpulseReadOnly other)
   {
      setIncludingFrame(other.getBodyFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(spatialVector);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(angularPart, linearPart);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, angularPart, linearPart);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, array);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, int startIndex, double[] array)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, startIndex, array);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, float[] array)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, array);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, int startIndex, float[] array)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, startIndex, array);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, matrix);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, int startRow, DenseMatrix64F matrix)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, startRow, matrix);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, int startRow, int column, DenseMatrix64F matrix)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, startRow, column, matrix);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart,
                                  Point3DReadOnly pointOfApplication)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(expressedInFrame, angularPart, linearPart, pointOfApplication);
   }

   default void setIncludingFrame(ReferenceFrame bodyFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart,
                                  FramePoint3DReadOnly pointOfApplication)
   {
      setBodyFrame(bodyFrame);
      setIncludingFrame(angularPart, linearPart, pointOfApplication);
   }
}
