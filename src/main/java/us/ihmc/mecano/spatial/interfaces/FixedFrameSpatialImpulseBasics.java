package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FixedFrameSpatialImpulseBasics extends SpatialImpulseReadOnly, FixedFrameSpatialForceBasics
{
   default void set(SpatialImpulseReadOnly other)
   {
      set(other.getBodyFrame(), other.getReferenceFrame(), other.getAngularPart(), other.getLinearPart());
   }

   default void setMatchingFrame(SpatialImpulseReadOnly other)
   {
      other.checkBodyFrameMatch(getBodyFrame());
      FixedFrameSpatialForceBasics.super.setMatchingFrame(other);
   }

   default void set(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      set(bodyFrame, spatialVector.getReferenceFrame(), spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   default void set(ReferenceFrame bodyFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(linearPart);
      set(bodyFrame, angularPart.getReferenceFrame(), angularPart, linearPart);
   }

   default void set(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      checkReferenceFrameMatch(bodyFrame, expressedInFrame);
      set(angularPart, linearPart);
   }

   default void set(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart,
                    Point3DReadOnly pointOfApplication)
   {
      checkReferenceFrameMatch(bodyFrame, expressedInFrame);
      set(angularPart, linearPart, pointOfApplication);
   }

   default void add(SpatialImpulseReadOnly other)
   {
      add(other.getBodyFrame(), other);
   }

   default void add(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      add(bodyFrame, spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   default void add(ReferenceFrame bodyFrame, FrameVector3DReadOnly angular, FrameVector3DReadOnly linear)
   {
      checkReferenceFrameMatch(bodyFrame, angular.getReferenceFrame());
      checkReferenceFrameMatch(linear);
      add((Vector3DReadOnly) angular, (Vector3DReadOnly) linear);
   }

   default void sub(SpatialImpulseReadOnly other)
   {
      sub(other.getBodyFrame(), other);
   }

   default void sub(ReferenceFrame bodyFrame, SpatialVectorReadOnly spatialVector)
   {
      sub(bodyFrame, spatialVector.getAngularPart(), spatialVector.getLinearPart());
   }

   default void sub(ReferenceFrame bodyFrame, FrameVector3DReadOnly angular, FrameVector3DReadOnly linear)
   {
      checkReferenceFrameMatch(bodyFrame, angular.getReferenceFrame());
      checkReferenceFrameMatch(linear);
      sub((Vector3DReadOnly) angular, (Vector3DReadOnly) linear);
   }
}
