package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

public interface SpatialImpulseReadOnly extends SpatialForceReadOnly
{
   ReferenceFrame getBodyFrame();

   default double dot(TwistReadOnly twist)
   {
      return twist.dot(this);
   }

   default void checkReferenceFrameMatch(SpatialImpulseReadOnly other) throws ReferenceFrameMismatchException
   {
      checkReferenceFrameMatch(other.getBodyFrame(), other.getReferenceFrame());
   }

   default void checkReferenceFrameMatch(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame) throws ReferenceFrameMismatchException
   {
      checkBodyFrameMatch(bodyFrame);
      checkExpressedInFrameMatch(expressedInFrame);
   }

   default void checkBodyFrameMatch(ReferenceFrame bodyFrame)
   {
      if (getBodyFrame() != bodyFrame)
         throw new ReferenceFrameMismatchException("bodyFrame mismatch: this.bodyFrame = " + getBodyFrame() + ", other bodyFrame = " + bodyFrame);
   }

   default void checkExpressedInFrameMatch(ReferenceFrame expressedInFrame)
   {
      if (getReferenceFrame() != expressedInFrame)
         throw new ReferenceFrameMismatchException("expressedInFrame mismatch: this.expressedInFrame = " + getReferenceFrame() + ", other expressedInFrame = "
               + expressedInFrame);
   }

   default boolean epsilonEquals(SpatialImpulseReadOnly other, double epsilon)
   {
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      return SpatialForceReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(SpatialImpulseReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      if (!getAngularPart().geometricallyEquals(getAngularPart(), epsilon))
         return false;
      if (!getLinearPart().geometricallyEquals(getLinearPart(), epsilon))
         return false;
      return true;
   }

   default boolean equals(SpatialImpulseReadOnly other)
   {
      if (other == null)
         return false;
      if (getBodyFrame() != other.getBodyFrame())
         return false;
      return SpatialForceReadOnly.super.equals(other);
   }
}
