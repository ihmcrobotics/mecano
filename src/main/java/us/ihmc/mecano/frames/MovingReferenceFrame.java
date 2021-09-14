package us.ihmc.mecano.frames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.exceptions.ScrewTheoryException;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

/**
 * In addition of having a defined pose in space as a {@code ReferenceFrame}, a
 * {@code MovingReferenceFrame} holds onto its velocity relative to its parent allowing to compute
 * its velocity with respect to world or another {@code MovingReferenceFrame}.
 * <p>
 * The parent of a {@code MovingReferenceFrame} can either be another {@code MovingReferenceFrame}
 * or a stationary {@code ReferenceFrame}, i.e. not moving with respect to its root frame.
 * </p>
 */
public abstract class MovingReferenceFrame extends ReferenceFrame
{
   /**
    * Dirty bit used to mark {@link #twistOfFrame} as out-of-date or up-to-date and allow to save some
    * computation. The twist is marked as out-of-date upon calling {@link #update()}.
    */
   private boolean isTwistOfFrameUpToDate = false;
   /**
    * This is the data that needs to be provided from the user side.
    */
   private final Twist twistRelativeToParent;
   /**
    * This the computed twist of this frame relative to the root moving frame, i.e. the ancestor frame
    * that was created with a stationary {@code ReferenceFrame} as a parent.
    */
   private final Twist twistOfFrame = new Twist();

   /**
    * The parent moving reference frame.
    * <p>
    * It is {@code null} when this frame's parent is a stationary {@code ReferenceFrame}.
    * </p>
    */
   private final MovingReferenceFrame closestAncestorMovingFrame;
   /**
    * List containing the descendants from this frame that this frame has to notify when their twist is
    * out-of-date.
    */
   private final List<MovingReferenceFrame> descendantsMovingFrames = new ArrayList<>();

   /**
    * Constructs a {@code MovingReferenceFrame} that has a 'zero-velocity' with respect to its parent.
    * <p>
    * The {@code transformToParent} should describe the pose of the new frame expressed in its parent
    * frame.
    * </p>
    *
    * @param frameName         the name of the new frame.
    * @param parentFrame       the parent of the new frame. It has to be either another
    *                          {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame},
    *                          i.e. not moving with respect to its root frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *                          frame to its parent frame. Not modified.
    * @return the new moving reference frame.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor a
    *                              stationary {@code ReferenceFrame}.
    */
   public static MovingReferenceFrame constructFrameFixedInParent(String frameName, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      return new FixedMovingReferenceFrame(frameName, parentFrame, transformToParent);
   }

   /**
    * Creates a new moving reference frame as a child of the given {@code parentFrame}.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose and velocity with respect to the {@code parentFrame} can be modified at runtime by
    * changing the transform and twist in the methods
    * {@link #updateTransformToParent(RigidBodyTransform)} and
    * {@link #updateTwistRelativeToParent(Twist)} when overriding them.
    * </p>
    * <p>
    * This frame is not expected to have its z-axis aligned at all time with the z-axis of the root
    * frame.
    * </p>
    *
    * @param frameName   the name of the new frame.
    * @param parentFrame the parent of the new frame. It has to be either another
    *                    {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame}, i.e. not
    *                    moving with respect to its root frame.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor a
    *                              stationary {@code ReferenceFrame}.
    */
   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      this(frameName, parentFrame, null, false, false);
   }

   /**
    * Creates a new moving reference frame as a child of the given {@code parentFrame} .
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose and velocity with respect to the {@code parentFrame} can be modified at runtime by
    * changing the transform and twist in the methods
    * {@link #updateTransformToParent(RigidBodyTransform)} and
    * {@link #updateTwistRelativeToParent(Twist)} when overriding them.
    * </p>
    *
    * @param frameName   the name of the new frame.
    * @param parentFrame the parent of the new frame. It has to be either another
    *                    {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame}, i.e. not
    *                    moving with respect to its root frame.
    * @param isZUpFrame  refers to whether this new frame has its z-axis aligned with the root frame at
    *                    all time or not.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor a
    *                              stationary {@code ReferenceFrame}.
    */
   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isZUpFrame)
   {
      this(frameName, parentFrame, new RigidBodyTransform(), isZUpFrame, false);
   }

   /**
    * Creates a new moving reference frame as a child of the given {@code parentFrame} and initializes
    * the transform to its parent.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose and velocity with respect to the {@code parentFrame} can be modified at runtime by
    * changing the transform and twist in the methods
    * {@link #updateTransformToParent(RigidBodyTransform)} and
    * {@link #updateTwistRelativeToParent(Twist)} when overriding them.
    * </p>
    * <p>
    * This frame is not expected to have its z-axis aligned at all time with the z-axis of the root
    * frame.
    * </p>
    *
    * @param frameName         the name of the new frame.
    * @param parentFrame       the parent of the new frame. It has to be either another
    *                          {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame},
    *                          i.e. not moving with respect to its root frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *                          frame to its parent frame. Not modified.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor a
    *                              stationary {@code ReferenceFrame}.
    */
   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      this(frameName, parentFrame, transformToParent, false, false);
   }

   /**
    * Creates a new moving reference frame as a child of the given {@code parentFrame} and initializes
    * the transform to its parent.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose and velocity with respect to the {@code parentFrame} can be modified at runtime by
    * changing the transform and twist in the methods
    * {@link #updateTransformToParent(RigidBodyTransform)} and
    * {@link #updateTwistRelativeToParent(Twist)} when overriding them.
    * </p>
    *
    * @param frameName         the name of the new frame.
    * @param parentFrame       the parent of the new frame. It has to be either another
    *                          {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame},
    *                          i.e. not moving with respect to its root frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *                          frame to its parent frame. Not modified.
    * @param isZUpFrame        refers to whether this new frame has its z-axis aligned with the root
    *                          frame at all time or not.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor a
    *                              stationary {@code ReferenceFrame}.
    */
   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent, boolean isZUpFrame)
   {
      this(frameName, parentFrame, transformToParent, isZUpFrame, false);
   }

   protected MovingReferenceFrame(String frameName,
                                  ReferenceFrame parentFrame,
                                  RigidBodyTransformReadOnly transformToParent,
                                  boolean isZUpFrame,
                                  boolean isFixedInParent)
   {
      super(frameName, parentFrame, transformToParent, parentFrame.isAStationaryFrame() && isFixedInParent, isZUpFrame, isFixedInParent);

      closestAncestorMovingFrame = findClosestAncestorMovingFrame(parentFrame);
      if (closestAncestorMovingFrame != null)
         closestAncestorMovingFrame.descendantsMovingFrames.add(this);

      if (!isAncestorValid(parentFrame))
         throw unhandledReferenceFrameTypeException(parentFrame);

      if (isFixedInParent)
         twistRelativeToParent = null;
      else
         twistRelativeToParent = new Twist(this, parentFrame, this);
   }

   private static MovingReferenceFrame findClosestAncestorMovingFrame(ReferenceFrame frame)
   {
      if (frame == null)
         return null;
      else if (frame instanceof MovingReferenceFrame)
         return (MovingReferenceFrame) frame;
      else
         return findClosestAncestorMovingFrame(frame.getParent());
   }

   private static boolean isAncestorValid(ReferenceFrame frame)
   {
      if (frame instanceof MovingReferenceFrame)
         return true;
      if (frame.isAStationaryFrame())
         return true;
      if (frame.isRootFrame())
         return true;
      if (frame.isFixedInParent())
         return isAncestorValid(frame.getParent());
      return false;
   }

   /**
    * In addition to performing {@code ReferenceFrame.update()}, it also marks the twist of this
    * reference frame as out-of-date such that it will be updated next time it is needed.
    * <p>
    * From {@code ReferenceFrame}:<br>
    * {@inheritDoc}
    * </p>
    *
    * @throws ReferenceFrameMismatchException if the twist set in
    *                                         {@link #updateTwistRelativeToParent(Twist)} is not
    *                                         expressed with the proper frames, see
    *                                         {@link #updateTwistRelativeToParent(Twist)}.
    */
   @Override
   public void update()
   {
      super.update();

      if (!isFixedInParent())
      {
         updateTwistRelativeToParent(twistRelativeToParent);
         twistRelativeToParent.checkReferenceFrameMatch(this, getParent(), this);
      }

      isTwistOfFrameUpToDate = false;
   }

   /**
    * Override this method to define what is the velocity of this moving reference frame relative to
    * its parent frame over time by setting the argument {@code twistRelativeToParentToPack}.
    * <p>
    * The frames of the {@code twistRelativeToParentToPack} should be as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code this}.
    * <li>{@code baseFrame} is {@code parentFrame}.
    * <li>{@code expressedInFrame} is {@code this}.
    * </ul>
    * </p>
    *
    * @param twistRelativeToParentToPack the transform to updated according to how this reference frame
    *                                    should now positioned with respect to its parent frame.
    *                                    Modified.
    */
   protected abstract void updateTwistRelativeToParent(Twist twistRelativeToParentToPack);

   private void updateTwistOfFrame()
   {
      if (isTwistOfFrameUpToDateRecursive())
         return;

      if (closestAncestorMovingFrame == null)
      {
         if (isFixedInParent())
            twistOfFrame.setToZero(this, getParent(), this);
         else
            twistOfFrame.setIncludingFrame(twistRelativeToParent);
      }
      else
      {
         twistOfFrame.setIncludingFrame(closestAncestorMovingFrame.getTwistOfFrame());
         twistOfFrame.changeFrame(this);

         if (isFixedInParent())
         {
            twistOfFrame.setBodyFrame(this);
         }
         else
         {
            twistOfFrame.setBodyFrame(getParent()); // This is necessary when the parent frame is a FixedReferenceFrame.
            twistOfFrame.add(twistRelativeToParent);
         }
      }

      isTwistOfFrameUpToDate = true;

      for (int i = 0; i < descendantsMovingFrames.size(); i++)
         descendantsMovingFrames.get(i).isTwistOfFrameUpToDate = false;
   }

   private boolean isTwistOfFrameUpToDateRecursive()
   {
      return isTwistOfFrameUpToDate && (closestAncestorMovingFrame == null || closestAncestorMovingFrame.isTwistOfFrameUpToDateRecursive());
   }

   /**
    * Gets the twist of this frame relative to its direct parent.
    *
    * @return the twist relative to the parent frame.
    */
   public TwistReadOnly getTwistRelativeToParent()
   {
      return twistRelativeToParent;
   }

   /**
    * Gets the twist of this frame with respect the closest stationary frame, i.e. not moving with
    * respect to its root frame.
    * <p>
    * The returned twist the twist of {@code this} with respect to (usually)
    * {@link ReferenceFrame#getWorldFrame()} and expressed in {@code this}.
    * </p>
    *
    * @return the absolute velocity of this frame. The returned object should not be modified.
    */
   public TwistReadOnly getTwistOfFrame()
   {
      updateTwistOfFrame();
      return twistOfFrame;
   }

   /**
    * Packs the twist of this frame with respect the closest stationary frame, i.e. not moving with
    * respect to its root frame.
    * <p>
    * The returned twist the twist of {@code this} with respect to (usually)
    * {@link ReferenceFrame#getWorldFrame()} and expressed in {@code this}.
    * </p>
    *
    * @param twistToPack the twist in which the absolute velocity of this frame is stored. Modified.
    */
   public void getTwistOfFrame(TwistBasics twistToPack)
   {
      twistToPack.setIncludingFrame(getTwistOfFrame());
   }

   /**
    * Computes the twist of this frame relative to the given {@code base}.
    * <p>
    * The reference frames of the resulting {@code Twist} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code this}.
    * <li>{@code baseFrame} is {@code base}.
    * <li>{@code expressedInFrame} is {@code this}.
    * </ul>
    * </p>
    *
    * @param base                the frame with respect to which the twist is to be computed.
    * @param relativeTwistToPack the twist of {@code this} with respect to {@code base}. Modified.
    */
   public void getTwistRelativeToOther(ReferenceFrame base, TwistBasics relativeTwistToPack)
   {
      verifySameRoots(base);

      if (base.isAStationaryFrame())
      {
         getTwistOfFrame(relativeTwistToPack);
         relativeTwistToPack.setBaseFrame(base);
      }
      else if (base instanceof MovingReferenceFrame)
      {
         ((MovingReferenceFrame) base).getTwistOfFrame(relativeTwistToPack);
         relativeTwistToPack.changeFrame(this);
         relativeTwistToPack.sub(getTwistOfFrame());
         relativeTwistToPack.invert();
      }
      else
      {
         throw unhandledReferenceFrameTypeException(base);
      }
   }

   /**
    * Returns the parent moving frame of this moving reference frame.
    * <p>
    * Note that it is {@code null} when this moving frame is the child of a stationary
    * {@code ReferenceFrame}, i.e. not moving with respect to the root frame.
    * </p>
    *
    * @return the parent moving frame of this moving reference frame.
    */
   public MovingReferenceFrame getMovingParent()
   {
      return closestAncestorMovingFrame;
   }

   private static ScrewTheoryException unhandledReferenceFrameTypeException(ReferenceFrame referenceFrame)
   {
      return new ScrewTheoryException("The reference frame type: " + referenceFrame.getClass().getSimpleName()
            + " is currently not handled. Reference frame name: " + referenceFrame.getName());
   }
}
