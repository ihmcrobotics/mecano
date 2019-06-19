package us.ihmc.mecano.spatial.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a spatial acceleration which reference frames can be changed.
 * <p>
 * A spatial acceleration is a vector composed of 6 components with an angular part and a linear
 * part. It represents the time derivative of a twist. The angular part represents a 3D angular
 * acceleration and the linear part a 3D linear acceleration.
 * </p>
 * <p>
 * A spatial acceleration always describes the relative velocity of a body with respect to a base.
 * These two entities are referred to here by using two reference frames: a {@code bodyFrame} that
 * is considered to be rigidly attached to the body, and a {@code baseFrame} that is considered to
 * be rigidly attached to the base.
 * </p>
 * <p>
 * When using a {@code SpatialAccelerationBasics}, it is important to note that the reference frame
 * in which it is expressed does not only refer to the coordinate system in which the angular and
 * linear 3D vectors are expressed. The origin of the reference frame is also used as the point
 * where the acceleration is measured. While the angular part remains the same as the point of
 * measurement changes, the linear part does depend on its location.
 * </p>
 * <p>
 * This framework for representing in an accurate and safe manner spatial accelerations is based on
 * the Ph.D. thesis of Vincent Duindam entitled <i>"Port-Based Modeling and Control for Efficient
 * Bipedal Walking Robots"</i>. Duindam's publications can be found
 * <a href="http://sites.google.com/site/vincentduindam/publications">here</a>. Several references
 * to this work are spread throughout the code.
 * </p>
 * <p>
 * The convention when using a spatial vector in matrix operations is that the angular part occupies
 * the 3 first rows and the linear part the 3 last as follows:<br>
 *
 * <pre>
 *     / angularX \
 *     | angularY |
 *     | angularZ |
 * V = | linearX  |
 *     | linearY  |
 *     \ linearX  /
 * </pre>
 * </p>
 *
 * @author Twan Koolen
 * @author Sylvain Bertrand
 */
public interface SpatialAccelerationBasics extends FixedFrameSpatialAccelerationBasics, SpatialMotionBasics
{
   /**
    * Transforms this vector such that the result is the spatial acceleration of the same body with
    * respect to the same base, but expressed in a different frame.
    * <p>
    * Once this spatial acceleration is transformed, the reference frame "expressed-in-frame" is
    * updated to {@code desiredFrame}. In the case, {@code this.expressedInFrame == desiredFrame}, this
    * method does nothing.
    * </p>
    * <p>
    * Note that from a physical perspective, changing the frame of a spatial acceleration essentially
    * means 3 things:
    * <ul>
    * <li>Changing the frame of the two internal 3D vectors such that their components ends up being
    * expressed in {@code desiredFrame}.
    * <li>Changing the location where the body acceleration is measured.
    * <li>Changing the reference from which the acceleration is perceived, this is essentially modifies
    * the measured acceleration when the velocity between the current and the new "expressed-in-frame"s
    * is non-zero.
    * </ul>
    * </p>
    * <p>
    * Effectively, the new spatial acceleration (A<sub>body</sub><sup>des, base</sup> =
    * [&omega;'<sub>new</sub>; &alpha;<sub>new</sub>]) is calculated as follows:
    *
    * <pre>
    * &omega;'<sub>new</sub> = R ( &omega;' + &omega;<sub>old</sub> &times; &omega;<sub>body</sub>)
    * &alpha;<sub>new</sub> = R &alpha; + P &times; &omega;'<sub>new</sub> + R (&nu;<sub>old</sub> &times; &omega;<sub>body</sub> + &omega;<sub>old</sub> &times; &nu;<sub>body</sub>)
    * </pre>
    *
    * where:
    * <ul>
    * <li>&omega;' and &alpha; are the current angular and linear parts of this spatial vector,
    * <li>R and P are the rotation and translation of the current "expressed-in-frame" with respect to
    * the desired frame,
    * <li>T<sub>body</sub><sup>old, base</sup> = [&omega;<sub>body</sub>; &nu;<sub>body</sub>] is the
    * twist of the body with respect to base,
    * <li>and finally T<sub>old</sub><sup>old, des</sup></sub> = [&omega;<sub>old</sub>;
    * &nu;<sub>old</sub>] is the twist of the current "expressed-in-frame" with respect to the desired
    * frame.
    * </ul>
    * </p>
    * <p>
    * These equations can be derived by calculating the time derivative of the identity presented in
    * Duindam, <i>Port-Based Modeling and Control for Efficient Bipedal Walking Robots</i>, page 25,
    * lemma 2.8 (c) as follows:
    *
    * <pre>
    *  <sub>    </sub><sup>         </sup>   d
    * A<sub>body</sub><sup>des, base</sup> = -- ( Ad<sub>H<sup>des</sup><sub>old</sub></sub> ) T<sub>body</sub><sup>old, base</sup> + Ad<sub>H<sup>des</sup><sub>old</sub></sub> A<sub>body</sub><sup>old, base</sup>
    *  <sub>    </sub><sup>         </sup>   dt
    * A<sub>body</sub><sup>des, base</sup> = Ad<sub>H<sup>des</sup><sub>old</sub></sub> Ad<sub>T<sub>old</sub><sup>old, des</sup></sub> T<sub>body</sub><sup>old, base</sup> + Ad<sub>H<sup>des</sup><sub>old</sub></sub> A<sub>body</sub><sup>old, base</sup>
    * </pre>
    *
    * Then introducing the identities introduced in the lemma 2.8 (f).
    * </p>
    * <p>
    * Although the equations above specifically dictate the frames of the twists to be provided, this
    * method attempts to provide as much flexibility as possible notably regarding the frame in which
    * they are expressed.
    * </p>
    *
    * @param desiredFrame the frame in which this spatial should be expressed.
    * @param deltaTwist   twists referring the relative velocity between the {@code desiredFrame} and
    *                     the current {@code expressedInFrame}. It can either be expressed in
    *                     {@code desiredFrame} or in {@code this.expressedInFrame}. Not modified.
    * @param bodyTwist    twist of {@code this.bodyFrame} with respect to {@code this.baseFrame}. It
    *                     can either be expressed in {@code desiredFrame} or in
    *                     {@code this.expressedInFrame}. Not modified.
    * @throws ReferenceFrameMismatchException if
    *                                         {@code deltaTwist.getReferenceFrame() != bodyTwist.getReferenceFrame()}
    *                                         and if the frame in which they are expressed is neither
    *                                         {@code this.expressedInFrame} or {@code desiredFrame}.
    * @throws ReferenceFrameMismatchException if {@code deltaTwist} does not refer to the relative
    *                                         velocity between {@code this.expressedInFrame} and
    *                                         {@code desiredFrame}.
    * @throws ReferenceFrameMismatchException if {@code bodyTwist}does not refer to the velocity of
    *                                         {@code this.bodyFrame} with respect to
    *                                         {@code desiredFrame}.
    */
   default void changeFrame(ReferenceFrame desiredFrame, TwistReadOnly deltaTwist, TwistReadOnly bodyTwist)
   {
      // trivial case:
      if (getReferenceFrame() == desiredFrame)
      {
         return;
      }
      else if (getReferenceFrame() == getBaseFrame() && desiredFrame == getBodyFrame())
      {
         /*
          * Trying benefit from the property of switching between base and body frames which does not require
          * the use of the twists as they cancel each other.
          */
         changeFrame(desiredFrame);
      }
      else if (getReferenceFrame() == getBodyFrame() && desiredFrame == getBaseFrame())
      {
         /*
          * Trying benefit from the property of switching between base and body frames which does not require
          * the use of the twists as they cancel each other.
          */
         changeFrame(desiredFrame);
      }
      else
      {
         // Ensure that the bodyTwist is the twist of the body with respect to the base.
         bodyTwist.checkBodyFrameMatch(getBodyFrame());
         bodyTwist.checkBaseFrameMatch(getBaseFrame());

         boolean flipCrossProducts;

         if (deltaTwist.getBodyFrame() == getReferenceFrame())
         {
            deltaTwist.checkBaseFrameMatch(desiredFrame);
            flipCrossProducts = false;
         }
         else
         {
            deltaTwist.checkBodyFrameMatch(desiredFrame);
            deltaTwist.checkBaseFrameMatch(getReferenceFrame());
            flipCrossProducts = true;
         }

         // Check that the two twists are expressed in the same frame.
         deltaTwist.checkExpressedInFrameMatch(bodyTwist.getReferenceFrame());

         if (deltaTwist.getReferenceFrame() == getReferenceFrame())
         { // Both twists are expressed in the current frame. By doing the cross-products first they will be transformed along with this acceleration.

            // Handling Coriolis acceleration or bias acceleration as referred to in Featherstone's book.
            if (flipCrossProducts)
            {
               // omega_body x v_new
               addCrossToLinearPart(bodyTwist.getAngularPart(), deltaTwist.getLinearPart());
               // v_body x omega_new
               addCrossToLinearPart(bodyTwist.getLinearPart(), deltaTwist.getAngularPart());
               // omega_body x omega_new
               addCrossToAngularPart(bodyTwist.getAngularPart(), deltaTwist.getAngularPart());
            }
            else
            {
               // v_old x omega_body
               addCrossToLinearPart(deltaTwist.getLinearPart(), bodyTwist.getAngularPart());
               // omega_old x v_body
               addCrossToLinearPart(deltaTwist.getAngularPart(), bodyTwist.getLinearPart());
               // omega_old x omega_body
               addCrossToAngularPart(deltaTwist.getAngularPart(), bodyTwist.getAngularPart());
            }

            changeFrame(desiredFrame);
         }
         else if (deltaTwist.getReferenceFrame() == desiredFrame)
         { // Both twists are already expressed in the desired frame, let's first change the frame of this acceleration and add the cross products after.

            changeFrame(desiredFrame);

            // Handling Coriolis acceleration or bias acceleration as referred to in Featherstone's book.
            if (flipCrossProducts)
            {
               // omega_body x v_new
               addCrossToLinearPart(bodyTwist.getAngularPart(), deltaTwist.getLinearPart());
               // v_body x omega_new
               addCrossToLinearPart(bodyTwist.getLinearPart(), deltaTwist.getAngularPart());
               // omega_body x omega_new
               addCrossToAngularPart(bodyTwist.getAngularPart(), deltaTwist.getAngularPart());
            }
            else
            {
               // v_old x omega_body
               addCrossToLinearPart(deltaTwist.getLinearPart(), bodyTwist.getAngularPart());
               // omega_old x v_body
               addCrossToLinearPart(deltaTwist.getAngularPart(), bodyTwist.getLinearPart());
               // omega_old x omega_body
               addCrossToAngularPart(deltaTwist.getAngularPart(), bodyTwist.getAngularPart());
            }
         }
         else
         {
            throw new ReferenceFrameMismatchException("The given twists should be expressed in either: " + getReferenceFrame() + " or " + desiredFrame
                  + " but were expressed in: " + deltaTwist.getReferenceFrame());
         }
      }
   }

   /**
    * Sets this spatial acceleration given the angular and linear acceleration of the body frame
    * perceived from the base frame.
    * <p>
    * It is often more intuitive to quantify a body acceleration from the perspective of the base
    * frame. However, when creating a spatial acceleration, the frame from which the acceleration is
    * perceived and the point of which the acceleration is measured are usually part of the same frame.
    * This implies an actual difference between the two representations due to Coriolis accelerations
    * solely introduced because the two reference frames are moving with respect to each other.
    * </p>
    * <p>
    * This method allows to change from the intuitive representation, to the one used in this
    * framework. This spatial acceleration is updated as follows:
    *
    * <pre>
    * &omega;'<sub>new</sub> = &omega;'<sub>arg</sub>
    * &alpha;<sub>new</sub> = &alpha;<sub>arg</sub> + &nu;<sub>twist</sub> &times; &omega;<sub>twist</sub>
    * </pre>
    * </p>
    *
    * @param bodyFrame           what we are specifying the acceleration of. It is also used to define
    *                            in what frame the acceleration is expressed.
    * @param baseFrame           with respect to what we are specifying the acceleration.
    * @param angularAcceleration the angular acceleration of the body. Not modified.
    * @param originAcceleration  the linear acceleration of the body frame's origin perceived from the
    *                            base frame. Not modified.
    * @param bodyTwist           this is the twist of {@code bodyFrame} with respect to
    *                            {@code baseFrame} and expressed in {@code expressedInFrame}.
    * @throws ReferenceFrameMismatchException if {@code bodyTwist} does not have the same frames as
    *                                         {@code this}.
    */
   default void setBasedOnOriginAccelerationIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector3DReadOnly angularAcceleration,
                                                           FrameVector3DReadOnly originAcceleration, TwistReadOnly bodyTwist)
   {
      setBodyFrame(bodyFrame);
      setBaseFrame(baseFrame);
      setReferenceFrame(bodyFrame);

      setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);
   }

   /**
    * Adds another spatial acceleration to {@code code} after performing the usual reference frame
    * checks.
    * <p>
    * Note that either the base frame or the body frame of this spatial acceleration after this
    * operation is updated. Let's assume the spatial acceleration A<sub>B</sub> of a banana with
    * respect to a table and the spatial acceleration A<sub>O</sub> of an orange with respect to the
    * banana, then the addition A<sub>B</sub> + A<sub>O</sub> is the spatial acceleration of the orange
    * with respect to the table.
    * </p>
    * <p>
    * See Duindam, <i>Port-Based Modeling and Control for Efficient Bipedal Walking Robots</i>, page
    * 25, lemma 2.8 (e) for twists. By differentiating, the statement results in the same thing for
    * spatial accelerations.
    * </p>
    *
    * @param other the other spatial acceleration to add to {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}. Or if the two spatial
    *                                         accelerations cannot be "concatenated":
    *                                         {@code this.bodyFrame != other.baseFrame} and
    *                                         {@code other.bodyFrame != this.baseFrame}.
    */
   default void add(SpatialAccelerationReadOnly other)
   {
      checkExpressedInFrameMatch(other.getReferenceFrame());
      if (getBodyFrame() == other.getBaseFrame())
         setBodyFrame(other.getBodyFrame());
      else if (getBaseFrame() == other.getBodyFrame())
         setBaseFrame(other.getBaseFrame());
      else
         throw new ReferenceFrameMismatchException("This acceleration and other are incompatible for addition.");

      // Casting to a frameless type to prevent unnecessary frame check.
      getAngularPart().add((Vector3DReadOnly) other.getAngularPart());
      getLinearPart().add((Vector3DReadOnly) other.getLinearPart());
   }

   /**
    * Subtracts another spatial acceleration to {@code code} after performing the usual reference frame
    * checks.
    * <p>
    * Note that either the base frame or the body frame of this spatial acceleration after this
    * operation is updated. Let's assume the spatial acceleration A<sub>B</sub> of a banana with
    * respect to a table and the spatial acceleration A<sub>O</sub> of an orange with respect to the
    * table, then the difference A<sub>B</sub> - A<sub>O</sub> is the spatial acceleration of the
    * banana with respect to the orange.
    * </p>
    * <p>
    * See Duindam, <i>Port-Based Modeling and Control for Efficient Bipedal Walking Robots</i>, page
    * 25, lemma 2.8 (e) for twists. By differentiating, the statement results in the same thing for
    * spatial accelerations.
    * </p>
    *
    * @param other the other spatial acceleration to subtract to {@code this}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}. Or if the two spatial
    *                                         accelerations cannot be "concatenated":
    *                                         {@code this.bodyFrame != other.baseFrame} and
    *                                         {@code other.bodyFrame != this.baseFrame}.
    */
   default void sub(SpatialAccelerationReadOnly other)
   {
      checkExpressedInFrameMatch(other.getReferenceFrame());
      // make sure that either the bodyFrames or baseFrames are the same
      if (getBaseFrame() == other.getBaseFrame())
         setBaseFrame(other.getBodyFrame());
      else if (getBodyFrame() == other.getBodyFrame())
         setBodyFrame(other.getBaseFrame());
      else
         throw new ReferenceFrameMismatchException("This acceleration and other are incompatible for difference.");

      // Casting to a frameless type to prevent unnecessary frame check.
      getAngularPart().sub((Vector3DReadOnly) other.getAngularPart());
      getLinearPart().sub((Vector3DReadOnly) other.getLinearPart());
   }
}
