package us.ihmc.mecano.tools;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FixedJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

/**
 * This class provides tools to integrate the state of a multi-body system.
 * <p>
 * The state can either be integrated from the joint velocities to estimate the new joint
 * configuration, or from the acceleration to estimate the new joint velocity and configuration.
 * When both acceleration and velocity information are available it is preferred to perform a
 * double-integration from the acceleration to obtain a better estimate of the joint configuration.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class MultiBodySystemStateIntegrator
{
   /** Internal reference to the time increment used for each integration. */
   private double dt;
   /**
    * Intermediate variable used for performance, it is equal to: {@code 0.5 * dt * dt}.
    */
   private double half_dt_dt;

   /** Intermediate variable used to perform garbage-free operations. */
   private final Vector3D deltaPosition = new Vector3D();
   /** Intermediate variable used to perform garbage-free operations. */
   private final Vector3D rotationVector = new Vector3D();
   /** Intermediate variable used to perform garbage-free operations. */
   private final Quaternion integrated = new Quaternion();
   /** Intermediate variable used to perform garbage-free operations. */
   private final FrameVector3D linearAcceleration = new FrameVector3D();

   /**
    * Creates a new integrator and initializes the internal time increment to {@link Double#NaN}. It
    * has to be set before this integrator can be used.
    * 
    * @see #setIntegrationDT(double)
    */
   public MultiBodySystemStateIntegrator()
   {
      this(Double.NaN);
   }

   /**
    * Creates a new integrator and initializes the internal time increment to the given {@code dt}.
    * 
    * @param dt the time increment to use for the integration.
    */
   public MultiBodySystemStateIntegrator(double dt)
   {
      setIntegrationDT(dt);
   }

   /**
    * Changes the internal time increment to use for the integration.
    * 
    * @param dt the new time increment.
    */
   public void setIntegrationDT(double dt)
   {
      this.dt = dt;
      half_dt_dt = 0.5 * dt * dt;
   }

   /**
    * Gets the time increment used to perform the integration.
    * 
    * @return the current time increment used internally.
    */
   public double getIntegrationDT()
   {
      return dt;
   }

   /**
    * Recursively navigates the subtree that starts at the given {@code rootBody} and integrates each
    * joint velocity to update their respective configuration.
    * <p>
    * If the acceleration of each joint is available, it is preferable to use
    * {@link #doubleIntegrateFromAccelerationSubtree(RigidBodyBasics)}.
    * </p>
    * 
    * @param rootBody the origin of the subtree to integrate the state of. The configuration of each
    *                 joint in the subtree is modified.
    */
   public void integrateFromVelocitySubtree(RigidBodyBasics rootBody)
   {
      if (rootBody == null)
         return;

      List<? extends JointBasics> childrenJoints = rootBody.getChildrenJoints();

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         JointBasics childJoint = childrenJoints.get(i);
         integrateFromVelocity(childJoint);
         integrateFromVelocitySubtree(childJoint.getSuccessor());
      }
   }

   /**
    * Iterates through the given {@code joints} and integrates each joint velocity to update their
    * respective configuration.
    * <p>
    * If the acceleration of each joint is available, it is preferable to use
    * {@link #doubleIntegrateFromAcceleration(List)}.
    * </p>
    * 
    * @param joints the list of the joints to integrate the state of. The configuration of each joint
    *               is modified.
    */
   public void integrateFromVelocity(List<? extends JointBasics> joints)
   {
      for (int i = 0; i < joints.size(); i++)
      {
         JointBasics joint = joints.get(i);
         integrateFromVelocity(joint);
      }
   }

   /**
    * Integrates the given {@code joint}'s velocity to update its configuration.
    * 
    * @param joint the joint to integrate the state of. The joint configuration is modified.
    */
   public void integrateFromVelocity(JointBasics joint)
   {
      if (joint instanceof OneDoFJointBasics)
         integrateFromVelocity((OneDoFJointBasics) joint);
      else if (joint instanceof FloatingJointBasics)
         integrateFromVelocity((FloatingJointBasics) joint);
      else if (joint instanceof SphericalJointBasics)
         integrateFromVelocity((SphericalJointBasics) joint);
      else if (joint instanceof FixedJointBasics)
         return;
      else
         throw new UnsupportedOperationException("Integrator does not support the joint type: " + joint.getClass().getSimpleName());
   }

   /**
    * Integrates the given {@code joint}'s velocity to update its configuration.
    * 
    * @param joint the joint to integrate the state of. The joint configuration is modified.
    */
   public void integrateFromVelocity(FloatingJointBasics joint)
   {
      integrate(joint.getJointTwist(), joint.getJointPose());
   }

   /**
    * Integrates the given {@code joint}'s velocity to update its configuration.
    * 
    * @param joint the joint to integrate the state of. The joint configuration is modified.
    */
   public void integrateFromVelocity(OneDoFJointBasics joint)
   {
      double initialQ = joint.getQ();
      double qd = joint.getQd();

      joint.setQ(integratePosition(qd, initialQ));
   }

   /**
    * Integrates the given {@code joint}'s velocity to update its configuration.
    * 
    * @param joint the joint to integrate the state of. The joint configuration is modified.
    */
   public void integrateFromVelocity(SphericalJointBasics joint)
   {
      integrate(joint.getJointAngularVelocity(), joint.getJointOrientation());
   }

   /**
    * Integrates the given {@code twist} to update the given {@code poseToIntegrate}.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * poseToIntegrate += dt * twist
    * </pre>
    * </p>
    * 
    * @param twist           the angular and linear velocity to integrate. Not modified.
    * @param poseToIntegrate the pose to update. Modified.
    * @throws ReferenceFrameMismatchException if the given {@code twist} is not expressed in the frame
    *                                         {@code twist.getBodyFrame()}.
    */
   public void integrate(TwistReadOnly twist, Pose3DBasics poseToIntegrate)
   {
      integrate(twist, poseToIntegrate, poseToIntegrate);
   }

   /**
    * Integrates the given {@code twist} to estimate the {@code finalPose}.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * finalPose = initialPose + dt * twist
    * </pre>
    * </p>
    * 
    * @param twist       the angular and linear velocity to integrate. Not modified.
    * @param initialPose the initial pose to append the integrated twist to. Not modified.
    * @param finalPose   the estimated pose after integration. Modified.
    * @throws ReferenceFrameMismatchException if the given {@code twist} is not expressed in the frame
    *                                         {@code twist.getBodyFrame()}.
    */
   public void integrate(TwistReadOnly twist, Pose3DReadOnly initialPose, Pose3DBasics finalPose)
   {
      twist.checkExpressedInFrameMatch(twist.getBodyFrame());

      FrameVector3DReadOnly angularVelocity = twist.getAngularPart();
      FrameVector3DReadOnly linearVelocity = twist.getLinearPart();

      QuaternionReadOnly initialOrientation = initialPose.getOrientation();
      Point3DReadOnly initialPosition = initialPose.getPosition();

      QuaternionBasics finalOrientation = finalPose.getOrientation();
      Point3DBasics finalPosition = finalPose.getPosition();

      integrate(angularVelocity, initialOrientation, finalOrientation);
      integrate(initialOrientation, linearVelocity, initialPosition, finalPosition);
   }

   /**
    * Integrates the given {@code angularVelocity} to update the given {@code orientationToIntegrate}.
    * The given velocity is expected to be expressed in the local frame described by the given
    * orientation.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * orientationToIntegrate += dt * angularVelocity
    * </pre>
    * </p>
    * 
    * @param angularVelocity        the angular velocity to integrate. Not modified.
    * @param orientationToIntegrate the orientation to update. Modified.
    */
   public void integrate(Vector3DReadOnly angularVelocity, Orientation3DBasics orientationToIntegrate)
   {
      integrate(angularVelocity, orientationToIntegrate, orientationToIntegrate);
   }

   /**
    * Integrates the given {@code angularVelocity} to estimate the {@code finalOrientation}. The given
    * velocity is expected to be expressed in the local frame described by the given
    * {@code initialOrientation}.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * finalOrientation = initialOrientation + dt * angularVelocity
    * </pre>
    * </p>
    * 
    * @param angularVelocity    the angular velocity to integrate. Not modified.
    * @param initialOrientation the initial orientation to append the integrated angular velocity to.
    *                           Not modified.
    * @param finalOrientation   the estimated orientation after integration. Modified.
    */
   public void integrate(Vector3DReadOnly angularVelocity, Orientation3DReadOnly initialOrientation, Orientation3DBasics finalOrientation)
   {
      rotationVector.setAndScale(dt, angularVelocity);
      integrated.setRotationVector(rotationVector);
      finalOrientation.set(initialOrientation);
      finalOrientation.append(integrated);
   }

   /**
    * Integrates the given {@code linearVelocity} to estimate the {@code positionToIntegrate}. The
    * given velocity is expected to be expressed in the local frame described by the given orientation.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * positionToIntegrate += orientation * dt * linearVelocity
    * </pre>
    * </p>
    * 
    * @param orientation         the orientation describing the frame in which the velocity is
    *                            expressed. If equal to {@code null}, the orientation is assumed to be
    *                            identity. Not modified.
    * @param linearVelocity      the linear velocity to integrate. Not modified.
    * @param positionToIntegrate the position to update. Modified.
    */
   public void integrate(Orientation3DReadOnly orientation, Vector3DReadOnly linearVelocity, Tuple3DBasics positionToIntegrate)
   {
      integrate(orientation, linearVelocity, positionToIntegrate, positionToIntegrate);
   }

   /**
    * Integrates the given {@code linearVelocity} to estimate the {@code finalPosition}. The given
    * velocity is expected to be expressed in the local frame described by the given orientation.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * finalPosition = initialPosition + orientation * dt * linearVelocity
    * </pre>
    * </p>
    * 
    * @param orientation     the orientation describing the frame in which the velocity is expressed.
    *                        If equal to {@code null}, the orientation is assumed to be identity. Not
    *                        modified.
    * @param linearVelocity  the linear velocity to integrate. Not modified.
    * @param initialPosition the initial position to add the integrated velocity to. Not modified.
    * @param finalPosition   the estimated position after integration. Modified.
    */
   public void integrate(Orientation3DReadOnly orientation, Vector3DReadOnly linearVelocity, Tuple3DReadOnly initialPosition, Tuple3DBasics finalPosition)
   {
      deltaPosition.setAndScale(dt, linearVelocity);

      if (orientation != null)
         orientation.transform(deltaPosition);
      finalPosition.add(initialPosition, deltaPosition);
   }

   /**
    * Integrates the given {@code velocity} to compute the next position.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * finalPosition = initialPosition + dt * velocity
    * </pre>
    * </p>
    * 
    * @param velocity        the velocity to integrate.
    * @param initialPosition the initial position to add the integrated velocity to.
    * @return the final position.
    */
   public double integratePosition(double velocity, double initialPosition)
   {
      return dt * velocity + initialPosition;
   }

   /**
    * Recursively navigates the subtree that starts at the given {@code rootBody} and integrates each
    * joint acceleration and velocity to update their respective velocity and configuration.
    * 
    * @param rootBody the origin of the subtree to integrate the state of. The configuration of each
    *                 joint in the subtree is modified.
    */
   public void doubleIntegrateFromAccelerationSubtree(RigidBodyBasics rootBody)
   {
      if (rootBody == null)
         return;

      List<? extends JointBasics> childrenJoints = rootBody.getChildrenJoints();

      for (int i = 0; i < childrenJoints.size(); i++)
      {
         JointBasics childJoint = childrenJoints.get(i);
         doubleIntegrateFromAcceleration(childJoint);
         doubleIntegrateFromAccelerationSubtree(childJoint.getSuccessor());
      }
   }

   /**
    * Iterates through the given {@code joints} and integrates each joint acceleration and velocity to
    * update their respective velocity and configuration.
    * 
    * @param joints the list of the joints to integrate the state of. The configuration of each joint
    *               is modified.
    */
   public void doubleIntegrateFromAcceleration(List<? extends JointBasics> joints)
   {
      for (int i = 0; i < joints.size(); i++)
      {
         JointBasics joint = joints.get(i);
         doubleIntegrateFromAcceleration(joint);
      }
   }

   /**
    * Integrates the given {@code joint}'s acceleration and velocity to update its velocity and
    * configuration.
    * 
    * @param joint the joint to integrate the state of. The joint configuration is modified.
    */
   public void doubleIntegrateFromAcceleration(JointBasics joint)
   {
      if (joint instanceof OneDoFJointBasics)
         doubleIntegrateFromAcceleration((OneDoFJointBasics) joint);
      else if (joint instanceof FloatingJointBasics)
         doubleIntegrateFromAcceleration((FloatingJointBasics) joint);
      else if (joint instanceof SphericalJointBasics)
         doubleIntegrateFromAcceleration((SphericalJointBasics) joint);
      else if (joint instanceof FixedJointBasics)
         return;
      else
         throw new UnsupportedOperationException("Integrator does not support the joint type: " + joint.getClass().getSimpleName());
   }

   /**
    * Integrates the given {@code joint}'s acceleration and velocity to update its velocity and
    * configuration.
    * 
    * @param joint the joint to integrate the state of. The joint configuration is modified.
    */
   public void doubleIntegrateFromAcceleration(FloatingJointBasics joint)
   {
      doubleIntegrate(joint.getJointAcceleration(), joint.getJointTwist(), joint.getJointPose());
   }

   /**
    * Integrates the given {@code joint}'s acceleration and velocity to update its velocity and
    * configuration.
    * 
    * @param joint the joint to integrate the state of. The joint configuration is modified.
    */
   public void doubleIntegrateFromAcceleration(OneDoFJointBasics joint)
   {
      double initialQ = joint.getQ();
      double initialQd = joint.getQd();
      double qdd = joint.getQdd();

      joint.setQ(doubleIntegratePosition(qdd, initialQd, initialQ));
      joint.setQd(integrateVelocity(qdd, initialQd));
   }

   /**
    * Integrates the given {@code joint}'s acceleration and velocity to update its velocity and
    * configuration.
    * 
    * @param joint the joint to integrate the state of. The joint configuration is modified.
    */
   public void doubleIntegrateFromAcceleration(SphericalJointBasics joint)
   {
      doubleIntegrate(joint.getJointAngularAcceleration(), joint.getJointAngularVelocity(), joint.getJointOrientation());
   }

   /**
    * Integrates the given {@code spatialAcceleration} and {@code twistToIntegrate} to estimate the
    * {@code poseToIntegrate} and {@code twistToIntegrate}.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * poseToIntegrate += 0.5 * dt * dt * spatialAcceleration + dt * twistToIntegrate
    * twistToIntegrate += dt * spatialAcceleration
    * </pre>
    * </p>
    * 
    * @param spatialAcceleration the angular and linear acceleration to integrate. Modified.
    * @param twistToIntegrate    the angular and linear velocity to integrate and update. Modified.
    * @param poseToIntegrate     the pose to update. Modified.
    * @throws ReferenceFrameMismatchException if the given {@code spatialAcceleration} is not expressed
    *                                         in the frame {@code spatialAcceleration.getBodyFrame()}.
    * @throws ReferenceFrameMismatchException if the given {@code spatialAcceleration},
    *                                         {@code initialTwist}, and {@code finalTwist} do not have
    *                                         the same reference frame.
    */
   public void doubleIntegrate(FixedFrameSpatialAccelerationBasics spatialAcceleration, FixedFrameTwistBasics twistToIntegrate, Pose3DBasics poseToIntegrate)
   {
      doubleIntegrate(spatialAcceleration, twistToIntegrate, poseToIntegrate, twistToIntegrate, poseToIntegrate);
   }

   /**
    * Integrates the given {@code spatialAcceleration} and {@code initialTwist} to estimate the
    * {@code finalPose} and {@code finalTwist}.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * poseToIntegrate += 0.5 * dt * dt * spatialAcceleration + dt * twistToIntegrate
    * twistToIntegrate += dt * spatialAcceleration
    * </pre>
    * </p>
    * 
    * @param spatialAcceleration the angular and linear acceleration to integrate. Modified.
    * @param initialTwist        the initial angular and linear velocity to integrate. Not modified.
    * @param initialPose         the initial pose to append the integrated term to. Not modified.
    * @param finalTwist          the estimated twist after integration. Modified.
    * @param finalPose           the estimated pose after integration. Modified.
    * @throws ReferenceFrameMismatchException if the given {@code spatialAcceleration} is not expressed
    *                                         in the frame {@code spatialAcceleration.getBodyFrame()}.
    * @throws ReferenceFrameMismatchException if the given {@code spatialAcceleration},
    *                                         {@code initialTwist}, and {@code finalTwist} do not have
    *                                         the same reference frame.
    */
   public void doubleIntegrate(FixedFrameSpatialAccelerationBasics spatialAcceleration, TwistReadOnly initialTwist, Pose3DReadOnly initialPose,
                               FixedFrameTwistBasics finalTwist, Pose3DBasics finalPose)
   {
      spatialAcceleration.checkReferenceFrameMatch(initialTwist);
      spatialAcceleration.checkExpressedInFrameMatch(spatialAcceleration.getBodyFrame());
      if (finalTwist != initialTwist)
         finalTwist.checkReferenceFrameMatch(initialTwist);

      QuaternionReadOnly initialOrientation = initialPose.getOrientation();
      QuaternionBasics finalOrientation = finalPose.getOrientation();
      Point3DReadOnly initialPosition = initialPose.getPosition();
      Point3DBasics finalPosition = finalPose.getPosition();

      FrameVector3DReadOnly initialAngularVelocity = initialTwist.getAngularPart();
      FixedFrameVector3DBasics finalAngularVelocity = finalTwist.getAngularPart();
      FrameVector3DReadOnly initialLinearVelocity = initialTwist.getLinearPart();
      FixedFrameVector3DBasics finalLinearVelocity = finalTwist.getLinearPart();

      FixedFrameVector3DBasics angularAcceleration = spatialAcceleration.getAngularPart();
      spatialAcceleration.getLinearAccelerationAtBodyOrigin(initialTwist, linearAcceleration);

      rotationVector.setAndScale(dt, initialAngularVelocity);
      rotationVector.scaleAdd(half_dt_dt, angularAcceleration, rotationVector);
      integrated.setRotationVector(rotationVector);

      if (finalAngularVelocity != null)
      {
         finalAngularVelocity.scaleAdd(dt, angularAcceleration, initialAngularVelocity);
      }

      if (finalPosition != null)
      {
         deltaPosition.setAndScale(dt, initialLinearVelocity);
         deltaPosition.scaleAdd(half_dt_dt, linearAcceleration, deltaPosition);
         if (initialOrientation != null)
            initialOrientation.transform(deltaPosition);

         finalPosition.add(initialPosition, deltaPosition);
      }

      if (finalLinearVelocity != null)
      {
         finalLinearVelocity.scaleAdd(dt, linearAcceleration, initialLinearVelocity);
         integrated.inverseTransform(finalLinearVelocity);
      }

      if (finalOrientation != null)
      {
         if (finalOrientation != initialOrientation)
            finalOrientation.set(initialOrientation);
         finalOrientation.append(integrated);
      }

      integrated.inverseTransform(linearAcceleration);
      spatialAcceleration.setBasedOnOriginAcceleration(angularAcceleration, linearAcceleration, finalTwist);
   }

   /**
    * Integrates the given {@code angularAcceleration} and {@code angularVelocityToIntegrate} to update
    * the given {@code orientationToIntegrate} and {@code angularVelocityToIntegrate}. The given
    * acceleration and velocity are expected to be expressed in the local frame described by the given
    * orientation.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * orientationToIntegrate += 0.5 * dt * dt * angularAcceleration + dt * angularVelocityToIntegrate
    * angularVelocityToIntegrate += dt * spatialAcceleration
    * </pre>
    * </p>
    * 
    * @param angularAcceleration        the angular acceleration to integrate. Not modified.
    * @param angularVelocityToIntegrate the angular velocity to integrate and update. Modified.
    * @param orientationToIntegrate     the orientation to update. Modified.
    */
   public void doubleIntegrate(Vector3DReadOnly angularAcceleration, Vector3DBasics angularVelocityToIntegrate, Orientation3DBasics orientationToIntegrate)
   {
      doubleIntegrate(angularAcceleration, angularVelocityToIntegrate, orientationToIntegrate, angularVelocityToIntegrate, orientationToIntegrate);
   }

   /**
    * Integrates the given {@code angularAcceleration} and {@code initialAngularVelocity} to update the
    * given {@code finalOrientation} and {@code finalAngularVelocity}. The given acceleration and
    * velocity are expected to be expressed in the local frame described by the given
    * {@code initialOrientation}.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * orientationToIntegrate += 0.5 * dt * dt * angularAcceleration + dt * angularVelocityToIntegrate
    * angularVelocityToIntegrate += dt * spatialAcceleration
    * </pre>
    * </p>
    * 
    * @param angularAcceleration    the angular acceleration to integrate. Not modified.
    * @param initialAngularVelocity the angular velocity to integrate. Not modified.
    * @param initialOrientation     the initial orientation to append the integrated term to. Not
    *                               modified.
    * @param finalAngularVelocity   the estimated angular velocity after integration. Modified.
    * @param finalOrientation       the estimated orientation after integration. Modified.
    */
   public void doubleIntegrate(Vector3DReadOnly angularAcceleration, Vector3DReadOnly initialAngularVelocity, Orientation3DReadOnly initialOrientation,
                               Vector3DBasics finalAngularVelocity, Orientation3DBasics finalOrientation)
   {
      if (finalOrientation != null)
      {
         rotationVector.setAndScale(dt, initialAngularVelocity);
         rotationVector.scaleAdd(half_dt_dt, angularAcceleration, rotationVector);
         integrated.setRotationVector(rotationVector);
         finalOrientation.set(initialOrientation);
         finalOrientation.append(integrated);
      }

      if (finalAngularVelocity != null)
      {
         finalAngularVelocity.scaleAdd(dt, angularAcceleration, initialAngularVelocity);
      }
   }

   /**
    * Integrates the given {@code linearAcceleration} and {@code linearVelocityToIntegrate} to estimate
    * the {@code positionToIntegrate} and {@code linearVelocityToIntegrate}. The given acceleration and
    * velocity is expected to be expressed in the local frame described by the given orientation.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * positionToIntegrate += orientation * (0.5 * dt * dt * linearAcceleration + dt * linearVelocityToIntegrate)
    * linearVelocityToIntegrate += dt * linearAcceleration
    * </pre>
    * </p>
    * 
    * @param orientation               the orientation describing the frame in which the acceleration
    *                                  and velocity are expressed. If equal to {@code null}, the
    *                                  orientation is assumed to be identity. Not modified.
    * @param linearAcceleration        the linear acceleration to integrate. Not modified.
    * @param linearVelocityToIntegrate the linear velocity to integrate and update. Modified.
    * @param positionToIntegrate       the position to update. Modified.
    * @deprecated This integration method is inaccurate and does not account for the interdependency
    *             between angular and linear states properly.
    */
   public void doubleIntegrate(Orientation3DReadOnly orientation, Vector3DReadOnly linearAcceleration, Vector3DBasics linearVelocityToIntegrate,
                               Tuple3DBasics positionToIntegrate)
   {
      doubleIntegrate(orientation, linearAcceleration, linearVelocityToIntegrate, positionToIntegrate, linearVelocityToIntegrate, positionToIntegrate);
   }

   /**
    * Integrates the given {@code linearAcceleration} and {@code initialLinearVelocity} to estimate the
    * {@code finalPosition} and {@code finalLinearVelocity}. The given acceleration and velocity is
    * expected to be expressed in the local frame described by the given orientation.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * finalPosition = initialPosition + orientation * (0.5 * dt * dt * linearAcceleration + dt * initialLinearVelocity)
    * finalLinearVelocity = initialLinearVelocity + dt * linearAcceleration
    * </pre>
    * </p>
    * 
    * @param orientation           the orientation describing the frame in which the velocity is
    *                              expressed. If equal to {@code null}, the orientation is assumed to
    *                              be identity. Not modified.
    * @param linearAcceleration    the linear acceleration to integrate. Not modified.
    * @param initialLinearVelocity the linear velocity to integrate. Not modified.
    * @param initialPosition       the initial position to add the integrated term to. Not modified.
    * @param finalLinearVelocity   the estimated linear velocity after integration. Modified.
    * @param finalPosition         the estimated position after integration. Modified.
    * @deprecated This integration method is inaccurate and does not account for the interdependency
    *             between angular and linear states properly.
    */
   public void doubleIntegrate(Orientation3DReadOnly orientation, Vector3DReadOnly linearAcceleration, Vector3DReadOnly initialLinearVelocity,
                               Tuple3DReadOnly initialPosition, Vector3DBasics finalLinearVelocity, Tuple3DBasics finalPosition)
   {
      if (finalPosition != null)
      {
         deltaPosition.setAndScale(dt, initialLinearVelocity);
         deltaPosition.scaleAdd(half_dt_dt, linearAcceleration, deltaPosition);

         if (orientation != null)
            orientation.transform(deltaPosition);
         finalPosition.add(initialPosition, deltaPosition);
      }

      if (finalLinearVelocity != null)
      {
         finalLinearVelocity.scaleAdd(dt, linearAcceleration, initialLinearVelocity);
      }
   }

   /**
    * Integrates the given {@code acceleration} and {@code velocity} to compute the next position.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * finalPosition = initialPosition + 0.5 * dt * dt + acceleration + dt * initialVelocity
    * </pre>
    * </p>
    * 
    * @param acceleration    the acceleration to integrate.
    * @param initialVelocity the velocity to integrate.
    * @param initialPosition the initial position to add the integrated term to.
    * @return the final position.
    */
   public double doubleIntegratePosition(double acceleration, double initialVelocity, double initialPosition)
   {
      return half_dt_dt * acceleration + dt * initialVelocity + initialPosition;
   }

   /**
    * Integrates the given {@code acceleration} to compute the next velocity.
    * <p>
    * Pseudo-code:
    * 
    * <pre>
    * finalVelocity = initialVelocity + dt * acceleration
    * </pre>
    * </p>
    * 
    * @param acceleration    the acceleration to integrate.
    * @param initialVelocity the initial velocity to add the integrated acceleration to.
    * @return the final velocity.
    */
   public double integrateVelocity(double acceleration, double initialVelocity)
   {
      return dt * acceleration + initialVelocity;
   }
}
