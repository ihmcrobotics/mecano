package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * This class is a tool that can be used to compute the spatial acceleration of each
 * {@code RigidBody} composing a rigid-body system.
 * <p>
 * Every time the system's state is changing, the spatial acceleration calculator can be notified
 * via {@link #reset()}, the spatial acceleration of any rigid-body can then be obtained as follows:
 * <ul>
 * <li>{@link #getAccelerationOfBody(RigidBodyReadOnly)} provides the spatial acceleration of any
 * rigid-body with respect to the {@code inertialFrame}.
 * <li>{@link #getRelativeAcceleration(RigidBodyReadOnly, RigidBodyReadOnly)} provides the spatial
 * acceleration of any rigid-body with respect to another rigid-body of the same system.
 * <li>{@link #getLinearAccelerationOfBodyFixedPoint(RigidBodyReadOnly, FramePoint3DReadOnly)}
 * provides the linear acceleration of a point of a rigid-body with respect to the
 * {@code inertialFrame}.
 * <li>{@link #getLinearAccelerationOfBodyFixedPoint(RigidBodyReadOnly, RigidBodyReadOnly, FramePoint3DReadOnly)}
 * provides the linear acceleration of a point of a rigid-body with respect to another rigid-body of
 * the same system.
 * </ul>
 * </p>
 */
public class SpatialAccelerationCalculator implements RigidBodyAccelerationProvider
{
   /**
    * The root body of the system for which this {@code SpatialAccelerationCalculator} is available.
    */
   private final RigidBodyReadOnly rootBody;
   /**
    * Whether rigid-body accelerations resulting from centrifugal and Coriolis effects are considered
    * or ignored.
    */
   private final boolean doVelocityTerms;
   /**
    * Whether rigid-body accelerations resulting from joint accelerations are considered or ignored.
    */
   private final boolean doAccelerationTerms;

   /**
    * Internal storage of the acceleration of each body the system. This is the map from rigid-bodies
    * to indices to use with {@code rigidBodiesWithAssignedAcceleration} and
    * {@code assignedAccelerations} for retrieving their acceleration if already computed. If no
    * acceleration has been computed yet, the index is equal to {@code -1}.
    */
   private final Map<RigidBodyReadOnly, AccelerationIndex> rigidBodyToAssignedAccelerationIndex = new HashMap<>();
   /**
    * List of the rigid-bodies with an up-to-date acceleration. This list allows a garbage free
    * clearance of the {@code rigidBodyToAssignedAccelerationIndex} map.
    */
   private final List<RigidBodyReadOnly> rigidBodiesWithAssignedAcceleration;
   /**
    * The list of up-to-date accelerations assigned to rigid-bodies. The association acceleration <->
    * rigid-body can be retrieved using the map {@code rigidBodyToAssignedAccelerationIndex}.
    * <p>
    * The first element of this list is always for the spatial acceleration of the root body. Even
    * assuming that the root is fixed in world, root's acceleration is usually set to be the opposite
    * of the gravitational acceleration, such that the effect of the gravity is naturally propagated to
    * the entire system.
    * </p>
    */
   private final List<SpatialAcceleration> assignedAccelerations;
   /**
    * List of out-of-date accelerations used to recycle memory.
    */
   private final List<SpatialAcceleration> unnassignedAccelerations = new ArrayList<>();

   private final ReferenceFrame inertialFrame;
   private final RigidBodyAccelerationProvider accelerationProvider;

   /**
    * Creates a new {@code SpatialAccelerationCalculator} that will compute all the spatial
    * accelerations of all the rigid-bodies of the system to which {@code body} belongs.
    *
    * @param body          a body that belongs to the system this spatial acceleration calculator will
    *                      be available for.
    * @param inertialFrame each rigid-body acceleration is calculated with respect to the inertial
    *                      frame.
    */
   public SpatialAccelerationCalculator(RigidBodyReadOnly body, ReferenceFrame inertialFrame)
   {
      this(body, inertialFrame, true);
   }

   /**
    * Creates a new {@code SpatialAccelerationCalculator} that will compute all the spatial
    * accelerations of all the rigid-bodies of the system to which {@code body} belongs.
    *
    * @param body            a body that belongs to the system this spatial acceleration calculator
    *                        will be available for.
    * @param inertialFrame   each rigid-body acceleration is calculated with respect to the inertial
    *                        frame.
    * @param doVelocityTerms whether the centrifugal and Coriolis are considered or ignored in the
    *                        computation of the rigid-body accelerations.
    */
   public SpatialAccelerationCalculator(RigidBodyReadOnly body, ReferenceFrame inertialFrame, boolean doVelocityTerms)
   {
      this(body, inertialFrame, doVelocityTerms, true);
   }

   /**
    * Creates a new {@code SpatialAccelerationCalculator} that will compute all the spatial
    * accelerations of all the rigid-bodies of the system to which {@code body} belongs.
    *
    * @param body                a body that belongs to the system this spatial acceleration calculator
    *                            will be available for.
    * @param inertialFrame       each rigid-body acceleration is calculated with respect to the
    *                            inertial frame.
    * @param doVelocityTerms     whether the centrifugal and Coriolis are considered or ignored in the
    *                            computation of the rigid-body accelerations.
    * @param doAccelerationTerms whether the joint accelerations are considered or ignored in the
    *                            computation of the rigid-body accelerations.
    */
   public SpatialAccelerationCalculator(RigidBodyReadOnly body, ReferenceFrame inertialFrame, boolean doVelocityTerms, boolean doAccelerationTerms)
   {
      this.inertialFrame = inertialFrame;
      rootBody = MultiBodySystemTools.getRootBody(body);
      this.doVelocityTerms = doVelocityTerms;
      this.doAccelerationTerms = doAccelerationTerms;

      int numberOfRigidBodies = (int) rootBody.subtreeStream().count();
      while (unnassignedAccelerations.size() < numberOfRigidBodies)
         unnassignedAccelerations.add(new SpatialAcceleration());
      rigidBodiesWithAssignedAcceleration = new ArrayList<>(numberOfRigidBodies);

      assignedAccelerations = new ArrayList<>(numberOfRigidBodies);
      assignedAccelerations.add(new SpatialAcceleration(rootBody.getBodyFixedFrame(), inertialFrame, rootBody.getBodyFixedFrame()));
      rigidBodiesWithAssignedAcceleration.add(rootBody);
      rigidBodyToAssignedAccelerationIndex.put(rootBody, new AccelerationIndex(0));

      accelerationProvider = RigidBodyAccelerationProvider.toRigidBodyAccelerationProvider(this::getAccelerationOfBody,
                                                                                           inertialFrame,
                                                                                           doVelocityTerms,
                                                                                           doAccelerationTerms);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration, it is usually equal to
    *                {@code (0, 0, -9.81)}.
    */
   public void setGravitionalAcceleration(FrameTuple3DReadOnly gravity)
   {
      gravity.checkReferenceFrameMatch(inertialFrame);
      setGravitionalAcceleration((Tuple3DReadOnly) gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration, it is usually equal to
    *                {@code (0, 0, -9.81)}.
    */
   public void setGravitionalAcceleration(Tuple3DReadOnly gravity)
   {
      SpatialAcceleration rootAcceleration = assignedAccelerations.get(0);
      rootAcceleration.setToZero();
      rootAcceleration.getLinearPart().setAndNegate(gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravity the gravitational linear acceleration along the z-axis, it is usually equal to
    *                {@code -9.81}.
    */
   public void setGravitionalAcceleration(double gravity)
   {
      setGravitionalAcceleration(0.0, 0.0, gravity);
   }

   /**
    * Set the gravitational acceleration to account for in this multi-body system.
    * <p>
    * The acceleration of the root body is set to the opposite of the gravitational acceleration such
    * that it gets naturally propagated to the whole system.
    * </p>
    *
    * @param gravityX the gravitational linear acceleration along the x-axis, it is usually equal to
    *                 {@code 0}.
    * @param gravityY the gravitational linear acceleration along the y-axis, it is usually equal to
    *                 {@code 0}.
    * @param gravityZ the gravitational linear acceleration along the z-axis, it is usually equal to
    *                 {@code -9.81}.
    */
   public void setGravitionalAcceleration(double gravityX, double gravityY, double gravityZ)
   {
      SpatialAcceleration rootAcceleration = assignedAccelerations.get(0);
      rootAcceleration.setToZero();
      rootAcceleration.getLinearPart().set(gravityX, gravityY, gravityZ);
      rootAcceleration.negate();
   }

   /**
    * Changes the spatial acceleration of the root. Even though the root is assumed to be non-moving,
    * the {@code rootAcceleration} is usually set to the opposite of the gravitational acceleration,
    * such that the effect of the gravity is naturally propagated to the entire system.
    *
    * @param newRootAcceleration the new spatial acceleration of the root.
    * @throws ReferenceFrameMismatchException if any of the reference frames of
    *                                         {@code newRootAcceleration} does not match this
    *                                         calculator's root spatial acceleration's frames.
    */
   public void setRootAcceleration(SpatialAccelerationReadOnly newRootAcceleration)
   {
      assignedAccelerations.get(0).set(newRootAcceleration);
   }

   /**
    * Notifies the system has changed state (configuration, velocity, and acceleration).
    * <p>
    * This method has to be called once every time the system state has changed, and before calling the
    * methods {@link #getAccelerationOfBody(RigidBodyReadOnly)} and
    * {@link #getRelativeAcceleration(RigidBodyReadOnly, RigidBodyReadOnly)}.
    * </p>
    */
   public void reset()
   {
      while (rigidBodiesWithAssignedAcceleration.size() > 1)
         rigidBodyToAssignedAccelerationIndex.get(rigidBodiesWithAssignedAcceleration.remove(rigidBodiesWithAssignedAcceleration.size() - 1)).setIndex(-1);

      while (assignedAccelerations.size() > 1)
         unnassignedAccelerations.add(assignedAccelerations.remove(assignedAccelerations.size() - 1));

      rigidBodyToAssignedAccelerationIndex.get(rootBody).setIndex(0);
   }

   @Override
   public SpatialAccelerationReadOnly getAccelerationOfBody(RigidBodyReadOnly body)
   {
      return computeOrGetAccelerationOfBody(body);
   }

   @Override
   public SpatialAccelerationReadOnly getRelativeAcceleration(RigidBodyReadOnly base, RigidBodyReadOnly body)
   {
      return accelerationProvider.getRelativeAcceleration(base, body);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationOfBodyFixedPoint(RigidBodyReadOnly base, RigidBodyReadOnly body, FramePoint3DReadOnly bodyFixedPoint)
   {
      return accelerationProvider.getLinearAccelerationOfBodyFixedPoint(base, body, bodyFixedPoint);
   }

   /**
    * Returns the reference to the root body of the system for which this calculator is available.
    *
    * @return the root body.
    */
   public RigidBodyReadOnly getRootBody()
   {
      return rootBody;
   }

   /**
    * Whether rigid-body accelerations resulting from centrifugal and Coriolis effects are considered
    * or ignored.
    *
    * @return {@code true} if this spatial acceleration calculator considers the velocity terms,
    *         {@code false} otherwise.
    */
   @Override
   public boolean areVelocitiesConsidered()
   {
      return doVelocityTerms;
   }

   /**
    * Whether rigid-body accelerations resulting from joint accelerations are considered or ignored.
    *
    * @return {@code true} if this spatial acceleration calculator considers joint accelerations,
    *         {@code false} otherwise.
    */
   @Override
   public boolean areAccelerationsConsidered()
   {
      return doAccelerationTerms;
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #computeOrGetAccelerationOfBody(RigidBodyReadOnly)}.
    */
   private final Twist twistForComputeOrGetTwistOfBody1 = new Twist();
   /**
    * Temporary twist used for intermediate garbage free operations. To use only in the method
    * {@link #computeOrGetAccelerationOfBody(RigidBodyReadOnly)}.
    */
   private final Twist twistForComputeOrGetTwistOfBody2 = new Twist();
   /**
    * Temporary acceleration used for intermediate garbage free operations. To use only in the method
    * {@link #computeOrGetAccelerationOfBody(RigidBodyReadOnly)}.
    */
   private final SpatialAcceleration accelerationForComputeOrGetAccelerationOfBody = new SpatialAcceleration();

   /**
    * Retrieves or computes the acceleration with respect to the inertial frame of the given
    * {@code body}.
    * <p>
    * WARNING: The returned {@code SpatialAccelerationVector} object will remain associated with the
    * given {@code body} and <b> should NOT be modified </b>.
    * </p>
    * <p>
    * In the case the acceleration of the given {@code body} has been computed already no extra
    * computation is done. However, if there is no up-to-date acceleration for this rigid-body, it is
    * then updated by a recursive approach using the following relation: <br>
    * A<sup>b, b</sup><sub>i</sub> = A<sup>p, b</sup><sub>i</sub> + A<sup>b, b</sup><sub>p</sub> </br>
    * where 'b' is the {@code body} frame, 'p' the predecessor frame, and 'i' the inertial frame. <br>
    * Starting from the given {@code body}, its acceleration can be updated using the acceleration of
    * the predecessor to the parent joint. The acceleration of the predecessor is updated in the same
    * manner. This is done recursively until the predecessor has an up-to-date twist or is the root
    * body.
    * </p>
    *
    * @param body the rigid-body to get the twist of.
    * @return the acceleration of the rigid-body with respect to the inertial frame.
    */
   private SpatialAccelerationReadOnly computeOrGetAccelerationOfBody(RigidBodyReadOnly body)
   {
      SpatialAcceleration acceleration = retrieveAssignedAcceleration(body);

      if (acceleration == null)
      {
         /*
          * The body acceleration has not been computed yet. Going up toward the root until we get the
          * acceleration of a rigidBody that is up-to-date, and then going back step-by-step to this body
          * while updating accelerations.
          */
         ReferenceFrame bodyFrame = body.getBodyFixedFrame();
         JointReadOnly parentJoint = body.getParentJoint();
         RigidBodyReadOnly predecessor = parentJoint.getPredecessor();
         acceleration = assignAndGetEmptyAcceleration(body);

         Twist localJointTwist = twistForComputeOrGetTwistOfBody1;
         Twist localPredecessorTwist = twistForComputeOrGetTwistOfBody2;
         SpatialAcceleration localJointAcceleration = accelerationForComputeOrGetAccelerationOfBody;

         MovingReferenceFrame predecessorFrame = predecessor.getBodyFixedFrame();

         acceleration.setIncludingFrame(computeOrGetAccelerationOfBody(predecessor));

         if (doVelocityTerms)
         {
            parentJoint.getPredecessorTwist(localJointTwist);
            predecessorFrame.getTwistOfFrame(localPredecessorTwist);
            acceleration.changeFrame(bodyFrame, localJointTwist, localPredecessorTwist);
         }
         else
         {
            acceleration.changeFrame(bodyFrame);
         }

         if (doAccelerationTerms)
         {
            parentJoint.getSuccessorAcceleration(localJointAcceleration);
            acceleration.add(localJointAcceleration);
         }
         else
         {
            acceleration.setBodyFrame(bodyFrame);
         }
      }

      return acceleration;
   }

   /**
    * Retrieves in the internal memory, the up-to-date acceleration associated with the given
    * {@code body}.
    *
    * @param body the query.
    * @return the up-to-date acceleration of the given {@code body}, returns {@code null} if no
    *         acceleration could be found.
    */
   private SpatialAcceleration retrieveAssignedAcceleration(RigidBodyReadOnly body)
   {
      AccelerationIndex index = rigidBodyToAssignedAccelerationIndex.get(body);

      if (index == null)
      {
         index = new AccelerationIndex(-1);
         rigidBodyToAssignedAccelerationIndex.put(body, index);
      }

      int assignedAccelerationIndex = index.getIndex();
      if (assignedAccelerationIndex == -1)
         return null;
      else
         return assignedAccelerations.get(assignedAccelerationIndex);
   }

   /**
    * Assigned an empty acceleration to the given {@code body} and returns it.
    *
    * @param body the rigid-body to assign a twist to.
    * @return the twist newly assigned to the rigid-body.
    */
   private SpatialAcceleration assignAndGetEmptyAcceleration(RigidBodyReadOnly body)
   {
      SpatialAcceleration newAssignedAcceleration;

      if (unnassignedAccelerations.isEmpty())
         newAssignedAcceleration = new SpatialAcceleration();
      else
         newAssignedAcceleration = unnassignedAccelerations.remove(unnassignedAccelerations.size() - 1);

      AccelerationIndex index = rigidBodyToAssignedAccelerationIndex.get(body);

      if (index == null)
      {
         index = new AccelerationIndex(0);
         rigidBodyToAssignedAccelerationIndex.put(body, index);
      }

      index.setIndex(assignedAccelerations.size());

      rigidBodiesWithAssignedAcceleration.add(body);
      assignedAccelerations.add(newAssignedAcceleration);
      return newAssignedAcceleration;
   }

   /**
    * Used to store an index with {@link Map} while avoiding autoboxing to {@code Integer} and thus
    * generating garbage.
    */
   private static class AccelerationIndex
   {
      private int index = -1;

      public AccelerationIndex(int index)
      {
         this.index = index;
      }

      public void setIndex(int index)
      {
         this.index = index;
      }

      public int getIndex()
      {
         return index;
      }
   }
}
