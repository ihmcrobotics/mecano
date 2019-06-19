package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Computes the geometric Jacobian that maps joint velocity space to a given end-effector spatial
 * velocity space.
 * <p>
 * In addition, by using the convective term, the mapping from joint acceleration space to
 * end-effector spatial acceleration space can be formulated as follows:
 * 
 * <pre>
 *                   / d  \
 * xDDot = J qDDot + |-- J| qDot = J qDDot + b
 *                   \dt  /
 * </pre>
 * 
 * where <tt>qDot</tt> and <tt>qDDot</tt> are the joint velocity and acceleration vectors,
 * <tt>xDDot</tt> is the end-effector spatial acceleration, and <tt>J</tt> is the geometric
 * Jacobian. The term <tt>b</tt> introduced on the right-hand side is called here the convective
 * term.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class GeometricJacobianCalculator
{
   /** The base is the predecessor of the first joint that the Jacobian considers. */
   private RigidBodyReadOnly base;
   /** The end-effector is the successor of the last joint that the Jacobian considers. */
   private RigidBodyReadOnly endEffector;
   /** The list of the joints in order that they are represented in the Jacobian matrix. */
   private final List<JointReadOnly> jointsFromBaseToEndEffector = new ArrayList<>(12);
   /**
    * The continuous chain rigid-bodies starting from the base and ending at the end-effector. It is
    * used for computing the Coriolis and centrifugal acceleration of the end-effector efficiently.
    */
   private final List<RigidBodyReadOnly> bodyPathFromBaseToEndEffector = new ArrayList<>(12);
   /** The frame in which the Jacobian is expressed. */
   private ReferenceFrame jacobianFrame;

   /** The number of degrees of freedom of the current kinematic chain. */
   private int numberOfDegreesOfFreedom;
   /** The geometric Jacobian matrix. */
   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(6, 12);
   /**
    * The convective term required when mapping the joint acceleration space to the end-effector
    * spatial acceleration space.
    */
   private final DenseMatrix64F convectiveTerm = new DenseMatrix64F(6, 1);

   /** Intermediate variable to store a joint unit-twist. */
   private final Twist jointUnitTwist = new Twist();
   /** Intermediate variable for garbage free operations. */
   private final DenseMatrix64F spatialVector = new DenseMatrix64F(6, 1);
   /** The total bias acceleration resulting Coriolis and centrifugal accelerations. */
   private final SpatialAcceleration endEffectorCoriolisAcceleration = new SpatialAcceleration();
   /**
    * Intermediate variable store the twist of a rigid-body in the kinematic chain with respect to the
    * base. Used for computing the Coriolis and centrifugal accelerations.
    */
   private final Twist bodyTwistRelativeToBase = new Twist();
   /**
    * Intermediate variable store the successor with respect to the predecessor of a joint in the
    * kinematic chain. Used for computing the Coriolis and centrifugal accelerations.
    */
   private final Twist predecessorTwist = new Twist();

   private boolean isJacobianUpToDate = false;
   private boolean isConvectiveTermUpToDate = false;

   /**
    * Creates an empty calculator.
    * <p>
    * At least the base, end-effector of the desired Jacobian are to be provided before trying to
    * compute it.
    * </p>
    */
   public GeometricJacobianCalculator()
   {
      clear();
   }

   /**
    * Clears the internal memory of this calculator, ensuring new data has to be provided, including
    * base and end-effector.
    */
   public void clear()
   {
      base = null;
      endEffector = null;
      numberOfDegreesOfFreedom = -1;
      jacobianFrame = null;
      jointsFromBaseToEndEffector.clear();

      reset();
   }

   /**
    * Clears the internal memory about the Jacobian matrix and the convective term.
    */
   public void reset()
   {
      isJacobianUpToDate = false;
      isConvectiveTermUpToDate = false;
   }

   /**
    * Sets the new kinematic chain the Jacobian is to be computed for.
    * <p>
    * Note that if the Jacobian frame was not set beforehand, it is automatically set to
    * {@code endEffector.getBodyFixedFrame()}. It can be changed via
    * {@link #setJacobianFrame(ReferenceFrame)}.
    * </p>
    * <p>
    * The internal memory is automatically cleared with {@link #reset()}.
    * </p>
    * 
    * @param base        the new base to use. Not modified.
    * @param endEffector the new end-effector to use. Not modified.
    * @throws IllegalArgumentException if the {@code base} is not an ancestor of {@code endEffector}.
    */
   public void setKinematicChain(RigidBodyReadOnly base, RigidBodyReadOnly endEffector)
   {
      this.base = base;
      this.endEffector = endEffector;
      if (jacobianFrame == null)
         jacobianFrame = endEffector.getBodyFixedFrame();
      reset();

      jointsFromBaseToEndEffector.clear();
      int distanceFromBase = MultiBodySystemTools.computeDistanceToAncestor(endEffector, base);
      if (distanceFromBase < 0)
         throw new IllegalArgumentException("The base: " + base.getName() + "is not an ancestor of the given end-effector: " + endEffector.getName());

      while (jointsFromBaseToEndEffector.size() < distanceFromBase)
         jointsFromBaseToEndEffector.add(null);

      numberOfDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(base, endEffector);

      RigidBodyReadOnly currentBody = endEffector;
      int index = jointsFromBaseToEndEffector.size() - 1;

      while (currentBody != base)
      {
         JointReadOnly joint = currentBody.getParentJoint();
         jointsFromBaseToEndEffector.set(index--, joint);
         currentBody = joint.getPredecessor();
      }

      updateRigidBodyPath();
   }

   /**
    * Sets the new kinematic chain the Jacobian is to be computed for.
    * <p>
    * Note that if the Jacobian frame was not set beforehand, it is automatically set to
    * {@code endEffector.getBodyFixedFrame()}. It can be changed via
    * {@link #setJacobianFrame(ReferenceFrame)}.
    * </p>
    * <p>
    * This method orders if necessary the joints such that the resulting kinematic chain starts from
    * the base and ends at the end-effector.
    * </p>
    * 
    * @param joints the array of joints to use for computing the Jacobian. Not modified.
    * @throws IllegalArgumentException if {@code joints[0].getPredecessor()} is not the ancestor of
    *                                  {@code joints[joints.length - 1].getSuccessor()} and that
    *                                  {@code joints[joints.length - 1].getPredecessor()} is not the
    *                                  ancestor of {@code joints[0].getSuccessor()}.
    */
   public void setKinematicChain(JointReadOnly[] joints)
   {
      base = joints[0].getPredecessor();
      endEffector = joints[joints.length - 1].getSuccessor();

      numberOfDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(joints);

      if (MultiBodySystemTools.isAncestor(endEffector, base))
      {
         if (jacobianFrame == null)
            jacobianFrame = endEffector.getBodyFixedFrame();
         reset();

         jointsFromBaseToEndEffector.clear();
         for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
            jointsFromBaseToEndEffector.add(joints[jointIndex]);
      }
      else
      {
         base = joints[joints.length - 1].getPredecessor();
         endEffector = joints[0].getSuccessor();
         if (!MultiBodySystemTools.isAncestor(endEffector, base))
            throw new IllegalArgumentException("Unable to process the array of joints: " + Arrays.toString(joints));

         if (jacobianFrame == null)
            jacobianFrame = endEffector.getBodyFixedFrame();
         reset();

         jointsFromBaseToEndEffector.clear();
         for (int jointIndex = joints.length - 1; jointIndex >= 0; jointIndex--)
            jointsFromBaseToEndEffector.add(joints[jointIndex]);
      }

      updateRigidBodyPath();
   }

   /**
    * Updates {@link #bodyPathFromBaseToEndEffector} based on the new {@code base} and
    * {@code endEffector}.
    */
   private void updateRigidBodyPath()
   {
      bodyPathFromBaseToEndEffector.clear();
      int pathLength = MultiBodySystemTools.computeDistanceToAncestor(endEffector, base) + 1;

      while (bodyPathFromBaseToEndEffector.size() < pathLength)
         bodyPathFromBaseToEndEffector.add(null);

      int index = pathLength - 1;
      RigidBodyReadOnly currentBody = endEffector;

      while (currentBody != base)
      {
         bodyPathFromBaseToEndEffector.set(index--, currentBody);
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      bodyPathFromBaseToEndEffector.set(0, base);
   }

   /**
    * Sets the reference frame in which the Jacobian should be expressed.
    * <p>
    * Usually the Jacobian frame is set to match the frame on the end-effector that is to be
    * controlled.
    * </p>
    * 
    * @param jacobianFrame the new frame for the Jacobian matrix.
    */
   public void setJacobianFrame(ReferenceFrame jacobianFrame)
   {
      if (jacobianFrame == this.jacobianFrame)
         return;
      this.jacobianFrame = jacobianFrame;
      reset();
   }

   /**
    * Updates the values of the Jacobian matrix.
    * <p>
    * The base, end-effector, and jacobian frame, have all to be properly set before calling this
    * method.
    * </p>
    * 
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   private void updateJacobianMatrix()
   {
      if (isJacobianUpToDate)
         return;

      if (base == null || endEffector == null)
         throw new RuntimeException("The base and end-effector have to be set first.");

      jacobianMatrix.reshape(SpatialVectorReadOnly.SIZE, numberOfDegreesOfFreedom);

      int column = 0;

      for (int jointIndex = 0; jointIndex < jointsFromBaseToEndEffector.size(); jointIndex++)
      {
         JointReadOnly joint = jointsFromBaseToEndEffector.get(jointIndex);

         for (int dofIndex = 0; dofIndex < joint.getDegreesOfFreedom(); dofIndex++)
         {
            jointUnitTwist.setIncludingFrame(joint.getUnitTwists().get(dofIndex));
            jointUnitTwist.changeFrame(jacobianFrame);
            jointUnitTwist.get(0, column++, jacobianMatrix);
         }
      }
      isJacobianUpToDate = true;
   }

   /**
    * Computes the convective term C<sub>6x1</sub> = JDot<sub>6xN</sub> * qDot<sub>Nx1</sub>.<br>
    * where N is the number of degrees of freedom between the {@code base} and {@code endEffector},
    * JDot<sub>6xN</sub> is the time-derivative of the Jacobian matrix, and qDot<sub>Nx1</sub> the
    * vector of joint velocities.
    * <p>
    * <b>WARNING: The {@code jacobianFrame} is assumed to be rigidly attached to the end-effector.</b>
    * </p>
    * <p>
    * The convective term represents the Coriolis acceleration of the {@code endEffector} relative to
    * the {@code base} expressed at the {@code jacobianFrame}. The Coriolis acceleration is an
    * acceleration that results from the combination of the angular and linear velocities happening on
    * the end-effector.
    * </p>
    * <p>
    * As shown in <a href="https://en.wikipedia.org/wiki/Coriolis_force">Wikipedia</a>, the Coriolis
    * acceleration also depends on the velocity of the frame it is expressed in. This method assumes
    * that the {@code jacobianFrame} is attached to the end-effector. If this is not the case, the
    * computed acceleration will be biased.
    * </p>
    * <p>
    * Using the convective term, the spatial acceleration xDDot<sub>6x1</sub> of the
    * {@code endEffector} relative to the {@code base} expressed in the {@code jacbianFrame} can be
    * computed as follows:<br>
    * xDDot<sub>6x1</sub> = J<sub>6xN</sub> * qDDot<sub>Nx1</sub> + C<sub>6x1</sub><br>
    * where qDDot<sub>Nx1</sub> is the vector of joint accelerations.
    * </p>
    * <p>
    * The base, end-effector, and jacobian frame, have all to be properly set before calling this
    * method.
    * </p>
    * 
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   private void updateConvectiveTerm()
   {
      if (isConvectiveTermUpToDate)
         return;

      if (base == null || endEffector == null)
         throw new RuntimeException("The base and end-effector have to be set first.");

      MovingReferenceFrame baseFrame = base.getBodyFixedFrame();
      endEffectorCoriolisAcceleration.setToZero(baseFrame, baseFrame, baseFrame);

      for (int i = 1; i < bodyPathFromBaseToEndEffector.size(); i++)
      {
         RigidBodyReadOnly currentBody = bodyPathFromBaseToEndEffector.get(i);
         JointReadOnly parentJoint = currentBody.getParentJoint();
         MovingReferenceFrame currentBodyFrame = currentBody.getBodyFixedFrame();
         MovingReferenceFrame predecessorBodyFrame = parentJoint.getPredecessor().getBodyFixedFrame();

         parentJoint.getPredecessorTwist(predecessorTwist);
         predecessorBodyFrame.getTwistRelativeToOther(baseFrame, bodyTwistRelativeToBase);

         // By changing the zero-acceleration from body to body until the end-effector, we collect Coriolis and centrifugal accelerations.
         endEffectorCoriolisAcceleration.changeFrame(currentBodyFrame, predecessorTwist, bodyTwistRelativeToBase);
         endEffectorCoriolisAcceleration.setBodyFrame(currentBodyFrame);
      }

      // The following line is where the jacobianFrame is assumed to be rigidly attached to the end-effector.
      endEffectorCoriolisAcceleration.changeFrame(jacobianFrame);

      convectiveTerm.reshape(6, 1);
      endEffectorCoriolisAcceleration.get(convectiveTerm);

      isConvectiveTermUpToDate = true;
   }

   /**
    * Computes and packs the twist of the end-effector relative to the base that is induced by the
    * given joint velocity vector.
    * 
    * @param jointVelocities        the joint velocity column vector, starting from base child joint
    *                               velocity. Not modified.
    * @param endEffectorTwistToPack the twist of the end effector with respect to the base, expressed
    *                               in the jacobianFrame. Modified.
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   public void getEndEffectorTwist(DenseMatrix64F jointVelocities, TwistBasics endEffectorTwistToPack)
   {
      CommonOps.mult(getJacobianMatrix(), jointVelocities, spatialVector);
      endEffectorTwistToPack.setIncludingFrame(getEndEffectorFrame(), getBaseFrame(), jacobianFrame, spatialVector);
   }

   /**
    * Computes and packs the spatial acceleration of the end-effector relative to the base that is
    * induced by the given joint acceleration vector.
    * 
    * @param jointAccelerations        the joint acceleration column vector, starting from base child
    *                                  joint acceleration. Not modified.
    * @param spatialAccelerationToPack the spatial acceleration of the end effector with respect to the
    *                                  base, expressed in the jacobianFrame. Modified.
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   public void getEndEffectorAcceleration(DenseMatrix64F jointAccelerations, SpatialAccelerationBasics spatialAccelerationToPack)
   {
      CommonOps.mult(getJacobianMatrix(), jointAccelerations, spatialVector);
      CommonOps.addEquals(spatialVector, getConvectiveTermMatrix());
      spatialAccelerationToPack.setIncludingFrame(getEndEffectorFrame(), getBaseFrame(), jacobianFrame, spatialVector);
   }

   /**
    * Computes and packs the joint torque vector that corresponds to the given wrench.
    * 
    * @param endEffectorWrench  the resulting wrench at the end effector. The wrench should be
    *                           expressed in {@code jacobianFrame} and the wrench's {@code bodyFrame}
    *                           should be the body fixed frame of the end-effector. Not modified.
    * @param jointTorquesToPack the dense matrix used to store the computed joint torques. Modified.
    * @throws ReferenceFrameMismatchException if the given wrench
    *                                         {@code wrench.getExpressedInFrame() != this.getJacobianFrame()}
    *                                         or
    *                                         {@code wrench.getBodyFrame() != this.getEndEffectorFrame()}.
    * @throws RuntimeException                if either the base or the end-effector has not been
    *                                         provided beforehand.
    */
   public void getJointTorques(WrenchReadOnly endEffectorWrench, DenseMatrix64F jointTorquesToPack)
   {
      endEffectorWrench.checkReferenceFrameMatch(getEndEffectorFrame(), jacobianFrame);
      endEffectorWrench.get(spatialVector);
      jointTorquesToPack.reshape(1, numberOfDegreesOfFreedom);
      CommonOps.multTransA(spatialVector, getJacobianMatrix(), jointTorquesToPack);
      CommonOps.transpose(jointTorquesToPack);
   }

   /**
    * Returns the base {@code RigidBody} of the current Jacobian. The base is the predecessor of the
    * first joint that the Jacobian considers.
    * 
    * @return the base of the Jacobian.
    */
   public RigidBodyReadOnly getBase()
   {
      return base;
   }

   /**
    * Returns the body fixed frame of the base {@code RigidBody} of the current Jacobian. The base is
    * the predecessor of the first joint that the Jacobian considers.
    * 
    * @return the body fixed frame of the base.
    */
   public ReferenceFrame getBaseFrame()
   {
      return base.getBodyFixedFrame();
   }

   /**
    * Returns the end-effector {@code RigidBody} of the current Jacobian. The end-effector is the
    * successor of the last joint the Jacobian considers.
    * 
    * @return the end-effector of the jacobian.
    */
   public RigidBodyReadOnly getEndEffector()
   {
      return endEffector;
   }

   /**
    * Returns the body fixed frame of the end-effector {@code RigidBody} of the current Jacobian. The
    * end-effector is the successor of the last joint the Jacobian considers.
    * 
    * @return the body fixed frame of the end-effector.
    */
   public ReferenceFrame getEndEffectorFrame()
   {
      return endEffector.getBodyFixedFrame();
   }

   /**
    * @return the frame in which the current Jacobian is expressed.
    */
   public ReferenceFrame getJacobianFrame()
   {
      return jacobianFrame;
   }

   /**
    * @return the number of degrees of freedom that the current kinematic chain considered contains.
    */
   public int getNumberOfDegreesOfFreedom()
   {
      return numberOfDegreesOfFreedom;
   }

   /**
    * Gets the list of joints considered by the current Jacobian ordered from the base to the
    * end-effector.
    * 
    * @return the list of joints.
    */
   public List<JointReadOnly> getJointsFromBaseToEndEffector()
   {
      return jointsFromBaseToEndEffector;
   }

   /**
    * Gets the current value of the Jacobian matrix.
    * 
    * @return the current value of the Jacobian matrix.
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   public DenseMatrix64F getJacobianMatrix()
   {
      updateJacobianMatrix();
      return jacobianMatrix;
   }

   /**
    * Gets the current value of the convective term: JDot * qDot.
    * <p>
    * As for the Jacobian matrix, the convective term is computed in {@link #jacobianFrame}.
    * </p>
    * 
    * @return the current value of the convective term.
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   public DenseMatrix64F getConvectiveTermMatrix()
   {
      updateConvectiveTerm();
      return convectiveTerm;
   }

   /**
    * Gets the current value of the convective term: JDot * qDot.
    * <p>
    * As for the Jacobian matrix, the convective term is computed in {@link #jacobianFrame}.
    * </p>
    * 
    * @return the current value of the convective term.
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   public SpatialAccelerationReadOnly getConvectiveTerm()
   {
      updateConvectiveTerm();
      return endEffectorCoriolisAcceleration;
   }

   /**
    * Creates a descriptive {@code String} for this Jacobian containing information such as the
    * {@code jacobianFrame}, the list of the joint names, and the current value of the Jacobian matrix.
    * 
    * @return a descriptive {@code String} for this Jacobian.
    */
   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("Jacobian. jacobianFrame = " + jacobianFrame + ". Joints:\n");

      RigidBodyReadOnly currentBody = endEffector;

      while (currentBody != base)
      {
         JointReadOnly joint = currentBody.getParentJoint();
         builder.append(joint.getClass().getSimpleName() + " " + joint.getName() + "\n");
         currentBody = joint.getPredecessor();
      }

      builder.append("\n");
      builder.append(jacobianMatrix.toString());

      return builder.toString();
   }

   public String getShortInfo()
   {
      return "Jacobian, end effector = " + getEndEffector() + ", base = " + getBase() + ", expressed in " + getJacobianFrame();
   }

}
