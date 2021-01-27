package us.ihmc.mecano.algorithms;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.JointStateType;
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
   /**
    * The common ancestor of {@code base} and {@code endEffector}, it is usually equal to {@code base}.
    */
   private RigidBodyReadOnly commonAncestor;
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
   private final DMatrixRMaj jacobianMatrix = new DMatrixRMaj(6, 12);
   /** The time-derivative of the geometric Jacobian matrix. */
   private final DMatrixRMaj jacobianRateMatrix = new DMatrixRMaj(6, 12);
   /**
    * The convective term required when mapping the joint acceleration space to the end-effector
    * spatial acceleration space.
    */
   private final DMatrixRMaj convectiveTerm = new DMatrixRMaj(6, 1);

   /** Intermediate variable to store a joint unit-twist. */
   private final Twist jointUnitTwist = new Twist();
   /** Intermediate variable for garbage free operations. */
   private final DMatrixRMaj spatialVector = new DMatrixRMaj(6, 1);
   /** The total bias acceleration resulting Coriolis and centrifugal accelerations. */
   private final SpatialAcceleration endEffectorCoriolisAcceleration = new SpatialAcceleration();

   /** Intermediate variable used when computing the Jacobian rate. */
   private final SpatialVector twistToEndEffector = new SpatialVector();
   /** Intermediate variable used when computing the Jacobian rate. */
   private final SpatialVector jacobianRateColumn = new SpatialVector();

   private boolean isJacobianUpToDate = false;
   private boolean isJacobianRateUpToDate = false;

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
      commonAncestor = null;
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
      isJacobianRateUpToDate = false;
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
      jacobianFrame = endEffector.getBodyFixedFrame();
      reset();

      numberOfDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(base, endEffector);
      commonAncestor = MultiBodySystemTools.collectJointPath(base, endEffector, jointsFromBaseToEndEffector);
      MultiBodySystemTools.collectRigidBodyPath(base, endEffector, bodyPathFromBaseToEndEffector);
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
    * @throws IllegalArgumentException if the number of joints is equal to 0.
    */
   public void setKinematicChain(JointReadOnly[] joints)
   {
      Objects.requireNonNull(joints);

      if (joints.length == 0)
      {
         throw new IllegalArgumentException("Cannot create a geometric jacobian for an empty kinematic chain.");
      }
      else if (joints.length == 1)
      {
         JointReadOnly joint = joints[0];
         base = joint.getPredecessor();
         endEffector = joint.getSuccessor();
         commonAncestor = base;
         bodyPathFromBaseToEndEffector.clear();
         bodyPathFromBaseToEndEffector.add(base);
         bodyPathFromBaseToEndEffector.add(endEffector);
         numberOfDegreesOfFreedom = joint.getDegreesOfFreedom();
      }
      else
      {
         JointReadOnly firstJoint = joints[0];
         JointReadOnly secondJoint = joints[1];
         JointReadOnly lastJoint = joints[joints.length - 1];
         JointReadOnly secondToLastJoint = joints[joints.length - 2];

         if (firstJoint == secondJoint.getPredecessor().getParentJoint())
            base = firstJoint.getPredecessor();
         else
            base = firstJoint.getSuccessor();

         if (lastJoint == secondToLastJoint.getPredecessor().getParentJoint())
            endEffector = lastJoint.getPredecessor();
         else
            endEffector = lastJoint.getSuccessor();

         commonAncestor = MultiBodySystemTools.collectRigidBodyPath(base, endEffector, bodyPathFromBaseToEndEffector);
         numberOfDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(joints);
      }

      jacobianFrame = endEffector.getBodyFixedFrame();
      reset();

      jointsFromBaseToEndEffector.clear();
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
         jointsFromBaseToEndEffector.add(joints[jointIndex]);
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
      boolean isGoingUpstream = commonAncestor != base;

      int column = 0;

      for (int jointIndex = 0; jointIndex < jointsFromBaseToEndEffector.size(); jointIndex++)
      {
         JointReadOnly joint = jointsFromBaseToEndEffector.get(jointIndex);

         for (int dofIndex = 0; dofIndex < joint.getDegreesOfFreedom(); dofIndex++)
         {
            jointUnitTwist.setIncludingFrame(joint.getUnitTwists().get(dofIndex));
            if (isGoingUpstream)
               jointUnitTwist.invert();
            jointUnitTwist.changeFrame(jacobianFrame);
            jointUnitTwist.get(0, column++, jacobianMatrix);
         }

         if (isGoingUpstream)
            isGoingUpstream = joint.getPredecessor() != commonAncestor;
      }
      isJacobianUpToDate = true;
   }

   private final DMatrixRMaj jointVelocities = new DMatrixRMaj(12, 1);

   /**
    * Computes the time-derivative of the Jacobian matrix JDot<sub>6xN</sub> and the convective term
    * C<sub>6x1</sub> = JDot<sub>6xN</sub> * qDot<sub>Nx1</sub>, where N is the number of degrees of
    * freedom between the {@code base} and {@code endEffector}.
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
   private void updateJacobianRateMatrix()
   {
      if (isJacobianRateUpToDate)
         return;

      updateJacobianMatrix();

      jointVelocities.reshape(numberOfDegreesOfFreedom, 1);
      MultiBodySystemTools.extractJointsState(jointsFromBaseToEndEffector, JointStateType.VELOCITY, jointVelocities);

      jacobianRateMatrix.reshape(SpatialVectorReadOnly.SIZE, numberOfDegreesOfFreedom);

      JointReadOnly joint = jointsFromBaseToEndEffector.get(jointsFromBaseToEndEffector.size() - 1);
      int startCol = numberOfDegreesOfFreedom - joint.getDegreesOfFreedom();
      int endCol = numberOfDegreesOfFreedom;

      fillColumns(startCol, endCol, 0.0, jacobianRateMatrix);
      twistToEndEffector.setToZero();

      for (int jointIdx = jointsFromBaseToEndEffector.size() - 2; jointIdx >= 0; jointIdx--)
      {
         addJointTwist(startCol, endCol, jointVelocities, jacobianMatrix, twistToEndEffector);

         joint = jointsFromBaseToEndEffector.get(jointIdx);
         endCol = startCol;
         startCol -= joint.getDegreesOfFreedom();
         computeJacobianRateMatrixBlock(startCol, endCol, twistToEndEffector);
      }

      CommonOps_DDRM.mult(jacobianRateMatrix, jointVelocities, convectiveTerm);
      endEffectorCoriolisAcceleration.setIncludingFrame(getEndEffectorFrame(), getBaseFrame(), getJacobianFrame(), convectiveTerm);

      isJacobianRateUpToDate = true;
   }

   /**
    * Computes the columns of the Jacobian rate matrix in [{@code startIndex}, {@code endIndex}[.
    * 
    * @param startIndex         the first column index (inclusive) of the Jacobian rate matrix to
    *                           compute.
    * @param endIndex           the last column index (exclusive) of the Jacobian rate matrix to
    *                           compute.
    * @param twistToEndEffector the twist of the end-effector with respect to the joint that maps to
    *                           the columns [{@code startIndex}, {@code endIndex}[ of the Jacobian. Not
    *                           modified.
    */
   private void computeJacobianRateMatrixBlock(int startIndex, int endIndex, SpatialVectorReadOnly twistToEndEffector)
   {
      for (int i = startIndex; i < endIndex; i++)
      {
         jacobianRateColumn.set(0, i, jacobianMatrix);

         // a_<z.a.> = v_J x w_others + w_J x v_others
         jacobianRateColumn.getLinearPart().cross(jacobianRateColumn.getLinearPart(), twistToEndEffector.getAngularPart());
         jacobianRateColumn.addCrossToLinearPart(jacobianRateColumn.getAngularPart(), twistToEndEffector.getLinearPart());

         // w_<z.a.> = w_J x w_others
         jacobianRateColumn.getAngularPart().cross(jacobianRateColumn.getAngularPart(), twistToEndEffector.getAngularPart());

         jacobianRateColumn.get(0, i, jacobianRateMatrix);
      }
   }

   /**
    * Fills the columns of the given matrix to contain {@code value}.
    * 
    * @param startColumn the first column index (inclusive).
    * @param endColumn   the last column index (exclusive).
    * @param value       the value used to fill the columns.
    * @param matrix      the matrix to fill columns of. Modified.
    */
   private static void fillColumns(int startColumn, int endColumn, double value, DMatrix matrix)
   {
      if (startColumn == endColumn)
         return;

      if (startColumn < 0 || startColumn >= matrix.getNumCols() || endColumn < 0 || endColumn > matrix.getNumCols() || endColumn < startColumn)
         throw new IllegalArgumentException("Illegal arguments: startColumn = " + startColumn + " , endColumn = " + endColumn + ".");

      for (int col = startColumn; col < endColumn; col++)
      {
         for (int row = 0; row < matrix.getNumRows(); row++)
         {
            matrix.unsafe_set(row, col, value);
         }
      }
   }

   /**
    * &forall;i &in; [<tt>startIndex</tt>,<tt>endIndex</tt>[ : <tt>vectorToModify += jointVelocities(i)
    * * jacobianMatrix_column(i)
    * 
    * @param startIndex      the first column index (inclusive) in the Jacobian matrix.
    * @param endIndex        the last column index (exclusive) in the Jacobian matrix.
    * @param jointVelocities the column vector containing the joint velocities. Not modified.
    * @param jacobianMatrix  the Jacobian matrix used to retrieve the joint twists. Not modified.
    * @param vectorToModify  the vector to which the joint twist is added to. Modified.
    */
   private static void addJointTwist(int startIndex, int endIndex, DMatrixRMaj jointVelocities, DMatrixRMaj jacobianMatrix, SpatialVectorBasics vectorToModify)
   {
      for (int col = startIndex; col < endIndex; col++)
      {
         double qDot = jointVelocities.get(col);
         double wx = qDot * jacobianMatrix.get(0, col);
         double wy = qDot * jacobianMatrix.get(1, col);
         double wz = qDot * jacobianMatrix.get(2, col);
         double vx = qDot * jacobianMatrix.get(3, col);
         double vy = qDot * jacobianMatrix.get(4, col);
         double vz = qDot * jacobianMatrix.get(5, col);
         vectorToModify.getAngularPart().add(wx, wy, wz);
         vectorToModify.getLinearPart().add(vx, vy, vz);
      }
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
   public void getEndEffectorTwist(DMatrix1Row jointVelocities, TwistBasics endEffectorTwistToPack)
   {
      CommonOps_DDRM.mult(getJacobianMatrix(), jointVelocities, spatialVector);
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
   public void getEndEffectorAcceleration(DMatrix1Row jointAccelerations, SpatialAccelerationBasics spatialAccelerationToPack)
   {
      CommonOps_DDRM.mult(getJacobianMatrix(), jointAccelerations, spatialVector);
      CommonOps_DDRM.addEquals(spatialVector, getConvectiveTermMatrix());
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
   public void getJointTorques(WrenchReadOnly endEffectorWrench, DMatrix1Row jointTorquesToPack)
   {
      endEffectorWrench.checkReferenceFrameMatch(getEndEffectorFrame(), jacobianFrame);
      endEffectorWrench.get(spatialVector);
      jointTorquesToPack.reshape(1, numberOfDegreesOfFreedom);
      CommonOps_DDRM.multTransA(spatialVector, getJacobianMatrix(), jointTorquesToPack);
      jointTorquesToPack.reshape(numberOfDegreesOfFreedom, 1, true); // Quick way to transpose
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
    * Returns the common ancestor to both the {@code base} and the {@code endEffector}.
    * <p>
    * It is usually equal to the {@code base}, but it differs from both {@code base} and
    * {@code endEffector} when expressing a Jacobian for kinematic chain that goes across branches of a
    * tree-shaped multi-body system
    * </p>
    *
    * @return the common ancestor to {@code base} and {@code endEffector}.
    */
   public RigidBodyReadOnly getCommonAncestor()
   {
      return commonAncestor;
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
   public DMatrixRMaj getJacobianMatrix()
   {
      updateJacobianMatrix();
      return jacobianMatrix;
   }

   /**
    * Gets the current value of the time-derivative of the Jacobian matrix.
    * <p>
    * <b>WARNING: The {@code jacobianFrame} is assumed to be rigidly attached to the end-effector.</b>
    * </p>
    * 
    * @return the current value of the time-derivative of the Jacobian matrix.
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   public DMatrixRMaj getJacobianRateMatrix()
   {
      updateJacobianRateMatrix();
      return jacobianRateMatrix;
   }

   /**
    * Gets the current value of the convective term: JDot * qDot.
    * <p>
    * As for the Jacobian matrix, the convective term is computed in {@link #jacobianFrame}.
    * </p>
    * <p>
    * <b>WARNING: The {@code jacobianFrame} is assumed to be rigidly attached to the end-effector.</b>
    * </p>
    *
    * @return the current value of the convective term.
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   public DMatrixRMaj getConvectiveTermMatrix()
   {
      updateJacobianRateMatrix();
      return convectiveTerm;
   }

   /**
    * Gets the current value of the convective term: JDot * qDot.
    * <p>
    * As for the Jacobian matrix, the convective term is computed in {@link #jacobianFrame}.
    * </p>
    * <p>
    * <b>WARNING: The {@code jacobianFrame} is assumed to be rigidly attached to the end-effector.</b>
    * </p>
    *
    * @return the current value of the convective term.
    * @throws RuntimeException if either the base or the end-effector has not been provided beforehand.
    */
   public SpatialAccelerationReadOnly getConvectiveTerm()
   {
      updateJacobianRateMatrix();
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

   /**
    * Creates a short descriptive {@code String} for this Jacobian containing information such as the
    * {@code jacobianFrame}, the {@code base} and {@code endEffector}.
    *
    * @return a short description {@code String} for this Jacobian.
    */
   public String getShortInfo()
   {
      return "Jacobian, end effector = " + getEndEffector() + ", base = " + getBase() + ", expressed in " + getJacobianFrame();
   }
}
