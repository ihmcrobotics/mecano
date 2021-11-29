package us.ihmc.mecano.fourBar;

import java.util.Arrays;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.fourBar.FourBarKinematicLoopFunctionTools.FourBarToJointConverter;
import us.ihmc.mecano.multiBodySystem.interfaces.KinematicLoopFunction;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;

/**
 * This class can be used to help enforcing the physical constraint of a four bar linkage given its
 * four joints.
 * <p>
 * Upon construction, the given joints are named to follow one of these layouts:
 * 
 * <pre>
 *      root            root
 *        |               |
 *        |               |
 *   A O-----O B     A O-----O B
 *     |     |          \   /
 *     |     |           \ /
 *     |     |    or      X
 *     |     |           / \
 *     |     |          /   \
 *   D O-----O C     C O-----O D
 *        |               |
 *   end-effector    end-effector
 * </pre>
 * </p>
 */
public class FourBarKinematicLoopFunction implements KinematicLoopFunction
{
   private static final double EPSILON = 1.0e-7;

   private final String name;
   private final FourBarVertex actuatedVertex;
   private final FourBar fourBar = new FourBar();
   private final RevoluteJointBasics jointA;
   private final RevoluteJointBasics jointB;
   private final RevoluteJointBasics jointC;
   private final RevoluteJointBasics jointD;
   private final List<RevoluteJointBasics> joints;

   private final FourBarToJointConverter converterA = new FourBarToJointConverter();
   private final FourBarToJointConverter converterB = new FourBarToJointConverter();
   private final FourBarToJointConverter converterC = new FourBarToJointConverter();
   private final FourBarToJointConverter converterD = new FourBarToJointConverter();
   private final FourBarToJointConverter[] converters = {converterA, converterB, converterC, converterD};

   private final DMatrixRMaj loopJacobianMatrix = new DMatrixRMaj(4, 1);
   private final DMatrixRMaj loopConvectiveTermMatrix = new DMatrixRMaj(4, 1);

   private final int actuatedJointIndex;
   private final int[] actuatedJointIndices;

   /**
    * Creates a new function to manage the physical constraint of a four bar linkage given its 4
    * joints.
    * 
    * @param name             the name of this four bar.
    * @param joints           the four joints composing the four bar linkage.
    * @param actuatedJointIndex the index among {@code joints} of the actuated joint, i.e. the only joint
    *                         which state defines the entire state of the four bar and which is
    *                         expected to be the only actuated joint.
    * @throws IllegalArgumentException if a four bar linkage could not be recognized from the given
    *                                  joints.
    * @see FourBarKinematicLoopFunctionTools#configureFourBarKinematics(RevoluteJointBasics[],
    *      FourBarToJointConverter[], FourBar, int, double)
    */
   public FourBarKinematicLoopFunction(String name, List<? extends RevoluteJointBasics> joints, int actuatedJointIndex)
   {
      this(name, joints.toArray(new RevoluteJointBasics[0]), actuatedJointIndex);
   }

   /**
    * Creates a new function to manage the physical constraint of a four bar linkage given its 4
    * joints.
    * 
    * @param name             the name of this four bar.
    * @param joints           the four joints composing the four bar linkage.
    * @param actuatedJointIndex the index among {@code joints} of the actuated joint, i.e. the only joint
    *                         which state defines the entire state of the four bar and which is
    *                         expected to be the only actuated joint.
    * @throws IllegalArgumentException if a four bar linkage could not be recognized from the given
    *                                  joints.
    * @see FourBarKinematicLoopFunctionTools#configureFourBarKinematics(RevoluteJointBasics[],
    *      FourBarToJointConverter[], FourBar, int, double)
    */
   public FourBarKinematicLoopFunction(String name, RevoluteJointBasics[] joints, int actuatedJointIndex)
   {
      this.name = name;

      // Copy the array so it cannot modified externally and the argument doesn't get modified. 
      joints = Arrays.copyOf(joints, joints.length);
      this.actuatedJointIndex = FourBarKinematicLoopFunctionTools.configureFourBarKinematics(joints, converters, fourBar, actuatedJointIndex, EPSILON);
      actuatedJointIndices = new int[] {this.actuatedJointIndex};
      this.joints = Arrays.asList(joints);
      jointA = joints[0];
      jointB = joints[1];
      jointC = joints[2];
      jointD = joints[3];

      actuatedVertex = fourBar.getVertex(FourBarAngle.values[this.actuatedJointIndex]);
   }

   /**
    * Tests if this four bar represents a cross four bar as follows:
    *
    * <pre>
    *    root
    *      |
    *      |
    * A O-----O B
    *    \   /
    *     \ /
    *      X
    *     / \
    *    /   \
    * C O-----O D
    *      |
    * end-effector
    * </pre>
    * 
    * @return {@code true} if this four bar is a cross four bar, {@code false} otherwise.
    */
   public boolean isCrossed()
   {
      return fourBar.isCrossed();
   }

   /**
    * Assuming the state of the actuated joint has been set, this method computes and updates the state
    * of the other joints such that the kinematic loop represents a proper four bar linkage with
    * constant side lengths.
    * <p>
    * This method also updates the loop Jacobian and convective term.
    * </p>
    * 
    * @param updateVelocity     whether the joint velocities should be computed and updated.
    * @param updateAcceleration whether the joint accelerations should be computed and updated,
    *                           requires {@code updateVelocity = true}.
    * @return if the configuration is within the four bar range, this method returns {@code null},
    *         otherwise returns the bound to which the configuration has been clamped.
    */
   public Bound updateState(boolean updateVelocity, boolean updateAcceleration)
   {
      if (updateAcceleration && !updateVelocity)
         throw new IllegalArgumentException("updateVelocity needs to be true for updateAcceleration to be true.");

      clampActuatedJointPosition();

      RevoluteJointBasics actuatedJoint = getActuatedJoint();
      FourBarToJointConverter actuatedConverter = converters[actuatedJointIndex];
      double angle = actuatedConverter.toFourBarInteriorAngle(actuatedJoint.getQ());
      Bound limit;

      if (updateVelocity)
      {
         double angleDot = actuatedConverter.toFourBarInteriorAngularDerivative(actuatedJoint.getQd());

         if (updateAcceleration)
         {
            double angleDDot = actuatedConverter.toFourBarInteriorAngularDerivative(actuatedJoint.getQdd());
            limit = fourBar.update(FourBarAngle.values[actuatedJointIndex], angle, angleDot, angleDDot);
         }
         else
         {
            limit = fourBar.update(FourBarAngle.values[actuatedJointIndex], angle, angleDot);
         }
      }
      else
      {
         limit = fourBar.update(FourBarAngle.values[actuatedJointIndex], angle);
      }

      for (int i = 0; i < 4; i++)
      {
         if (i == actuatedJointIndex)
            continue;

         RevoluteJointBasics joint = joints.get(i);
         FourBarToJointConverter converter = converters[i];
         FourBarVertex fourBarVertex = fourBar.getVertex(FourBarAngle.values[i]);

         joint.setQ(converter.toJointAngle(fourBarVertex.getAngle()));
         if (updateVelocity)
            joint.setQd(converter.toJointDerivative(fourBarVertex.getAngleDot()));
         if (updateAcceleration)
            joint.setQdd(converter.toJointDerivative(fourBarVertex.getAngleDDot()));
      }

      updateLoopJacobian();
      updateLoopConvectiveTerm();
      return limit;
   }

   /**
    * Assuming the effort for each joint has been previously updated, this method centralizes the
    * efforts on the actuated joint while preserving the resulting dynamics of the linkage.
    */
   public void updateEffort()
   {
      double tau = 0.0;

      for (int i = 0; i < 4; i++)
      {
         RevoluteJointBasics joint = joints.get(i);
         tau += loopJacobianMatrix.get(i, 0) * joint.getTau();
         joint.setTau(0.0);
      }

      getActuatedJoint().setTau(tau / loopJacobianMatrix.get(actuatedJointIndex, 0));
   }

   /** {@inheritDoc} */
   @Override
   public void adjustConfiguration(DMatrixRMaj jointConfigurations)
   {
      if (jointConfigurations.getNumRows() != 4 || jointConfigurations.getNumCols() != 1)
         throw new IllegalArgumentException("Unexpected matrix size. [row=" + jointConfigurations.getNumRows() + ", col=" + jointConfigurations.getNumCols()
               + "].");

      double angle = EuclidCoreTools.clamp(jointConfigurations.get(actuatedJointIndex),
                                           getActuatedJoint().getJointLimitLower(),
                                           getActuatedJoint().getJointLimitUpper());
      fourBar.update(FourBarAngle.values[actuatedJointIndex], converters[actuatedJointIndex].toFourBarInteriorAngle(angle));

      for (int i = 0; i < 4; i++)
      {
         double q = converters[i].toJointAngle(fourBar.getVertex(FourBarAngle.values[i]).getAngle());
         jointConfigurations.set(i, q);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void adjustTau(DMatrixRMaj tauJoints)
   {
      if (tauJoints.getNumRows() != 4 || tauJoints.getNumCols() != 1)
         throw new IllegalArgumentException("Unexpected matrix size. [row=" + tauJoints.getNumRows() + ", col=" + tauJoints.getNumCols() + "].");

      double tau = 0.0;

      for (int i = 0; i < 4; i++)
      {
         tau += loopJacobianMatrix.get(i, 0) * tauJoints.get(i);
         tauJoints.set(i, 0.0);
      }

      tauJoints.set(actuatedJointIndex, tau / loopJacobianMatrix.get(actuatedJointIndex, 0));
   }

   private void updateLoopJacobian()
   {
      // Definitely not the most effective to compute the Jacobian but should not matter compared to the rest of the controller's computational load.
      double angle = converters[actuatedJointIndex].toFourBarInteriorAngle(getActuatedJoint().getQ());
      double angleDot = converters[actuatedJointIndex].toFourBarInteriorAngularDerivative(1.0);
      fourBar.update(FourBarAngle.values[actuatedJointIndex], angle, angleDot);

      for (int i = 0; i < 4; i++)
      {
         if (i == actuatedJointIndex)
         {
            loopJacobianMatrix.set(i, 0, 1.0);
         }
         else
         {
            FourBarVertex fourBarVertex = fourBar.getVertex(FourBarAngle.values[i]);
            loopJacobianMatrix.set(i, 0, converters[i].toJointDerivative(fourBarVertex.getAngleDot()));
         }
      }
   }

   private void updateLoopConvectiveTerm()
   {
      // Definitely not the most effective to compute the convective term but should not matter compared to the rest of the controller's computational load.
      double angle = converters[actuatedJointIndex].toFourBarInteriorAngle(getActuatedJoint().getQ());
      double angleDot = converters[actuatedJointIndex].toFourBarInteriorAngularDerivative(getActuatedJoint().getQd());
      double angleDDot = 0.0;
      fourBar.update(FourBarAngle.values[actuatedJointIndex], angle, angleDot, angleDDot);

      for (int i = 0; i < 4; i++)
      {
         if (i == actuatedJointIndex)
         {
            loopConvectiveTermMatrix.set(i, 0, 0.0);
         }
         else
         {
            FourBarVertex fourBarVertex = fourBar.getVertex(FourBarAngle.values[i]);
            loopConvectiveTermMatrix.set(i, 0, converters[i].toJointDerivative(fourBarVertex.getAngleDDot()));
         }
      }
   }

   /** {@inheritDoc} */
   @Override
   public List<RevoluteJointBasics> getLoopJoints()
   {
      return joints;
   }

   private void clampActuatedJointPosition()
   {
      RevoluteJointBasics actuatedJoint = getActuatedJoint();

      if (actuatedJoint.getQ() < actuatedJoint.getJointLimitLower())
      {
         actuatedJoint.setQ(actuatedJoint.getJointLimitLower());
      }
      else if (actuatedJoint.getQ() > actuatedJoint.getJointLimitUpper())
      {
         actuatedJoint.setQ(actuatedJoint.getJointLimitUpper());
      }
   }

   /**
    * Gets the name of the four bar this function is for.
    * 
    * @return the name of the four bar.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Returns one of the two joints starting the linkage:
    * 
    * <pre>
    *      root            root
    *        |               |
    *        |               |
    *   A O-----O B     A O-----O B
    *     |     |          \   /
    *     |     |           \ /
    *     |     |    or      X
    *     |     |           / \
    *     |     |          /   \
    *   D O-----O C     C O-----O D
    *        |               |
    *   end-effector    end-effector
    * </pre>
    * 
    * @return the reference to the joint A.
    */
   public RevoluteJointBasics getJointA()
   {
      return jointA;
   }

   /**
    * Returns one of the two joints starting the linkage:
    * 
    * <pre>
    *      root            root
    *        |               |
    *        |               |
    *   A O-----O B     A O-----O B
    *     |     |          \   /
    *     |     |           \ /
    *     |     |    or      X
    *     |     |           / \
    *     |     |          /   \
    *   D O-----O C     C O-----O D
    *        |               |
    *   end-effector    end-effector
    * </pre>
    * 
    * @return the reference to the joint B.
    */
   public RevoluteJointBasics getJointB()
   {
      return jointB;
   }

   /**
    * Returns one of the two joints ending the linkage:
    * 
    * <pre>
    *      root            root
    *        |               |
    *        |               |
    *   A O-----O B     A O-----O B
    *     |     |          \   /
    *     |     |           \ /
    *     |     |    or      X
    *     |     |           / \
    *     |     |          /   \
    *   D O-----O C     C O-----O D
    *        |               |
    *   end-effector    end-effector
    * </pre>
    * 
    * @return the reference to the joint C.
    */
   public RevoluteJointBasics getJointC()
   {
      return jointC;
   }

   /**
    * Returns one of the two joints ending the linkage:
    * 
    * <pre>
    *      root            root
    *        |               |
    *        |               |
    *   A O-----O B     A O-----O B
    *     |     |          \   /
    *     |     |           \ /
    *     |     |    or      X
    *     |     |           / \
    *     |     |          /   \
    *   D O-----O C     C O-----O D
    *        |               |
    *   end-effector    end-effector
    * </pre>
    * 
    * @return the reference to the joint D.
    */
   public RevoluteJointBasics getJointD()
   {
      return jointD;
   }

   /**
    * The index of the actuated joint in the list {@link #getLoopJoints()}.
    * 
    * @return the index of the actuated joint.
    */
   public int getActuatedJointIndex()
   {
      return actuatedJointIndex;
   }

   /**
    * The vertex of {@link #getFourBar()} that corresponds to the actuated joint.
    * 
    * @return the actuated vertex.
    */
   public FourBarVertex getActuatedVertex()
   {
      return actuatedVertex;
   }

   /** {@inheritDoc} */
   @Override
   public int[] getActuatedJointIndices()
   {
      return actuatedJointIndices;
   }

   /**
    * Returns the reference to the actuated joint of this four bar, i.e. the only joint which state
    * defines the entire state of the four bar and which is expected to be the only actuated joint.
    * 
    * @return the reference to the actuated joint.
    */
   public RevoluteJointBasics getActuatedJoint()
   {
      return joints.get(actuatedJointIndex);
   }

   /** {@inheritDoc} */
   @Override
   public DMatrixRMaj getLoopJacobian()
   {
      return loopJacobianMatrix;
   }

   /** {@inheritDoc} */
   @Override
   public DMatrixRMaj getLoopConvectiveTerm()
   {
      return loopConvectiveTermMatrix;
   }

   /**
    * Returns the reference to the internal calculator used for this kinematic loop.
    * <p>
    * Mainly used for testing.
    * </p>
    * 
    * @return the reference to the four bar geometry calculator.
    */
   public FourBar getFourBar()
   {
      return fourBar;
   }

   /**
    * Returns the converters used to switch between four bar interior angles and joint angles.
    * 
    * @return the converters used to switch between four bar interior angles and joint angles.
    */
   public FourBarToJointConverter[] getConverters()
   {
      return converters;
   }
}
