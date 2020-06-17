package us.ihmc.mecano.algorithms;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

/**
 * Computes the center of mass position of a multi-body system.
 *
 * @author Sylvain Bertrand
 */
public class CenterOfMassCalculator implements ReferenceFrameHolder
{
   /** The frame in which center of mass position is to be expressed. */
   private final ReferenceFrame referenceFrame;
   /** Defines the multi-body system to use with this calculator. */
   private final MultiBodySystemReadOnly input;
   /** The array of considered rigid-bodies to iterate through. */
   private final RigidBodyReadOnly[] rigidBodies;
   /** The total mass of the system. */
   private double totalMass;
   /** The center of mass position. */
   private final FramePoint3D centerOfMass = new FramePoint3D();
   /** Intermediate variable for garbage free operations. */
   private final FramePoint3D tempPoint = new FramePoint3D();
   /**
    * Whether the center of mass position has been calculated since the last call to {@link #reset()}.
    */
   private boolean isCenterOfMassUpToDate = false;

   /**
    * Creates a new calculator for the subtree that starts off the given {@code rootBody}.
    *
    * @param rootBody       the start of subtree for which the center of mass is to be computed. Not
    *                       modified.
    * @param referenceFrame the frame in which the center of mass is to be expressed.
    */
   public CenterOfMassCalculator(RigidBodyReadOnly rootBody, ReferenceFrame referenceFrame)
   {
      this(MultiBodySystemReadOnly.toMultiBodySystemInput(rootBody), referenceFrame);
   }

   /**
    * Creates a new calculator for the given {@code input}.
    *
    * @param input          the definition of the system to be evaluated by this calculator.
    * @param referenceFrame the frame in which the center of mass is to be expressed.
    */
   public CenterOfMassCalculator(MultiBodySystemReadOnly input, ReferenceFrame referenceFrame)
   {
      this.input = input;
      // Preventing repetitions dues to possible kinematic loop(s) by adding 'distinct()'
      rigidBodies = input.getJointsToConsider().stream().map(JointReadOnly::getSuccessor).distinct().toArray(RigidBodyReadOnly[]::new);
      this.referenceFrame = referenceFrame;
   }

   /**
    * Invalidates the internal memory.
    */
   public void reset()
   {
      isCenterOfMassUpToDate = false;
   }

   private void updateCenterOfMass()
   {
      if (isCenterOfMassUpToDate)
         return;

      centerOfMass.setToZero(referenceFrame);
      totalMass = 0.0;

      for (RigidBodyReadOnly rigidBody : rigidBodies)
      {
         SpatialInertiaReadOnly inertia = rigidBody.getInertia();

         tempPoint.setIncludingFrame(inertia.getCenterOfMassOffset());
         double mass = inertia.getMass();
         tempPoint.changeFrame(referenceFrame);
         tempPoint.scale(mass);
         centerOfMass.add(tempPoint);
         totalMass += mass;
      }

      centerOfMass.scale(1.0 / totalMass);
      isCenterOfMassUpToDate = true;
   }

   /**
    * Gets the definition of the multi-body system that was used to create this calculator.
    *
    * @return this calculator input.
    */
   public MultiBodySystemReadOnly getInput()
   {
      return input;
   }

   /**
    * Gets the total mass of the multi-body system.
    *
    * @return the mass of the multi-body system.
    */
   public double getTotalMass()
   {
      updateCenterOfMass();
      return totalMass;
   }

   /**
    * Gets the center of mass position of the multi-body system.
    *
    * @return the center of mass position.
    */
   public FramePoint3DReadOnly getCenterOfMass()
   {
      updateCenterOfMass();
      return centerOfMass;
   }

   /**
    * The reference frame in which the center of mass position is computed.
    *
    * @return this calculator's reference frame.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
