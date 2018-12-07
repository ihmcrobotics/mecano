package us.ihmc.mecano.algorithms;

import static us.ihmc.robotics.Assert.*;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class SpatialAccelerationCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 1000;

   @Test
   public void testWithChainComposedOfPrismaticJoints() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<PrismaticJoint> prismaticJoints = MultiBodySystemRandomTools.nextPrismaticJointChain(random, numberOfJoints);
      RigidBodyBasics randomBody = prismaticJoints.get(random.nextInt(numberOfJoints)).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      boolean doAccelerationTerms = true;

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector3D rootLinearAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D rootAngularAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.set(rootAngularAcceleration, rootLinearAcceleration);
         SpatialAccelerationCalculator calculator = new SpatialAccelerationCalculator(randomBody, worldFrame, doVelocityTerms, doAccelerationTerms);
         calculator.setRootAcceleration(rootAcceleration);

         for (JointStateType stateSelection : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateSelection, -10.0, 10.0, prismaticJoints);
         rootBody.updateFramesRecursively();

         calculator.reset();

         FrameVector3D cumulatedLinearAcceleration = new FrameVector3D(rootLinearAcceleration);

         for (PrismaticJoint joint : prismaticJoints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            actualAcceleration.setIncludingFrame(calculator.getAccelerationOfBody(body));

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(bodyFrame, worldFrame, bodyFrame);

            FrameVector3DReadOnly jointAxis = joint.getJointAxis();
            cumulatedLinearAcceleration.changeFrame(jointAxis.getReferenceFrame());
            double qdd = joint.getQdd();
            cumulatedLinearAcceleration.scaleAdd(qdd, jointAxis, cumulatedLinearAcceleration);
            cumulatedLinearAcceleration.changeFrame(bodyFrame);
            expectedAcceleration.getLinearPart().set(cumulatedLinearAcceleration);

            // Need to compute the cross part due to the root angular acceleration
            FramePoint3D rootPosition = new FramePoint3D(rootBody.getBodyFixedFrame());
            rootPosition.changeFrame(bodyFrame);
            rootAngularAcceleration.changeFrame(bodyFrame);
            Vector3D crossPart = new Vector3D();
            crossPart.cross(new Vector3D(rootPosition), rootAngularAcceleration);
            expectedAcceleration.getLinearPart().add(crossPart);

            expectedAcceleration.getAngularPart().set(rootAngularAcceleration);

            assertSpatialAccelerationEquals(expectedAcceleration, actualAcceleration, 1.0e-12);
         }
      }
   }

   @Test
   public void testWithChainComposedOfRevoluteJointsAssertAngularAccelerationOnly() throws Exception
   {
      Random random = new Random(234234L);
      int numberOfJoints = 20;
      List<RevoluteJoint> revoluteJoints = MultiBodySystemRandomTools.nextRevoluteJointChain(random, numberOfJoints);
      RigidBodyBasics randomBody = revoluteJoints.get(random.nextInt(numberOfJoints)).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      boolean doAccelerationTerms = true;

      // No velocity
      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector3D rootLinearAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D rootAngularAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.set(rootAngularAcceleration, rootLinearAcceleration);
         SpatialAccelerationCalculator calculator = new SpatialAccelerationCalculator(randomBody, worldFrame, doVelocityTerms, doAccelerationTerms);
         calculator.setRootAcceleration(rootAcceleration);

         for (JointStateType stateSelection : new JointStateType[] {JointStateType.CONFIGURATION, JointStateType.ACCELERATION})
            MultiBodySystemRandomTools.nextState(random, stateSelection, -10.0, 10.0, revoluteJoints);
         rootBody.updateFramesRecursively();

         calculator.reset();

         FrameVector3D cumulatedAngularAcceleration = new FrameVector3D(rootAngularAcceleration);

         for (RevoluteJoint joint : revoluteJoints)
         {
            RigidBodyBasics body = joint.getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            actualAcceleration.setIncludingFrame(calculator.getAccelerationOfBody(body));

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(bodyFrame, worldFrame, bodyFrame);

            FrameVector3DReadOnly jointAxis = joint.getJointAxis();
            cumulatedAngularAcceleration.changeFrame(jointAxis.getReferenceFrame());
            double qdd = joint.getQdd();
            cumulatedAngularAcceleration.scaleAdd(qdd, jointAxis, cumulatedAngularAcceleration);
            cumulatedAngularAcceleration.changeFrame(bodyFrame);
            expectedAcceleration.getAngularPart().set(cumulatedAngularAcceleration);
            expectedAcceleration.checkReferenceFrameMatch(actualAcceleration);

            EuclidCoreTestTools.assertTuple3DEquals(expectedAcceleration.getAngularPart(), actualAcceleration.getAngularPart(), 5.0e-12);
         }
      }

      // Non-zero velocities
      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean doVelocityTerms = random.nextBoolean();

         FrameVector3D rootLinearAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D rootAngularAcceleration = new FrameVector3D(rootBody.getBodyFixedFrame(), EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.set(rootAngularAcceleration, rootLinearAcceleration);
         SpatialAccelerationCalculator calculator = new SpatialAccelerationCalculator(randomBody, worldFrame, doVelocityTerms, doAccelerationTerms);
         calculator.setRootAcceleration(rootAcceleration);

         for (JointStateType stateSelection : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateSelection, -10.0, 10.0, revoluteJoints);
         rootBody.updateFramesRecursively();

         calculator.reset();

         FrameVector3D cumulatedAngularAcceleration = new FrameVector3D(rootAngularAcceleration);

         for (int j = 0; j < revoluteJoints.size(); j++)
         {
            RevoluteJoint joint = revoluteJoints.get(j);
            RigidBodyBasics body = joint.getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            actualAcceleration.setIncludingFrame(calculator.getAccelerationOfBody(body));

            ReferenceFrame bodyFrame = body.getBodyFixedFrame();
            SpatialAcceleration expectedAcceleration = new SpatialAcceleration(bodyFrame, worldFrame, bodyFrame);

            FrameVector3D jointAxis = new FrameVector3D(joint.getJointAxis());
            cumulatedAngularAcceleration.changeFrame(jointAxis.getReferenceFrame());
            double qdd = joint.getQdd();
            cumulatedAngularAcceleration.scaleAdd(qdd, jointAxis, cumulatedAngularAcceleration);
            cumulatedAngularAcceleration.changeFrame(bodyFrame);

            if (doVelocityTerms)
            {
               // Need to account for the Coriolis acceleration
               Twist bodyTwist = new Twist();
               body.getBodyFixedFrame().getTwistOfFrame(bodyTwist);
               bodyTwist.changeFrame(bodyFrame);
               FrameVector3D coriolis = new FrameVector3D(bodyFrame);
               jointAxis.changeFrame(bodyFrame);
               coriolis.cross(bodyTwist.getAngularPart(), jointAxis);
               coriolis.scale(joint.getQd());
               cumulatedAngularAcceleration.add(coriolis);
            }

            expectedAcceleration.getAngularPart().set(cumulatedAngularAcceleration);
            expectedAcceleration.checkReferenceFrameMatch(bodyFrame, worldFrame, bodyFrame);
            actualAcceleration.checkReferenceFrameMatch(bodyFrame, worldFrame, bodyFrame);

            EuclidCoreTestTools.assertTuple3DEquals(expectedAcceleration.getAngularPart(), actualAcceleration.getAngularPart(), 1.0e-10);
         }
      }
   }

   @Test
   public void testWithChainRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(234234L);

      int numberOfJoints = 10;
      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
      List<OneDoFJointBasics> jointsInFuture = Arrays.asList(MultiBodySystemFactories.cloneOneDoFJointKinematicChain(joints.toArray(new OneDoFJoint[numberOfJoints])));

      RigidBodyBasics randomBody = joints.get(random.nextInt(joints.size())).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);

      double dt = 1.0e-8;

      for (int i = 0; i < ITERATIONS; i++)
      {
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.set(EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         SpatialAccelerationCalculator calculator = new SpatialAccelerationCalculator(randomBody, worldFrame, true, true);
         calculator.setRootAcceleration(rootAcceleration);

         for (JointStateType stateSelection : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, stateSelection, -1.0, 1.0, joints);

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            double q = joints.get(jointIndex).getQ();
            double qd = joints.get(jointIndex).getQd();
            double qdd = joints.get(jointIndex).getQdd();

            double qdFuture = qd + dt * qdd;
            double qFuture = q + dt * qd + 0.5 * dt * dt * qdd;
            jointsInFuture.get(jointIndex).setQ(qFuture);
            jointsInFuture.get(jointIndex).setQd(qdFuture);
         }

         joints.get(0).updateFramesRecursively();
         jointsInFuture.get(0).updateFramesRecursively();

         calculator.reset();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJoint joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            RigidBodyBasics bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();

            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            actualAcceleration.setIncludingFrame(calculator.getAccelerationOfBody(body));

            SpatialAcceleration expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, rootAcceleration);

            assertSpatialAccelerationEquals(expectedAcceleration, actualAcceleration, 1.0e-5);
         }
      }
   }

   @Test
   public void testWithFloatingJointRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 30;
      RandomFloatingRevoluteJointChain floatingChain = new RandomFloatingRevoluteJointChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<Joint> joints = floatingChain.getJoints();
      List<JointBasics> jointsInFuture = Arrays.asList(MultiBodySystemFactories.cloneKinematicChain(joints.toArray(new Joint[numberOfRevoluteJoints + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);

      RigidBodyBasics randomBody = joints.get(0).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);

      double dt = 1.0e-7;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

      for (int i = 0; i < ITERATIONS; i++)
      {
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.set(EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         SpatialAccelerationCalculator calculator = new SpatialAccelerationCalculator(randomBody, worldFrame, true, true);
         calculator.setRootAcceleration(rootAcceleration);

         for (JointStateType selection : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, selection, floatingJoint);

         for (JointStateType selection : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, selection, -1.0, 1.0, revoluteJoints);

         MultiBodySystemTools.copyJointsState(joints, jointsInFuture, JointStateType.CONFIGURATION);
         MultiBodySystemTools.copyJointsState(joints, jointsInFuture, JointStateType.VELOCITY);
         MultiBodySystemTools.copyJointsState(joints, jointsInFuture, JointStateType.ACCELERATION);

         integrator.doubleIntegrateFromAccelerationSubtree(floatingJointInFuture.getPredecessor());

         floatingJoint.updateFramesRecursively();
         floatingJointInFuture.updateFramesRecursively();

         calculator.reset();

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints + 1; jointIndex++)
         {
            Joint joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            RigidBodyBasics bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            actualAcceleration.setIncludingFrame(calculator.getAccelerationOfBody(body));

            SpatialAcceleration expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, rootAcceleration);

            assertSpatialAccelerationEquals(expectedAcceleration, actualAcceleration, 1.0e-4);

            Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
            FramePoint3D frameBodyFixedPoint = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
            FrameVector3D actualLinearAcceleration = new FrameVector3D(calculator.getLinearAccelerationOfBodyFixedPoint(body, frameBodyFixedPoint));
            FrameVector3D expectedLinearAcceleration = computeExpectedLinearAccelerationByFiniteDifference(dt, body, bodyInFuture, bodyFixedPoint,
                                                                                                           rootAcceleration);

            expectedLinearAcceleration.checkReferenceFrameMatch(actualLinearAcceleration);
            EuclidCoreTestTools.assertTuple3DEquals(expectedLinearAcceleration, actualLinearAcceleration, 1.1e-4);
         }
      }
   }

   @Test
   public void testRelativeAccelerationWithFloatingJointRobotAgainstFiniteDifference() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 50;
      RandomFloatingRevoluteJointChain floatingChain = new RandomFloatingRevoluteJointChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<Joint> joints = floatingChain.getJoints();
      List<JointBasics> jointsInFuture = Arrays.asList(MultiBodySystemFactories.cloneKinematicChain(joints.toArray(new Joint[numberOfRevoluteJoints + 1])));
      SixDoFJoint floatingJointInFuture = (SixDoFJoint) jointsInFuture.get(0);

      RigidBodyBasics randomBody = joints.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);

      double dt = 1.0e-8;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

      for (int i = 0; i < 50; i++)
      {
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.set(EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         SpatialAccelerationCalculator calculator = new SpatialAccelerationCalculator(randomBody, worldFrame, true, true);
         calculator.setRootAcceleration(rootAcceleration);

         for (JointStateType selection : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, selection, floatingJoint);

         for (JointStateType selection : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, selection, -1.0, 1.0, revoluteJoints);

         MultiBodySystemTools.copyJointsState(joints, jointsInFuture, JointStateType.CONFIGURATION);
         MultiBodySystemTools.copyJointsState(joints, jointsInFuture, JointStateType.VELOCITY);
         MultiBodySystemTools.copyJointsState(joints, jointsInFuture, JointStateType.ACCELERATION);

         integrator.doubleIntegrateFromAccelerationSubtree(floatingJointInFuture.getPredecessor());

         floatingJoint.updateFramesRecursively();
         floatingJointInFuture.updateFramesRecursively();

         calculator.reset();

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints + 1; jointIndex++)
         {
            Joint joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            RigidBodyBasics bodyInFuture = jointsInFuture.get(jointIndex).getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            actualAcceleration.setIncludingFrame(calculator.getAccelerationOfBody(body));

            SpatialAcceleration expectedAcceleration = computeExpectedAccelerationByFiniteDifference(dt, body, bodyInFuture, rootAcceleration);

            MecanoTestTools.assertSpatialAccelerationEquals("Iteration : " + i, expectedAcceleration, actualAcceleration, 1.0e-4);

            // Assert relative twist
            for (int baseJointIndex = 0; baseJointIndex < numberOfRevoluteJoints + 1; baseJointIndex++)
            {
               RigidBodyBasics base = joints.get(baseJointIndex).getSuccessor();
               RigidBodyBasics baseInFuture = jointsInFuture.get(baseJointIndex).getSuccessor();
               SpatialAcceleration actualRelativeAcceleration = new SpatialAcceleration(calculator.getRelativeAcceleration(base, body));

               SpatialAcceleration expectedRelativeAcceleration = computeExpectedRelativeAccelerationByFiniteDifference(dt, body, bodyInFuture, base,
                                                                                                                        baseInFuture, rootAcceleration);

               assertSpatialAccelerationEquals(expectedRelativeAcceleration, actualRelativeAcceleration, 1.0e-4);

               Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
               FramePoint3D frameBodyFixedPoint = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
               FrameVector3D actualLinearAcceleration = new FrameVector3D(calculator.getLinearAccelerationOfBodyFixedPoint(base, body, frameBodyFixedPoint));
               FrameVector3D expectedLinearAcceleration = computeExpectedLinearAccelerationByFiniteDifference(dt, body, bodyInFuture, base, baseInFuture,
                                                                                                              bodyFixedPoint);

               expectedLinearAcceleration.checkReferenceFrameMatch(actualLinearAcceleration);
               EuclidCoreTestTools.assertTuple3DEquals("iteration: " + i + ", joint index: " + baseJointIndex, expectedLinearAcceleration,
                                                       actualLinearAcceleration, 1.1e-4);
            }
         }
      }
   }

   @Test
   public void testWithDoVelocityTermsSetToFalse() throws Exception
   {
      Random random = new Random(435345L);

      int numberOfRevoluteJoints = 50;
      RandomFloatingRevoluteJointChain floatingChain = new RandomFloatingRevoluteJointChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<Joint> joints = floatingChain.getJoints();
      List<JointBasics> jointsNoVelocity = Arrays.asList(MultiBodySystemFactories.cloneKinematicChain(joints.toArray(new Joint[numberOfRevoluteJoints + 1]), ""));
      SixDoFJoint floatingJointNoVelocity = (SixDoFJoint) jointsNoVelocity.get(0);
      List<RevoluteJoint> revoluteJointsNoVelocity = MultiBodySystemTools.filterJoints(jointsNoVelocity, RevoluteJoint.class);

      RigidBodyBasics randomBody = joints.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBodyBasics randomBodyNoVelocity = jointsNoVelocity.get(random.nextInt(numberOfRevoluteJoints)).getPredecessor();
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(randomBody);
      RigidBodyBasics rootBodyNoVelocity = MultiBodySystemTools.getRootBody(randomBodyNoVelocity);

      for (int i = 0; i < 50; i++)
      {
         SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         rootAcceleration.set(EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         SpatialAccelerationCalculator calculator = new SpatialAccelerationCalculator(randomBody, worldFrame, false, true);
         calculator.setRootAcceleration(rootAcceleration);

         SpatialAcceleration rootAccelerationNoVelocity = new SpatialAcceleration(rootBodyNoVelocity.getBodyFixedFrame(), worldFrame,
                                                                                  rootBodyNoVelocity.getBodyFixedFrame());
         rootAccelerationNoVelocity.set((Vector3DReadOnly) rootAcceleration.getAngularPart(), (Vector3DReadOnly) rootAcceleration.getLinearPart());
         SpatialAccelerationCalculator calculatorNoVelocity = new SpatialAccelerationCalculator(randomBodyNoVelocity, worldFrame, true, true);
         calculatorNoVelocity.setRootAcceleration(rootAccelerationNoVelocity);

         Quaternion floatingJointRotation = EuclidCoreRandomTools.nextQuaternion(random);
         Point3D floatingJointPosition = EuclidCoreRandomTools.nextPoint3D(random, -10.0, 10.0);

         floatingJoint.setJointConfiguration(floatingJointRotation, floatingJointPosition);
         floatingJointNoVelocity.setJointConfiguration(floatingJointRotation, floatingJointPosition);

         SpatialAcceleration floatJointAcceleration = new SpatialAcceleration(floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                                              floatingJoint.getFrameAfterJoint());
         floatJointAcceleration.set(EuclidCoreRandomTools.nextVector3D(random), EuclidCoreRandomTools.nextVector3D(random));
         floatingJoint.setJointAcceleration(floatJointAcceleration);
         SpatialAcceleration floatJointAccelerationNoVelocity = new SpatialAcceleration(floatingJointNoVelocity.getFrameAfterJoint(),
                                                                                        floatingJointNoVelocity.getFrameBeforeJoint(),
                                                                                        floatingJointNoVelocity.getFrameAfterJoint());
         floatJointAccelerationNoVelocity.set((Vector3DReadOnly) floatJointAcceleration.getAngularPart(),
                                              (Vector3DReadOnly) floatJointAcceleration.getLinearPart());
         floatingJointNoVelocity.setJointAcceleration(floatJointAccelerationNoVelocity);

         Twist floatingJointTwist = MecanoRandomTools.nextTwist(random, floatingJoint.getFrameAfterJoint(), floatingJoint.getFrameBeforeJoint(),
                                                                           floatingJoint.getFrameAfterJoint());
         floatingJoint.setJointTwist(floatingJointTwist);

         for (JointStateType selection : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, selection, -1.0, 1.0, revoluteJoints);

         for (int jointIndex = 0; jointIndex < revoluteJointsNoVelocity.size(); jointIndex++)
         {
            RevoluteJoint joint = revoluteJoints.get(jointIndex);
            RevoluteJoint jointNoVelocity = revoluteJointsNoVelocity.get(jointIndex);
            jointNoVelocity.setQ(joint.getQ());
            jointNoVelocity.setQd(0.0);
            jointNoVelocity.setQdd(joint.getQdd());
         }

         floatingJoint.updateFramesRecursively();
         floatingJointNoVelocity.updateFramesRecursively();

         calculator.reset();
         calculatorNoVelocity.reset();

         for (int jointIndex = 0; jointIndex < numberOfRevoluteJoints + 1; jointIndex++)
         {
            Joint joint = joints.get(jointIndex);
            RigidBodyBasics body = joint.getSuccessor();
            RigidBodyBasics bodyNoVelocity = jointsNoVelocity.get(jointIndex).getSuccessor();
            SpatialAcceleration actualAcceleration = new SpatialAcceleration();
            actualAcceleration.setIncludingFrame(calculator.getAccelerationOfBody(body));

            SpatialAcceleration expectedAcceleration = new SpatialAcceleration();
            expectedAcceleration.setIncludingFrame(calculatorNoVelocity.getAccelerationOfBody(bodyNoVelocity));

            assertSpatialAccelerationEquals(expectedAcceleration, actualAcceleration, 1.0e-12, true);

            Point3D bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
            FramePoint3D frameBodyFixedPoint = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
            FrameVector3D actualLinearAcceleration = new FrameVector3D(calculator.getLinearAccelerationOfBodyFixedPoint(body, frameBodyFixedPoint));
            FrameVector3D expectedLinearAcceleration = new FrameVector3D(calculatorNoVelocity.getLinearAccelerationOfBodyFixedPoint(bodyNoVelocity, frameBodyFixedPoint));

            assertEquals(expectedLinearAcceleration.getReferenceFrame().getName(), actualLinearAcceleration.getReferenceFrame().getName());
            EuclidCoreTestTools.assertTuple3DEquals(expectedLinearAcceleration, actualLinearAcceleration, 1.0e-12);

            // Assert relative twist
            for (int baseJointIndex = 0; baseJointIndex < numberOfRevoluteJoints + 1; baseJointIndex++)
            {
               RigidBodyBasics base = joints.get(baseJointIndex).getSuccessor();
               RigidBodyBasics baseNoVelocity = jointsNoVelocity.get(baseJointIndex).getSuccessor();
               SpatialAcceleration actualRelativeAcceleration = new SpatialAcceleration(calculator.getRelativeAcceleration(base, body));

               SpatialAcceleration expectedRelativeAcceleration = new SpatialAcceleration(calculatorNoVelocity.getRelativeAcceleration(baseNoVelocity, bodyNoVelocity));

               String messagePrefix = "iteration: " + i + ", joint index: " + jointIndex + ", base joint index: " + baseJointIndex;
               assertSpatialAccelerationEquals(messagePrefix, expectedRelativeAcceleration, actualRelativeAcceleration, 1.0e-12, true);

               bodyFixedPoint = EuclidCoreRandomTools.nextPoint3D(random, 1.0);
               frameBodyFixedPoint = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
               actualLinearAcceleration.setIncludingFrame(calculator.getLinearAccelerationOfBodyFixedPoint(base, body, frameBodyFixedPoint));
               expectedLinearAcceleration.setIncludingFrame(calculatorNoVelocity.getLinearAccelerationOfBodyFixedPoint(baseNoVelocity, bodyNoVelocity, frameBodyFixedPoint));

               assertEquals(expectedLinearAcceleration.getReferenceFrame().getName(), actualLinearAcceleration.getReferenceFrame().getName());
               EuclidCoreTestTools.assertTuple3DEquals(messagePrefix, expectedLinearAcceleration, actualLinearAcceleration, 1.0e-12);
            }
         }
      }
   }

   public static FrameVector3D computeExpectedLinearAccelerationByFiniteDifference(double dt, RigidBodyReadOnly body, RigidBodyReadOnly bodyInFuture,
                                                                                   Point3DReadOnly bodyFixedPoint, SpatialAccelerationReadOnly rootAcceleration)
   {
      FrameVector3D pointLinearVelocity = new FrameVector3D();
      FrameVector3D pointLinearVelocityInFuture = new FrameVector3D();

      FramePoint3D point = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
      FramePoint3D pointInFuture = new FramePoint3D(bodyInFuture.getBodyFixedFrame(), bodyFixedPoint);

      body.getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(point, pointLinearVelocity);
      bodyInFuture.getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(pointInFuture, pointLinearVelocityInFuture);

      pointLinearVelocity.changeFrame(worldFrame);
      pointLinearVelocityInFuture.changeFrame(worldFrame);

      FrameVector3D pointLinearAcceleration = new FrameVector3D(worldFrame);
      pointLinearAcceleration.sub(pointLinearVelocityInFuture, pointLinearVelocity);
      pointLinearAcceleration.scale(1.0 / dt);

      // Need to account for the root acceleration
      rootAcceleration.getBodyFrame().checkReferenceFrameMatch(rootAcceleration.getReferenceFrame());
      FrameVector3D rootAngularAcceleration = new FrameVector3D(rootAcceleration.getAngularPart());
      FrameVector3D rootLinearAcceleration = new FrameVector3D(rootAcceleration.getLinearPart());

      FramePoint3D bodyFixedPointToRoot = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
      bodyFixedPointToRoot.changeFrame(rootAcceleration.getBodyFrame());
      FrameVector3D crossPart = new FrameVector3D(rootAcceleration.getBodyFrame());
      crossPart.cross(bodyFixedPointToRoot, rootAngularAcceleration);
      crossPart.changeFrame(worldFrame);
      rootLinearAcceleration.changeFrame(worldFrame);
      pointLinearAcceleration.sub(crossPart);
      pointLinearAcceleration.add(rootLinearAcceleration);

      return pointLinearAcceleration;

   }

   public static FrameVector3D computeExpectedLinearAccelerationByFiniteDifference(double dt, RigidBodyReadOnly body, RigidBodyReadOnly bodyInFuture,
                                                                                   RigidBodyReadOnly base, RigidBodyBasics baseInFuture,
                                                                                   Point3DReadOnly bodyFixedPoint)
   {
      FrameVector3D pointLinearVelocity = new FrameVector3D();
      FrameVector3D pointLinearVelocityInFuture = new FrameVector3D();

      FramePoint3D point = new FramePoint3D(body.getBodyFixedFrame(), bodyFixedPoint);
      FramePoint3D pointInFuture = new FramePoint3D(bodyInFuture.getBodyFixedFrame(), bodyFixedPoint);

      Twist twist = new Twist();
      Twist twistInFuture = new Twist();

      body.getBodyFixedFrame().getTwistRelativeToOther(base.getBodyFixedFrame(), twist);
      bodyInFuture.getBodyFixedFrame().getTwistRelativeToOther(baseInFuture.getBodyFixedFrame(), twistInFuture);
      twist.getLinearVelocityAt(point, pointLinearVelocity);
      twistInFuture.getLinearVelocityAt(pointInFuture, pointLinearVelocityInFuture);

      pointLinearVelocity.changeFrame(base.getBodyFixedFrame());
      pointLinearVelocityInFuture.changeFrame(baseInFuture.getBodyFixedFrame());

      FrameVector3D pointLinearAcceleration = new FrameVector3D(base.getBodyFixedFrame());
      pointLinearAcceleration.sub((Vector3DReadOnly) pointLinearVelocityInFuture, pointLinearVelocity);
      pointLinearAcceleration.scale(1.0 / dt);

      return pointLinearAcceleration;

   }

   private SpatialAcceleration computeExpectedRelativeAccelerationByFiniteDifference(double dt, RigidBodyReadOnly body, RigidBodyReadOnly bodyInFuture,
                                                                                     RigidBodyReadOnly base, RigidBodyReadOnly baseInFuture,
                                                                                     SpatialAccelerationReadOnly rootAcceleration)
   {
      Twist relativeTwist = new Twist();
      Twist relativeTwistInFuture = new Twist();

      body.getBodyFixedFrame().getTwistRelativeToOther(base.getBodyFixedFrame(), relativeTwist);
      bodyInFuture.getBodyFixedFrame().getTwistRelativeToOther(baseInFuture.getBodyFixedFrame(), relativeTwistInFuture);

      Vector3D angularAcceleration = firstOrderFiniteDifference(dt, relativeTwist.getAngularPart(), relativeTwistInFuture.getAngularPart());
      Vector3D linearAcceleration = firstOrderFiniteDifference(dt, relativeTwist.getLinearPart(), relativeTwistInFuture.getLinearPart());

      return new SpatialAcceleration(body.getBodyFixedFrame(), base.getBodyFixedFrame(), body.getBodyFixedFrame(), angularAcceleration, linearAcceleration);
   }

   private static SpatialAcceleration computeExpectedAccelerationByFiniteDifference(double dt, RigidBodyReadOnly body, RigidBodyReadOnly bodyInFuture,
                                                                                    SpatialAccelerationReadOnly rootAcceleration)
   {
      SpatialAcceleration expectedAcceleration = new SpatialAcceleration(body.getBodyFixedFrame(), worldFrame, body.getBodyFixedFrame());

      Twist bodyTwist = new Twist();
      Twist bodyTwistInFuture = new Twist();

      body.getBodyFixedFrame().getTwistOfFrame(bodyTwist);
      bodyInFuture.getBodyFixedFrame().getTwistOfFrame(bodyTwistInFuture);

      Vector3D angularAcceleration = firstOrderFiniteDifference(dt, bodyTwist.getAngularPart(), bodyTwistInFuture.getAngularPart());
      Vector3D linearAcceleration = firstOrderFiniteDifference(dt, bodyTwist.getLinearPart(), bodyTwistInFuture.getLinearPart());
      expectedAcceleration.set(angularAcceleration, linearAcceleration);

      // Need to account for the root acceleration
      FrameVector3D rootAngularAcceleration = new FrameVector3D(rootAcceleration.getReferenceFrame(), rootAcceleration.getAngularPart());
      FrameVector3D rootLinearAcceleration = new FrameVector3D(rootAcceleration.getReferenceFrame(), rootAcceleration.getLinearPart());
      rootAngularAcceleration.changeFrame(expectedAcceleration.getReferenceFrame());
      rootLinearAcceleration.changeFrame(expectedAcceleration.getReferenceFrame());
      expectedAcceleration.getAngularPart().add(rootAngularAcceleration);

      FramePoint3D rootToBody = new FramePoint3D(rootAcceleration.getReferenceFrame());
      rootToBody.changeFrame(expectedAcceleration.getReferenceFrame());
      Vector3D crossPart = new Vector3D();
      crossPart.cross(rootToBody, rootAngularAcceleration);
      expectedAcceleration.getLinearPart().add(crossPart);
      expectedAcceleration.getLinearPart().add(rootLinearAcceleration);

      return expectedAcceleration;
   }

   public static Vector3D firstOrderFiniteDifference(double dt, Vector3DReadOnly now, Vector3DReadOnly next)
   {
      Vector3D finiteDifference = new Vector3D();
      finiteDifference.sub(next, now);
      finiteDifference.scale(1.0 / dt);
      return finiteDifference;
   }

   public static void assertSpatialAccelerationEquals(SpatialAccelerationReadOnly expected, SpatialAccelerationReadOnly actual, double epsilon)
         throws AssertionError
   {
      assertSpatialAccelerationEquals(null, expected, actual, epsilon, false);
   }

   public static void assertSpatialAccelerationEquals(SpatialAccelerationReadOnly expected, SpatialAccelerationReadOnly actual, double epsilon,
                                                      boolean checkFrameByName)
   {
      assertSpatialAccelerationEquals(null, expected, actual, epsilon, checkFrameByName);
   }

   public static void assertSpatialAccelerationEquals(String messagePrefix, SpatialAccelerationReadOnly expected, SpatialAccelerationReadOnly actual,
                                                      double epsilon, boolean checkFrameByName)
         throws AssertionError
   {
      try
      {
         if (checkFrameByName)
         {
            assertEquals(expected.getBodyFrame().getName(), actual.getBodyFrame().getName());
            assertEquals(expected.getBaseFrame().getName(), actual.getBaseFrame().getName());
            assertEquals(expected.getReferenceFrame().getName(), actual.getReferenceFrame().getName());
            EuclidCoreTestTools.assertTuple3DEquals(expected.getAngularPart(), actual.getAngularPart(), epsilon);
            EuclidCoreTestTools.assertTuple3DEquals(expected.getLinearPart(), actual.getLinearPart(), epsilon);
         }
         else
         {
            assertTrue(expected.epsilonEquals(actual, epsilon));
         }
      }
      catch (AssertionError e)
      {
         Vector3D difference = new Vector3D();
         difference.sub(expected.getLinearPart(), actual.getLinearPart());
         double linearPartDifference = difference.length();
         difference.sub(expected.getAngularPart(), actual.getAngularPart());
         double angularPartDifference = difference.length();
         messagePrefix = messagePrefix != null ? messagePrefix + " " : "";
         throw new AssertionError(messagePrefix + "expected:\n<" + expected + ">\n but was:\n<" + actual + ">\n difference: linear part: "
               + linearPartDifference + ", angular part: " + angularPartDifference);
      }
   }
}
