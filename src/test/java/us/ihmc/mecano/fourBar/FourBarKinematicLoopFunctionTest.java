package us.ihmc.mecano.fourBar;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.fourBar.CrossFourBarTest.Viewer;
import us.ihmc.mecano.fourBar.FourBarKinematicLoopFunctionTools.FourBarToJointConverter;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class FourBarKinematicLoopFunctionTest
{
   private static final int ITERATIONS = 1000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double EPSILON = 1.0e-10;
   private static final String[] names = {"A", "B", "C", "D"};

   @Test
   public void testConfigurationConvexFourBar() throws InterruptedException
   {
      Random random = new Random(78934678);

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean clockwise = random.nextBoolean();
         int actuatedJointIndex = random.nextInt(4);
         boolean flipAxesRandomly = true;
         boolean crossFourBar = false;
         FourBarKinematicLoopFunction fourBarKinematicLoop = nextFourBar(random, clockwise, actuatedJointIndex, flipAxesRandomly, crossFourBar);
         RevoluteJointBasics actuatedJoint = fourBarKinematicLoop.getActuatedJoint();
         FourBarVertices2D verticesZeroConfig = computeFourBarVertices2D(fourBarKinematicLoop);

         actuatedJoint.setQ(0.01);
         MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, actuatedJoint);
         fourBarKinematicLoop.updateState(false, false);
         MultiBodySystemTools.getRootBody(actuatedJoint.getPredecessor()).updateFramesRecursively();

         FourBarVertices2D vertices = computeFourBarVertices2D(fourBarKinematicLoop);

         try
         {
            for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++)
            {
               FourBarAngle fourBarAngle = FourBarAngle.values[vertexIndex];
               FramePoint2D prev = vertices.getPrevious(vertexIndex);
               FramePoint2D curr = vertices.get(vertexIndex);
               FramePoint2D next = vertices.getNext(vertexIndex);

               double expectedAngle = FourBarKinematicLoopFunctionTools.angleABC(prev, curr, next);

               double actualAngle = fourBarKinematicLoop.getFourBar().getVertex(fourBarAngle).getAngle();
               assertEquals(expectedAngle,
                            actualAngle,
                            EPSILON,
                            String.format("Iteration: %d. Interior angle at %s. Joint angle: %f. Error: %g",
                                          i,
                                          names[vertexIndex],
                                          actuatedJoint.getQ(),
                                          Math.abs(expectedAngle - actualAngle)));

               // Check that lengths are preserved
               double expectedLength = fourBarKinematicLoop.getFourBar().getVertex(fourBarAngle).getNextEdge().getLength();
               double actualLength = curr.distance(next);
               assertEquals(expectedLength,
                            actualLength,
                            EPSILON,
                            String.format("Iteration: %d. Side %s%s. Joint angle: %f. Error: %g.",
                                          i,
                                          names[vertexIndex],
                                          names[(vertexIndex + 1) % 4],
                                          actuatedJoint.getQ(),
                                          Math.abs(expectedLength - actualLength)));
               assertFourBarIsClosed(i, fourBarKinematicLoop);
            }
         }
         catch (Throwable e)
         { // Show the error message in the console as well.
            LogTools.info("Actuated joint: " + names[actuatedJointIndex]);
            LogTools.error("\n\n" + e.getMessage() + "\n\n");
            Viewer viewer = CrossFourBarTest.startupViewer();
            List<FramePoint2D> allVertices = new ArrayList<FramePoint2D>();
            allVertices.addAll(verticesZeroConfig.vertexList());
            allVertices.addAll(vertices.vertexList());
            viewer.updateFOV(allVertices);
            CrossFourBarTest.draw(viewer, verticesZeroConfig.vertexList());
            CrossFourBarTest.draw(viewer, vertices.vertexList(), "'");
            viewer.waitUntilClosed();
            throw e;
         }
      }
   }

   @Test
   public void testConfigurationCrossFourBar() throws InterruptedException
   {
      Random random = new Random(7834678);

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean clockwise = random.nextBoolean();
         int actuatedJointIndex = random.nextInt(4);
         boolean flipAxesRandomly = true;
         boolean crossFourBar = true;
         FourBarKinematicLoopFunction fourBarKinematicLoop = nextFourBar(random, clockwise, actuatedJointIndex, flipAxesRandomly, crossFourBar);
         FourBar fourBar = fourBarKinematicLoop.getFourBar();
         RevoluteJointBasics actuatedJoint = fourBarKinematicLoop.getActuatedJoint();
         FourBarVertices2D verticesZeroConfig = computeFourBarVertices2D(fourBarKinematicLoop);

         actuatedJoint.setQ(0.01);
         MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, actuatedJoint);
         fourBarKinematicLoop.updateState(false, false);
         MultiBodySystemTools.getRootBody(actuatedJoint.getPredecessor()).updateFramesRecursively();

         FourBarVertices2D vertices = computeFourBarVertices2D(fourBarKinematicLoop);

         boolean reverse = false;

         if (FourBarKinematicLoopFunctionTools.angleABC(vertices.D, vertices.A, vertices.B) < 0.0 && fourBar.getVertex(FourBarAngle.DAB).isConvex())
            reverse = true;

         try
         {
            for (int vertexIndex = 0; vertexIndex < 4; vertexIndex++)
            {
               FourBarAngle fourBarAngle = FourBarAngle.values[vertexIndex];
               FramePoint2D prev = vertices.getPrevious(vertexIndex);
               FramePoint2D curr = vertices.get(vertexIndex);
               FramePoint2D next = vertices.getNext(vertexIndex);
               FourBarVertex fourBarVertex = fourBar.getVertex(fourBarAngle);

               double expectedAngle = FourBarKinematicLoopFunctionTools.angleABC(prev, curr, next);

               if (reverse)
                  expectedAngle = -expectedAngle;

               double actualAngle = fourBarVertex.getAngle();
               assertEquals(expectedAngle,
                            actualAngle,
                            EPSILON,
                            String.format("Iteration: %d. Interior angle at %s. Joint angle: %f. Error: %g",
                                          i,
                                          names[vertexIndex],
                                          actuatedJoint.getQ(),
                                          Math.abs(expectedAngle - actualAngle)));

               // Check that lengths are preserved
               double expectedLength = fourBarVertex.getNextEdge().getLength();
               double actualLength = curr.distance(next);
               assertEquals(expectedLength,
                            actualLength,
                            EPSILON,
                            String.format("Iteration: %d. Side: %s%s. Joint angle: %f. Error: %g.",
                                          i,
                                          names[vertexIndex],
                                          names[(vertexIndex + 1) % 4],
                                          actuatedJoint.getQ(),
                                          Math.abs(expectedLength - actualLength)));
               assertFourBarIsClosed(i, fourBarKinematicLoop);
            }
         }
         catch (Throwable e)
         { // Show the error message in the console as well.
            LogTools.info("\n-------------------\nActuated joint: " + names[actuatedJointIndex]);
            LogTools.error("\n----------------------------------");
            LogTools.error(e.getMessage());
            LogTools.error("-------------------------------------\n");
            Viewer viewer = CrossFourBarTest.startupViewer();
            List<FramePoint2D> allVertices = new ArrayList<FramePoint2D>();
            allVertices.addAll(verticesZeroConfig.vertexList());
            allVertices.addAll(vertices.vertexList());
            viewer.updateFOV(allVertices);
            CrossFourBarTest.draw(viewer, verticesZeroConfig.vertexList());
            CrossFourBarTest.draw(viewer, vertices.vertexList(), "'");
            viewer.waitUntilClosed();
            throw e;
         }
      }
   }

   @Test
   public void testLimitConfigurationCrossFourBar()
   {
      Random random = new Random(7834678);

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean clockwise = random.nextBoolean();
         int actuatedJointIndex = random.nextInt(4);
         boolean flipAxesRandomly = true;
         boolean crossFourBar = true;
         FourBarKinematicLoopFunction fourBarKinematicLoop = nextFourBar(random, clockwise, actuatedJointIndex, flipAxesRandomly, crossFourBar);
         FourBar fourBar = fourBarKinematicLoop.getFourBar();

         List<RevoluteJointBasics> joints = fourBarKinematicLoop.getLoopJoints();
         FourBarToJointConverter[] converters = fourBarKinematicLoop.getConverters();
         FourBarVertex[] vertices = {fourBar.getVertexA(), fourBar.getVertexB(), fourBar.getVertexC(), fourBar.getVertexD()};

         for (int j = 0; j < 4; j++)
         {
            RevoluteJointBasics joint = joints.get(j);
            FourBarToJointConverter converter = converters[j];
            FourBarVertex vertex = vertices[j];
            Supplier<String> messageSupplier = () -> "Failed for :" + joint.getName();

            if (converter.getSign() > 0.0)
            {
               assertEquals(joint.getJointLimitLower(), converter.toJointAngle(vertex.getMinAngle()), messageSupplier);
               assertEquals(joint.getJointLimitUpper(), converter.toJointAngle(vertex.getMaxAngle()), messageSupplier);
            }
            else
            {
               assertEquals(joint.getJointLimitLower(), converter.toJointAngle(vertex.getMaxAngle()), messageSupplier);
               assertEquals(joint.getJointLimitUpper(), converter.toJointAngle(vertex.getMinAngle()), messageSupplier);
            }
         }

         RevoluteJointBasics joint = fourBarKinematicLoop.getActuatedJoint();
         FourBarToJointConverter converter = converters[fourBarKinematicLoop.getActuatedJointIndex()];
         FourBarVertex vertex = fourBarKinematicLoop.getActuatedVertex();
         Supplier<String> messageSupplier = () -> "Failed for :" + joint.getName();

         if (converter.getSign() > 0.0)
         {
            assertEquals(joint.getJointLimitLower(), converter.toJointAngle(vertex.getMinAngle()), messageSupplier);
            assertEquals(joint.getJointLimitUpper(), converter.toJointAngle(vertex.getMaxAngle()), messageSupplier);
         }
         else
         {
            assertEquals(joint.getJointLimitLower(), converter.toJointAngle(vertex.getMaxAngle()), messageSupplier);
            assertEquals(joint.getJointLimitUpper(), converter.toJointAngle(vertex.getMinAngle()), messageSupplier);
         }
      }

      { // Test with four bar example
         FourBarKinematicLoopFunction fourBarKinematicLoop = createFourBarExample1(random, 0, false);
         FourBar fourBar = fourBarKinematicLoop.getFourBar();

         List<RevoluteJointBasics> joints = fourBarKinematicLoop.getLoopJoints();
         FourBarToJointConverter[] converters = fourBarKinematicLoop.getConverters();
         FourBarVertex[] vertices = {fourBar.getVertexA(), fourBar.getVertexB(), fourBar.getVertexC(), fourBar.getVertexD()};

         for (int j = 0; j < 4; j++)
         {
            RevoluteJointBasics joint = joints.get(j);
            FourBarToJointConverter converter = converters[j];
            FourBarVertex vertex = vertices[j];
            Supplier<String> messageSupplier = () -> "Failed for :" + joint.getName();

            if (converter.getSign() > 0.0)
            {
               assertEquals(joint.getJointLimitLower(), converter.toJointAngle(vertex.getMinAngle()), messageSupplier);
               assertEquals(joint.getJointLimitUpper(), converter.toJointAngle(vertex.getMaxAngle()), messageSupplier);
            }
            else
            {
               assertEquals(joint.getJointLimitLower(), converter.toJointAngle(vertex.getMaxAngle()), messageSupplier);
               assertEquals(joint.getJointLimitUpper(), converter.toJointAngle(vertex.getMinAngle()), messageSupplier);
            }
         }

         RevoluteJointBasics joint = fourBarKinematicLoop.getActuatedJoint();
         FourBarToJointConverter converter = converters[fourBarKinematicLoop.getActuatedJointIndex()];
         FourBarVertex vertex = fourBarKinematicLoop.getActuatedVertex();
         Supplier<String> messageSupplier = () -> "Failed for :" + joint.getName();

         if (converter.getSign() > 0.0)
         {
            assertEquals(joint.getJointLimitLower(), converter.toJointAngle(vertex.getMinAngle()), messageSupplier);
            assertEquals(joint.getJointLimitUpper(), converter.toJointAngle(vertex.getMaxAngle()), messageSupplier);
         }
         else
         {
            assertEquals(joint.getJointLimitLower(), converter.toJointAngle(vertex.getMaxAngle()), messageSupplier);
            assertEquals(joint.getJointLimitUpper(), converter.toJointAngle(vertex.getMinAngle()), messageSupplier);
         }
      }
   }

   private static void assertFourBarIsClosed(int iteration, FourBarKinematicLoopFunction fourBarKinematicLoop)
   {
      RevoluteJointBasics closingJoint = null;

      if (fourBarKinematicLoop.getJointA().isLoopClosure())
         closingJoint = fourBarKinematicLoop.getJointA();
      else if (fourBarKinematicLoop.getJointB().isLoopClosure())
         closingJoint = fourBarKinematicLoop.getJointB();
      else if (fourBarKinematicLoop.getJointC().isLoopClosure())
         closingJoint = fourBarKinematicLoop.getJointC();
      else if (fourBarKinematicLoop.getJointD().isLoopClosure())
         closingJoint = fourBarKinematicLoop.getJointD();

      assertNotNull(closingJoint);

      FramePoint3D error = new FramePoint3D(closingJoint.getLoopClosureFrame());
      error.changeFrame(closingJoint.getSuccessor().getParentJoint().getFrameAfterJoint());
      assertEquals(0.0, error.distanceFromOrigin(), EPSILON, "Iteration: " + iteration);
   }

   private static FourBarVertices2D computeFourBarVertices2D(FourBarKinematicLoopFunction fourBarKinematicLoop)
   {
      RevoluteJointBasics jointA = fourBarKinematicLoop.getJointA();
      RevoluteJointBasics jointB = fourBarKinematicLoop.getJointB();
      RevoluteJointBasics jointC = fourBarKinematicLoop.getJointC();
      RevoluteJointBasics jointD = fourBarKinematicLoop.getJointD();
      RevoluteJointBasics actuatedJoint = fourBarKinematicLoop.getActuatedJoint();
      MultiBodySystemTools.getRootBody(actuatedJoint.getPredecessor()).updateFramesRecursively();

      ReferenceFrame fourBarLocalFrame = FourBarKinematicLoopFunctionTools.constructReferenceFrameFromPointAndAxis("LocalFrame",
                                                                                                                   new FramePoint3D(actuatedJoint.getFrameBeforeJoint()),
                                                                                                                   Axis3D.Z,
                                                                                                                   actuatedJoint.getJointAxis());
      FourBarVertices2D vertices = new FourBarVertices2D();
      vertices.setToZero(fourBarLocalFrame);
      vertices.setFromReferenceFrame(jointA.getFrameAfterJoint(), jointB.getFrameAfterJoint(), jointC.getFrameAfterJoint(), jointD.getFrameAfterJoint());
      return vertices;
   }

   private static class FourBarVertices2D
   {
      private final FramePoint2D A = new FramePoint2D();
      private final FramePoint2D B = new FramePoint2D();
      private final FramePoint2D C = new FramePoint2D();
      private final FramePoint2D D = new FramePoint2D();
      private final List<FramePoint2D> vertexList = Arrays.asList(A, B, C, D);

      FramePoint2D get(int index)
      {
         return vertexList.get(index);
      }

      FramePoint2D getPrevious(int index)
      {
         return vertexList.get((index + 3) % 4);
      }

      FramePoint2D getNext(int index)
      {
         return vertexList.get((index + 1) % 4);
      }

      void setToZero(ReferenceFrame referenceFrame)
      {
         A.setToZero(referenceFrame);
         B.setToZero(referenceFrame);
         C.setToZero(referenceFrame);
         D.setToZero(referenceFrame);
      }

      void setFromReferenceFrame(ReferenceFrame frameA, ReferenceFrame frameB, ReferenceFrame frameC, ReferenceFrame frameD)
      {
         FramePoint3D A3D = new FramePoint3D(frameA);
         FramePoint3D B3D = new FramePoint3D(frameB);
         FramePoint3D C3D = new FramePoint3D(frameC);
         FramePoint3D D3D = new FramePoint3D(frameD);

         A3D.changeFrame(A.getReferenceFrame());
         B3D.changeFrame(D.getReferenceFrame());
         C3D.changeFrame(C.getReferenceFrame());
         D3D.changeFrame(D.getReferenceFrame());

         A.set(A3D);
         B.set(B3D);
         C.set(C3D);
         D.set(D3D);
      }

      List<FramePoint2D> vertexList()
      {
         return vertexList;
      }
   }

   public FourBarKinematicLoopFunction nextFourBar(Random random, boolean clockwise, int actuatedJointIndex, boolean flipAxesRandomly, boolean crossFourBar)
   {
      List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
      if (crossFourBar)
      {
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
      }
      if (!clockwise)
         Collections.reverse(vertices);
      Point2D A = vertices.get(0);
      Point2D B = vertices.get(1);
      Point2D C = vertices.get(2);
      Point2D D = vertices.get(3);

      return nextFourBar(random, A, B, C, D, actuatedJointIndex, flipAxesRandomly);
   }

   public FourBarKinematicLoopFunction createFourBarExample1(Random random, int actuatedJointIndex, boolean flipAxesRandomly)
   {
      Point2D A = new Point2D(0.227, 0.100);
      Point2D B = new Point2D(0.227, -0.100);
      Point2D C = new Point2D(0.427, 0.100);
      Point2D D = new Point2D(0.427, -0.100);
      return nextFourBar(random, A, B, C, D, actuatedJointIndex, flipAxesRandomly);
   }

   private FourBarKinematicLoopFunction nextFourBar(Random random, Point2D A, Point2D B, Point2D C, Point2D D, int actuatedJointIndex, boolean flipAxesRandomly)
   {
      Vector2D AB = new Vector2D();
      Vector2D AC = new Vector2D();
      Vector2D AD = new Vector2D();
      AB.sub(B, A);
      AC.sub(C, A);
      AD.sub(D, A);

      UnitVector3D commonAxis = EuclidCoreRandomTools.nextUnitVector3D(random);
      Vector3D axisA = new Vector3D(commonAxis);
      Vector3D axisB = new Vector3D(commonAxis);
      Vector3D axisC = new Vector3D(commonAxis);
      Vector3D axisD = new Vector3D(commonAxis);

      FramePoint3D jointAPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame);
      ReferenceFrame fourBarLocalFrame = FourBarKinematicLoopFunctionTools.constructReferenceFrameFromPointAndAxis("LocalFrame",
                                                                                                                   jointAPosition,
                                                                                                                   Axis3D.Z,
                                                                                                                   new FrameVector3D(worldFrame, commonAxis));

      FramePoint3D jointBPosition = new FramePoint3D(fourBarLocalFrame, AB);
      jointBPosition.setZ(EuclidCoreRandomTools.nextDouble(random));
      jointBPosition.changeFrame(worldFrame);
      FramePoint3D jointCPosition = new FramePoint3D(fourBarLocalFrame, AC);
      jointCPosition.setZ(EuclidCoreRandomTools.nextDouble(random));
      jointCPosition.changeFrame(worldFrame);
      FramePoint3D jointDPosition = new FramePoint3D(fourBarLocalFrame, AD);
      jointDPosition.setZ(EuclidCoreRandomTools.nextDouble(random));
      jointDPosition.changeFrame(worldFrame);

      if (flipAxesRandomly)
      { // Flip any but actuated joint such that the clockwise order is respected.
         if (actuatedJointIndex != 0 && random.nextBoolean())
            axisA.negate();
         if (actuatedJointIndex != 1 && random.nextBoolean())
            axisB.negate();
         if (actuatedJointIndex != 2 && random.nextBoolean())
            axisC.negate();
         if (actuatedJointIndex != 3 && random.nextBoolean())
            axisD.negate();
      }

      RigidBody rootBody = new RigidBody("root", worldFrame);
      RevoluteJoint jointA = new RevoluteJoint("jointA", rootBody, jointAPosition, axisA);
      RevoluteJoint jointB = new RevoluteJoint("jointB", rootBody, jointBPosition, axisB);
      RigidBody bodyDA = new RigidBody("bodyDA", jointA, 0, 0, 0, 0, new Vector3D());
      RigidBody bodyBC = new RigidBody("bodyBC", jointB, 0, 0, 0, 0, new Vector3D());
      jointCPosition.changeFrame(jointB.getFrameAfterJoint());
      RevoluteJoint jointC = new RevoluteJoint("jointC", bodyBC, jointCPosition, axisC);
      jointDPosition.changeFrame(jointA.getFrameAfterJoint());
      RevoluteJoint jointD = new RevoluteJoint("jointD", bodyDA, jointDPosition, axisD);

      RigidBody bodyCD = new RigidBody("bodyCD", jointD, 0, 0, 0, 0, new Vector3D());
      jointCPosition.changeFrame(jointD.getFrameAfterJoint());
      jointC.setupLoopClosure(bodyCD, new RigidBodyTransform(new Quaternion(), jointCPosition));
      List<RevoluteJoint> joints = new ArrayList<>(Arrays.asList(jointA, jointB, jointC, jointD));
      Collections.shuffle(joints, random);
      return new FourBarKinematicLoopFunction("fourBar", joints, actuatedJointIndex);
   }
}
