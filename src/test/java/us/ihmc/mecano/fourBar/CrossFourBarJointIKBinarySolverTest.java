package us.ihmc.mecano.fourBar;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;

public class CrossFourBarJointIKBinarySolverTest
{
   private static final int ITERATIONS = 50000;
   private static final double EPSILON = 1.0e-7;

   @Test
   public void test()
   {
      Random random = new Random(4534);
      FourBar fourBar = new FourBar();
      CrossFourBarJointIKBinarySolver defaultSolver = new CrossFourBarJointIKBinarySolver(EPSILON);
      defaultSolver.setUseNaiveMethod(false);
      CrossFourBarJointIKBinarySolver naiveSolver = new CrossFourBarJointIKBinarySolver(EPSILON);
      naiveSolver.setUseNaiveMethod(true);

      long defaultMethodTotalTimeNano = 0L;
      long naiveMethodTotalTimeNano = 0L;
      long start, end;

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Point2D> vertices = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 5.0, 4);
         int flippedIndex = random.nextInt(4);
         Collections.swap(vertices, flippedIndex, (flippedIndex + 1) % 4);
         Point2D A = vertices.get(0);
         Point2D B = vertices.get(1);
         Point2D C = vertices.get(2);
         Point2D D = vertices.get(3);

         fourBar.setup(A, B, C, D);

         FourBarAngle source = FourBarAngle.values[random.nextInt(4)];
         FourBarVertex sourceVertex = fourBar.getVertex(source);
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, sourceVertex.getMinAngle(), sourceVertex.getMaxAngle());
         fourBar.update(source, expectedAngle);

         // Retrieving the set of 2 vertices which angles sum up to theta, the 2 vertices that are not selected sum up to -theta.
         FourBarEdge nonFlippedEdge = fourBar.getEdgeAB();
         while (nonFlippedEdge.isCrossing() || nonFlippedEdge.isFlipped())
            nonFlippedEdge = nonFlippedEdge.getNext();

         double theta = nonFlippedEdge.getEnd().getAngle() + nonFlippedEdge.getEnd().getNextVertex().getAngle();

         start = System.nanoTime();
         double actualAngle = defaultSolver.solve(theta, sourceVertex);
         assertEquals(expectedAngle, actualAngle, EPSILON);
         end = System.nanoTime();
         defaultMethodTotalTimeNano += end - start;

         start = System.nanoTime();
         actualAngle = naiveSolver.solve(theta, sourceVertex);
         assertEquals(expectedAngle, actualAngle, EPSILON);
         end = System.nanoTime();
         naiveMethodTotalTimeNano += end - start;
      }

      System.out.println(getClass().getSimpleName() + " Default method average solve time: " + (defaultMethodTotalTimeNano / ITERATIONS / 1.0e6) + "millisec");
      System.out.println(getClass().getSimpleName() + " Naive method average solve time: " + (naiveMethodTotalTimeNano / ITERATIONS / 1.0e6) + "millisec");
   }

}
