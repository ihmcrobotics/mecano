package us.ihmc.mecano.fourBar;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.fourBar.FourBarKinematicLoopFunctionTools.FourBarToJointConverter;

/**
 * Numerical solver that performs a binary search using the four bar range of motion as initial
 * boundaries.
 */
public class CrossFourBarJointIKBinarySolver implements CrossFourBarJointIKSolver
{
   private static final boolean DEBUG = false;

   private final double tolerance;
   private final int maxIterations;
   private int iterations;
   private final FourBar fourBar = new FourBar();
   private boolean useNaiveMethod = false;

   private FourBarToJointConverter[] converters = null;

   /**
    * Creates a new solver.
    * 
    * @param tolerance used as terminal condition for the binary search.
    */
   public CrossFourBarJointIKBinarySolver(double tolerance)
   {
      this(tolerance, 100);
   }

   /**
    * Creates a new solver.
    * 
    * @param tolerance     used as terminal condition for the binary search.
    * @param maxIterations upper limit on the number of iterations for the binary search.
    */
   public CrossFourBarJointIKBinarySolver(double tolerance, int maxIterations)
   {
      this.tolerance = tolerance;
      this.maxIterations = maxIterations;
   }

   /**
    * This class implements two solvers, a simple naive solver that is meant for education, and a
    * (default) solver that is less understandable but more efficient computation-wise.
    * 
    * @param useNaiveMethod {@code true} to use the naive solver (not recommended), {@code false}
    *                       (default) to use the more efficient solver.
    */
   public void setUseNaiveMethod(boolean useNaiveMethod)
   {
      this.useNaiveMethod = useNaiveMethod;
   }

   /** {@inheritDoc} */
   @Override
   public void setConverters(FourBarToJointConverter[] converters)
   {
      this.converters = converters;
   }

   private final SolutionBounds solutionBounds = new SolutionBounds();
   private FourBarVertex lastVertexToSolveFor = null;

   /** {@inheritDoc} */
   @Override
   public double solve(double theta, FourBarVertex vertexToSolveFor)
   {
      // Check if the user is requesting for the same configuration as the last that was solved.
      if (vertexToSolveFor == lastVertexToSolveFor && solutionBounds.isInsideBounds(theta))
         return convert(vertexToSolveFor, solutionBounds.computeSolution(theta));

      if (!FourBarTools.isCrossFourBar(vertexToSolveFor))
         throw new IllegalArgumentException("The given vertex does not belong to a cross four bar.");

      lastVertexToSolveFor = vertexToSolveFor;
      solutionBounds.clear();

      double angle = isThetaAtLimit(theta, vertexToSolveFor, solutionBounds);

      if (!Double.isNaN(angle))
      {
         solutionBounds.clear();
         return angle;
      }

      // The search uses the property that theta is the sum of the angles of the two vertices that are the end-points of a crossing edge.
      // Note that there are 2 sets of such vertices, while the sum of a first set is equal to theta, the other is equal to -theta.
      if (vertexToSolveFor.isConvex() != vertexToSolveFor.getNextEdge().isCrossing())
      {
         if (DEBUG)
            System.out.println("Flip " + vertexToSolveFor.getName());
         theta = -theta;
      }

      if (useNaiveMethod)
         solveNaive(theta, vertexToSolveFor, solutionBounds);
      else
         solveInternal(theta, vertexToSolveFor, solutionBounds);

      return convert(vertexToSolveFor, solutionBounds.computeSolution(theta));
   }

   private double isThetaAtLimit(double theta, FourBarVertex vertexToSolveFor, SolutionBounds boundsToPack)
   {
      FourBarVertex A = vertexToSolveFor;
      boolean isNextCrossing = A.getNextEdge().isCrossing();
      FourBarVertex B = isNextCrossing ? A.getNextVertex() : A.getPreviousVertex();

      boundsToPack.minDAB = A.getMinAngle();
      boundsToPack.maxDAB = A.getMaxAngle();
      double minAngleA = convert(A, boundsToPack.minDAB);
      double maxAngleA = convert(A, boundsToPack.maxDAB);
      boundsToPack.thetaMin = minAngleA + convert(B, B.getMinAngle());
      boundsToPack.thetaMax = maxAngleA + convert(B, B.getMaxAngle());

      boundsToPack.limitsInverted = boundsToPack.thetaMin > boundsToPack.thetaMax;

      if (boundsToPack.limitsInverted)
      {
         boundsToPack.thetaMin = -boundsToPack.thetaMin;
         boundsToPack.thetaMax = -boundsToPack.thetaMax;
      }

      if (A.isConvex() == isNextCrossing)
      {
         if (theta <= boundsToPack.thetaMin + tolerance)
            return minAngleA;
         else if (theta >= boundsToPack.thetaMax - tolerance)
            return maxAngleA;
      }
      else
      {
         if (-theta <= boundsToPack.thetaMin + tolerance)
            return minAngleA;
         else if (-theta >= boundsToPack.thetaMax - tolerance)
            return maxAngleA;
      }

      return Double.NaN;
   }

   private void solveNaive(double theta, FourBarVertex vertexToSolveFor, SolutionBounds bounds)
   {
      long start = System.nanoTime();
      setupFourBar(vertexToSolveFor);

      FourBarToJointConverter converterA = null;
      FourBarToJointConverter converterB = null;
      FourBarToJointConverter converterD = null;

      if (converters != null)
      {
         converterA = getConverter(vertexToSolveFor);
         converterB = getConverter(vertexToSolveFor.getNextVertex());
         converterD = getConverter(vertexToSolveFor.getPreviousVertex());
      }

      double minDAB = bounds.minDAB;
      double maxDAB = bounds.maxDAB;
      double thetaMin = bounds.thetaMin;
      double thetaMax = bounds.thetaMax;

      double currentDAB = Double.NaN;

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         currentDAB = 0.5 * (minDAB + maxDAB);
         fourBar.update(FourBarAngle.DAB, currentDAB);
         double actualTheta;
         if (converterA == null)
            actualTheta = currentDAB;
         else
            actualTheta = converterA.toJointAngle(currentDAB);

         if (fourBar.getEdgeDA().isCrossing())
         { // theta = DAB + CDA
            actualTheta += convert(converterD, fourBar.getAngleCDA());
         }
         else
         { // theta = DAB + ABC
            actualTheta += convert(converterB, fourBar.getAngleABC());
         }

         if (bounds.limitsInverted)
            actualTheta = -actualTheta;

         if (actualTheta > theta)
         {
            maxDAB = currentDAB;
            thetaMax = actualTheta;
         }
         else
         {
            minDAB = currentDAB;
            thetaMin = actualTheta;
         }

         if (Math.abs(thetaMax - thetaMin) <= tolerance)
            break;
      }

      if (DEBUG)
         System.out.println("Iterations: " + iterations + ", time elapsed: " + (System.nanoTime() - start) / 1.0e6 + "millisec");

      bounds.thetaMin = thetaMin;
      bounds.thetaMax = thetaMax;
      bounds.minDAB = minDAB;
      bounds.maxDAB = maxDAB;
   }

   private void setupFourBar(FourBarVertex vertexToSolverFor)
   {
      FourBarVertex A = vertexToSolverFor;
      FourBarVertex B = A.getNextVertex();
      FourBarVertex C = B.getNextVertex();
      FourBarVertex D = C.getNextVertex();
      fourBar.setup(A.getNextEdge().getLength(),
                    B.getNextEdge().getLength(),
                    C.getNextEdge().getLength(),
                    D.getNextEdge().getLength(),
                    A.isConvex(),
                    B.isConvex(),
                    C.isConvex(),
                    D.isConvex());
   }

   private void solveInternal(double theta, FourBarVertex vertexToSolveFor, SolutionBounds bounds)
   {
      long start = System.nanoTime();
      FourBarVertex A = vertexToSolveFor;
      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = ABEdge.getNext();
      FourBarEdge CDEdge = BCEdge.getNext();
      FourBarEdge DAEdge = CDEdge.getNext();
      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();

      double minDAB = bounds.minDAB;
      double maxDAB = bounds.maxDAB;
      double thetaMin = bounds.thetaMin;
      double thetaMax = bounds.thetaMax;

      FourBarToJointConverter converterA = null;
      FourBarToJointConverter converterB = null;
      FourBarToJointConverter converterD = null;

      if (converters != null)
      {
         converterA = getConverter(vertexToSolveFor);
         converterB = getConverter(vertexToSolveFor.getNextVertex());
         converterD = getConverter(vertexToSolveFor.getPreviousVertex());
      }

      double currentDAB = Double.NaN;

      /*
       * TODO: Ideally we want to avoid relying onto Math.acos(...) that is really expensive. When only
       * using the cosine values to do the search, the algorithm works almost all the time, but at
       * occasion it fails pretty bad. There's obviously some edge-case that is not properly considered
       * and I couldn't figure it out yet, so for now it is safer to compare angles directly.
       */

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         currentDAB = 0.5 * (minDAB + maxDAB);

         double otherAngle;

         double BD = EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(AB, DA, currentDAB);

         if (DAEdge.isCrossing())
         { // theta = DAB + CDA
            double cosADB = FourBarTools.cosineAngleWithCosineLaw(DA, BD, AB);
            double cosCDB = FourBarTools.cosineAngleWithCosineLaw(CD, BD, BC);
            // Using property: acos(x) - acos(y) = acos(xy + sqrt((1 - xx)(1 - yy))
            // angleCDA
            otherAngle = Math.abs(FourBarTools.fastAcos(cosADB * cosCDB + Math.sqrt((1.0 - cosADB * cosADB) * (1.0 - cosCDB * cosCDB))));
            if (!DAEdge.getStart().isConvex())
               otherAngle = -otherAngle;
            otherAngle = convert(converterD, otherAngle);
         }
         else
         { // theta = DAB + ABC
            double cosABD = FourBarTools.cosineAngleWithCosineLaw(AB, BD, DA);
            double cosCBD = FourBarTools.cosineAngleWithCosineLaw(BC, BD, CD);
            // Using property: acos(x) - acos(y) = acos(xy + sqrt((1 - xx)(1 - yy))
            // angleABC
            otherAngle = Math.abs(FourBarTools.fastAcos(cosABD * cosCBD + Math.sqrt((1.0 - cosABD * cosABD) * (1.0 - cosCBD * cosCBD))));
            if (!BCEdge.getStart().isConvex())
               otherAngle = -otherAngle;
            otherAngle = convert(converterB, otherAngle);
         }

         double actualTheta = convert(converterA, currentDAB) + otherAngle;

         if (bounds.limitsInverted)
            actualTheta = -actualTheta;

         if (actualTheta > theta)
         {
            maxDAB = currentDAB;
            thetaMax = actualTheta;
         }
         else
         {
            minDAB = currentDAB;
            thetaMin = actualTheta;
         }

         if (Math.abs(thetaMax - thetaMin) <= tolerance)
            break;
      }

      if (DEBUG)
         System.out.println("Iterations: " + iterations + ", time elapsed: " + (System.nanoTime() - start) / 1.0e6 + "millisec");

      bounds.thetaMin = thetaMin;
      bounds.thetaMax = thetaMax;
      bounds.minDAB = minDAB;
      bounds.maxDAB = maxDAB;
   }

   private double convert(FourBarVertex vertex, double vertexAngle)
   {
      return convert(getConverter(vertex), vertexAngle);
   }

   private double convert(FourBarToJointConverter converter, double vertexAngle)
   {
      return converter == null ? vertexAngle : converter.toJointAngle(vertexAngle);
   }

   private FourBarToJointConverter getConverter(FourBarVertex vertex)
   {
      return converters == null ? null : converters[vertex.getFourBarAngle().ordinal()];
   }

   private static class SolutionBounds
   {
      private boolean limitsInverted;
      private double thetaMin, thetaMax;
      private double minDAB, maxDAB;

      public SolutionBounds()
      {
         clear();
      }

      private void clear()
      {
         thetaMin = Double.NaN;
         thetaMax = Double.NaN;
         minDAB = Double.NaN;
         maxDAB = Double.NaN;
      }

      private boolean isInsideBounds(double theta)
      {
         if (Double.isNaN(thetaMin) || Double.isNaN(thetaMax))
            return false;
         return theta >= thetaMin && theta <= thetaMax;
      }

      private double computeSolution(double theta)
      {
         double alpha = (theta - thetaMin) / (thetaMax - thetaMin);
         return EuclidCoreTools.interpolate(minDAB, maxDAB, alpha);
      }
   }
}
