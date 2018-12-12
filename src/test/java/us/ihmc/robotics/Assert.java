package us.ihmc.robotics;

public class Assert
{
   public static void assertArrayEquals(double[] expecteds, double[] actuals, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals);
      else
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals, delta);
   }

   static public void assertEquals(double expected, double actual, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta);
   }

   static public void assertEquals(float expected, float actual, float delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta);
   }
}
