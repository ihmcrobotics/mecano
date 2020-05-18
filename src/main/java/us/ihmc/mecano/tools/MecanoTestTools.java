package us.ihmc.mecano.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.getStringFormat;
import static us.ihmc.mecano.tools.MecanoIOTools.getMomentumString;
import static us.ihmc.mecano.tools.MecanoIOTools.getSpatialAccelerationString;
import static us.ihmc.mecano.tools.MecanoIOTools.getSpatialForceString;
import static us.ihmc.mecano.tools.MecanoIOTools.getSpatialInertiaString;
import static us.ihmc.mecano.tools.MecanoIOTools.getSpatialVectorString;
import static us.ihmc.mecano.tools.MecanoIOTools.getTwistString;
import static us.ihmc.mecano.tools.MecanoIOTools.getWrenchString;

import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

/**
 * This class provides the tools to perform a variety of assertions on the various types of the IHMC
 * Rigid-Body Dynamics Library.
 *
 * @author Sylvain Bertrand
 */
public class MecanoTestTools
{
   private static final String DEFAULT_FORMAT = getStringFormat(15, 12);

   /**
    * Asserts on a per component basis that the two spatial vectors are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected spatial vector. Not modified.
    * @param actual   the actual spatial vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two spatial vectors are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertSpatialVectorEquals(SpatialVectorReadOnly expected, SpatialVectorReadOnly actual, double epsilon)
   {
      assertSpatialVectorEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two spatial vectors are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected spatial vector. Not modified.
    * @param actual        the actual spatial vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two spatial vectors are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertSpatialVectorEquals(String messagePrefix, SpatialVectorReadOnly expected, SpatialVectorReadOnly actual, double epsilon)
   {
      assertSpatialVectorEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two spatial vectors are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected spatial vector. Not modified.
    * @param actual        the actual spatial vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two spatial vectors are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertSpatialVectorEquals(String messagePrefix, SpatialVectorReadOnly expected, SpatialVectorReadOnly actual, double epsilon,
                                                String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two spatial force vectors are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected spatial force vector. Not modified.
    * @param actual   the actual spatial force vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two spatial force vectors are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialForceEquals(SpatialForceReadOnly expected, SpatialForceReadOnly actual, double epsilon)
   {
      assertSpatialForceEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two spatial force vectors are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected spatial force vector. Not modified.
    * @param actual        the actual spatial force vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two spatial force vectors are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialForceEquals(String messagePrefix, SpatialForceReadOnly expected, SpatialForceReadOnly actual, double epsilon)
   {
      assertSpatialForceEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two spatial force vectors are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected spatial force vector. Not modified.
    * @param actual        the actual spatial force vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two spatial force vectors are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialForceEquals(String messagePrefix, SpatialForceReadOnly expected, SpatialForceReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two wrenches are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected wrench. Not modified.
    * @param actual   the actual wrench. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two wrenches are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertWrenchEquals(WrenchReadOnly expected, WrenchReadOnly actual, double epsilon)
   {
      assertWrenchEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two wrenches are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected wrench. Not modified.
    * @param actual        the actual wrench. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two wrenches are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertWrenchEquals(String messagePrefix, WrenchReadOnly expected, WrenchReadOnly actual, double epsilon)
   {
      assertWrenchEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two wrenches are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected wrench. Not modified.
    * @param actual        the actual wrench. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two wrenches are not equal. If only one of the arguments is equal
    *                        to {@code null}.
    */
   public static void assertWrenchEquals(String messagePrefix, WrenchReadOnly expected, WrenchReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two twists are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected twist. Not modified.
    * @param actual   the actual spatial twist. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two twists are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTwistEquals(TwistReadOnly expected, TwistReadOnly actual, double epsilon)
   {
      assertTwistEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two twists are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected twist. Not modified.
    * @param actual        the actual spatial twist. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two twists are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTwistEquals(String messagePrefix, TwistReadOnly expected, TwistReadOnly actual, double epsilon)
   {
      assertTwistEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two twists are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected twist. Not modified.
    * @param actual        the actual spatial twist. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two twists are not equal. If only one of the arguments is equal to
    *                        {@code null}.
    */
   public static void assertTwistEquals(String messagePrefix, TwistReadOnly expected, TwistReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two spatial acceleration vectors are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected spatial acceleration vector. Not modified.
    * @param actual   the actual spatial acceleration vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two spatial acceleration vectors are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialAccelerationEquals(SpatialAccelerationReadOnly expected, SpatialAccelerationReadOnly actual, double epsilon)
   {
      assertSpatialAccelerationEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two spatial acceleration vectors are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected spatial acceleration vector. Not modified.
    * @param actual        the actual spatial acceleration vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two spatial acceleration vectors are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialAccelerationEquals(String messagePrefix, SpatialAccelerationReadOnly expected, SpatialAccelerationReadOnly actual,
                                                      double epsilon)
   {
      assertSpatialAccelerationEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two spatial acceleration vectors are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected spatial acceleration vector. Not modified.
    * @param actual        the actual spatial acceleration vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two spatial acceleration vectors are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialAccelerationEquals(String messagePrefix, SpatialAccelerationReadOnly expected, SpatialAccelerationReadOnly actual,
                                                      double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two momentum vectors are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected momentum vector. Not modified.
    * @param actual   the actual momentum vector. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two momentum vectors are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertMomentumEquals(MomentumReadOnly expected, MomentumReadOnly actual, double epsilon)
   {
      assertMomentumEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two momentum vectors are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected momentum vector. Not modified.
    * @param actual        the actual momentum vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two momentum vectors are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertMomentumEquals(String messagePrefix, MomentumReadOnly expected, MomentumReadOnly actual, double epsilon)
   {
      assertMomentumEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two momentum vectors are equal to an {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected momentum vector. Not modified.
    * @param actual        the actual momentum vector. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two momentum vectors are not equal. If only one of the arguments is
    *                        equal to {@code null}.
    */
   public static void assertMomentumEquals(String messagePrefix, MomentumReadOnly expected, MomentumReadOnly actual, double epsilon, String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts on a per component basis that the two spatial inertia matrices are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param expected the expected spatial inertia matrix. Not modified.
    * @param actual   the actual spatial inertia matrix. Not modified.
    * @param epsilon  the tolerance to use.
    * @throws AssertionError if the two spatial inertia matrices are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialInertiaEquals(SpatialInertiaReadOnly expected, SpatialInertiaReadOnly actual, double epsilon)
   {
      assertSpatialInertiaEquals(null, expected, actual, epsilon);
   }

   /**
    * Asserts on a per component basis that the two spatial inertia matrices are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected spatial inertia matrix. Not modified.
    * @param actual        the actual spatial inertia matrix. Not modified.
    * @param epsilon       the tolerance to use.
    * @throws AssertionError if the two spatial inertia matrices are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialInertiaEquals(String messagePrefix, SpatialInertiaReadOnly expected, SpatialInertiaReadOnly actual, double epsilon)
   {
      assertSpatialInertiaEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   /**
    * Asserts on a per component basis that the two spatial inertia matrices are equal to an
    * {@code epsilon}.
    * <p>
    * Note: the two arguments are considered to be equal if they are both equal to {@code null}.
    * </p>
    *
    * @param messagePrefix prefix to add to the automated message.
    * @param expected      the expected spatial inertia matrix. Not modified.
    * @param actual        the actual spatial inertia matrix. Not modified.
    * @param epsilon       the tolerance to use.
    * @param format        the format to use for printing each component when an {@code AssertionError}
    *                      is thrown.
    * @throws AssertionError if the two spatial inertia matrices are not equal. If only one of the
    *                        arguments is equal to {@code null}.
    */
   public static void assertSpatialInertiaEquals(String messagePrefix, SpatialInertiaReadOnly expected, SpatialInertiaReadOnly actual, double epsilon,
                                                 String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!expected.epsilonEquals(actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   /**
    * Asserts that when executing the given runnable, an specific exception is thrown.
    *
    * @param runnable              the code to be executed and to be throwing an exception.
    * @param expectedExceptionType the expected type of the exception to catch when executing the
    *                              runnable.
    * @throws AssertionError if the no exception is thrown or if the thrown exception is not equal to
    *                        {@code expectedExceptionType}.
    */
   public static void assertExceptionIsThrown(Runnable runnable, Class<? extends Exception> expectedExceptionType)
   {
      Exception exceptionCaught = null;

      try
      {
         runnable.run();
      }
      catch (Exception e)
      {
         exceptionCaught = e;
      }
      finally
      {
         if (exceptionCaught == null)
            throw new AssertionError("The operation should have thrown an exception.");
         if (!exceptionCaught.getClass().equals(expectedExceptionType))
            throw new AssertionError("Unexpected exception: expected = " + expectedExceptionType.getSimpleName() + ", actual = "
                  + exceptionCaught.getClass().getSimpleName());
      }
   }

   private static void throwNotEqualAssertionError(String messagePrefix, SpatialVectorReadOnly expected, SpatialVectorReadOnly actual, String format)
   {
      String expectedAsString = getSpatialVectorString(format, expected);
      String actualAsString = getSpatialVectorString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(computeNormError(expected, actual)));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, SpatialForceReadOnly expected, SpatialForceReadOnly actual, String format)
   {
      String expectedAsString = getSpatialForceString(format, expected);
      String actualAsString = getSpatialForceString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(computeNormError(expected, actual)));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, WrenchReadOnly expected, WrenchReadOnly actual, String format)
   {
      String expectedAsString = getWrenchString(format, expected);
      String actualAsString = getWrenchString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(computeNormError(expected, actual)));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, TwistReadOnly expected, TwistReadOnly actual, String format)
   {
      String expectedAsString = getTwistString(format, expected);
      String actualAsString = getTwistString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(computeNormError(expected, actual)));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, SpatialAccelerationReadOnly expected, SpatialAccelerationReadOnly actual,
                                                   String format)
   {
      String expectedAsString = getSpatialAccelerationString(format, expected);
      String actualAsString = getSpatialAccelerationString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(computeNormError(expected, actual)));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, MomentumReadOnly expected, MomentumReadOnly actual, String format)
   {
      String expectedAsString = getMomentumString(format, expected);
      String actualAsString = getMomentumString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, Double.toString(computeNormError(expected, actual)));
   }

   private static void throwNotEqualAssertionError(String messagePrefix, SpatialInertiaReadOnly expected, SpatialInertiaReadOnly actual, String format)
   {
      String expectedAsString = getSpatialInertiaString(format, expected);
      String actualAsString = getSpatialInertiaString(format, actual);
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString)
   {
      throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString, null);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, String expectedAsString, String actualAsString, String differenceAsString)
   {
      String errorMessage = addPrefixToMessage(messagePrefix, "expected:\n" + expectedAsString + "\n but was:\n" + actualAsString);
      if (differenceAsString != null)
         errorMessage += "\nDifference of: " + differenceAsString;

      throw new AssertionError(errorMessage);
   }

   private static String addPrefixToMessage(String prefix, String message)
   {
      if (prefix != null && !prefix.isEmpty())
         return prefix + " " + message;
      else
         return message;
   }

   private static double computeNormError(SpatialVectorReadOnly expected, SpatialVectorReadOnly actual)
   {
      SpatialVector spatialError = new SpatialVector(expected);
      spatialError.sub(actual);
      return spatialError.length();
   }
}
