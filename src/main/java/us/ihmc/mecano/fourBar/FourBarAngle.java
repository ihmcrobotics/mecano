package us.ihmc.mecano.fourBar;

/**
 * <p>
 * Given the following four bar linkage name convention:
 * 
 * <pre>
 *    D--------A
 *    |\      /|
 *    | \    / |
 *    |  \  /  |
 *    |   \/   |
 *    |   /\   |
 *    |  /  \  |
 *    | /    \ |
 *    |/      \|
 *    C--------B
 * </pre>
 * 
 * The angles are named as follows:
 * <ul>
 * <li>Inner angle at vertex A: DAB
 * <li>Inner angle at vertex B: ABC
 * <li>Inner angle at vertex C: BCD
 * <li>Inner angle at vertex D: CDA
 * </ul>
 */
public enum FourBarAngle
{
   /** Inner angle at vertex A. */
   DAB,
   /** Inner angle at vertex B. */
   ABC,
   /** Inner angle at vertex C. */
   BCD,
   /** Inner angle at vertex D. */
   CDA;

   /**
    * Static final field holding the return from {@link #values()}. This field should be used in place
    * of calling values() for garbage-free operations.
    */
   public static final FourBarAngle[] values = values();
}