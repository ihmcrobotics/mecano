package us.ihmc.mecano.exceptions;

/**
 * {@code ScrewTheoryException} is used for handling exception thrown from the screw theory library.
 */
public class ScrewTheoryException extends RuntimeException
{
   private static final long serialVersionUID = -4504468232252668130L;

   /**
    * Constructs an {@code ScrewTheoryException} with the specified detail message.
    *
    * @param message the detail message.
    */
   public ScrewTheoryException(String message)
   {
      super(message);
   }
}
