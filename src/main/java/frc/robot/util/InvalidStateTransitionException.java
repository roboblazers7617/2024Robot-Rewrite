package frc.robot.util;

/**
 * Exception thrown when an invalid state transition occurs in a state machine.
 */
public class InvalidStateTransitionException extends Exception {
	/**
	 * Creates a new InvalidStateTransitionException with the given message.
	 *
	 * @param message
	 *            Message to use.
	 */
	public InvalidStateTransitionException(String message) {
		super(message);
	}

	/**
	 * Creates a new InvalidStateTransitionException with no message.
	 */
	public InvalidStateTransitionException() {
		super();
	}
}
