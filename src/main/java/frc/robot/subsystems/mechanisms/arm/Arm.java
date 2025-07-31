package frc.robot.subsystems.mechanisms.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A superstructure to control both the Elevator and Pivot. Includes some important safety stuff so
 * this shouldn't be skipped.
 */
public class Arm extends SubsystemBase {
	/**
	 * The Elevator component of the Arm.
	 */
	private final Elevator elevator;
	/**
	 * The Pivot component of the Arm.
	 */
	private final Pivot pivot;

	/**
	 * Creates a new Arm subsystem using a new Elevator and Pivot.
	 */
	public Arm() {
		this(new Elevator(), new Pivot());
	}

	/**
	 * Creates a new Arm subsystem using the provided Elevator and Pivot.
	 *
	 * @param elevator
	 *            The Elevator to use.
	 * @param pivot
	 *            The Pivot to use.
	 */
	public Arm(Elevator elevator, Pivot pivot) {
		this.elevator = new Elevator();
		this.pivot = new Pivot();
	}

	/**
	 * A command to set the elevator and wrist to a position.
	 *
	 * @param position
	 *            The Constants.ArmPosition to set the elevator and wrist to.
	 * @return
	 *         {@link Command} to run.
	 */
	public Command setPositionCommand(ArmPosition position) {
		Command command = new Command() {
			@Override
			public void initialize() {
				elevator.setTarget(position.getElevatorPosition());
				// pivot.setPosition(position.getPivotAngle());
			}

			@Override
			public boolean isFinished() {
				return isAtTarget();
			}

			@Override
			public void end(boolean interrupted) {
				System.out.println("done. interrupted: " + interrupted);
			}
		};
		command.addRequirements(this);

		return command;
	}

	/**
	 * Checks if the {@link #elevator} and {@link #pivot} are at their target positions.
	 *
	 * @return
	 *         True if both the elevator and pivot are at their target positions, false otherwise.
	 */
	public Boolean isAtTarget() {
		return elevator.isAtTarget();// && pivot.isAtTarget();
	}
}
