package frc.robot.subsystems.mechanisms.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;

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
		this.elevator = elevator;
		this.pivot = pivot;

		// Add a brake toggle button to SmartDashboard.
		SmartDashboard.putData("Toggle Arm Brake Mode", toggleBrakeModesCommand());
	}

	/**
	 * This method is responsible for ensuring safe interactions between the head and the chassis of the
	 * robot.
	 * <p>
	 * This method will check the setpoint of the wrist, and, in the case that it is too low for the
	 * current elevator position, it will set the target to the lowest safe position.
	 */
	@Override
	public void periodic() {
		// If the elevator target or position is too low for the pivot target, set the pivot higher so the
		// head can clear the body of the robot
		boolean elevatorSafe = (elevator.getPosition() < ElevatorConstants.HEAD_CLEAR_POSITION) && (elevator.getTarget() < ElevatorConstants.HEAD_CLEAR_POSITION);
		if (elevatorSafe && pivot.getTarget() < PivotConstants.SAFE_MIN_POSITION) {
			pivot.setTarget(PivotConstants.SAFE_MIN_POSITION);
		}
	}

	/**
	 * Initializes the {@link #elevator} and {@link #pivot}. Should be called pre-enable.
	 */
	public void init() {
		elevator.init();
		pivot.init();
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
				pivot.setTarget(position.getPivotAngle());
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
	public boolean isAtTarget() {
		return elevator.isAtTarget() && pivot.isAtTarget();
	}

	public Command toggleBrakeModesCommand() {
		return elevator.toggleBrakeModesCommand()
				.andThen(pivot.toggleBrakeModesCommand());
	}
}
