package frc.robot.subsystems.mechanisms.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ArmPosition {
	/**
	 * The elevator position, as a distance from the fully retracted position.
	 */
	public Distance elevatorPosition;
	/**
	 * The pivot angle, as an angle from the field (90 degrees is straight up).
	 */
	public Angle pivotAngle;

	/**
	 * A combined elevator and pivot position.
	 *
	 * @param elevatorPosition
	 *            The {@link #elevatorPosition} to set.
	 * @param pivotAngle
	 *            The {@link #pivotAngle} to set.
	 */
	public ArmPosition(Distance elevatorPosition, Angle pivotAngle) {
		this.elevatorPosition = elevatorPosition;
		this.pivotAngle = pivotAngle;
	}

	/**
	 * Gets the {@link #elevatorPosition}.
	 *
	 * @return
	 *         The elevator's position.
	 */
	public Distance getElevatorPosition() {
		return elevatorPosition;
	}

	/**
	 * Gets the {@link #pivotAngle}.
	 *
	 * @return
	 *         The pivot's angle.
	 */
	public Angle getPivotAngle() {
		return pivotAngle;
	}
}
