// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem to control the elevator.
 * <p>
 * Elevator safety features are in the {@link #periodic()} method.
 */
@Logged
public class Elevator extends SubsystemBase {
	/**
	 * The FeedForward used for the elevator.
	 */
	private final ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
	/**
	 * The right elevator motor.
	 */
	private final SparkMax leaderMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/**
	 * The left elevator motor.
	 */
	private final SparkMax followerMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

	/**
	 * The {@link #leaderMotor}'s encoder.
	 */
	private final RelativeEncoder relativeEncoder = leaderMotor.getEncoder();

	/**
	 * Trapezoidal motion profile used to profile the elevator's movement.
	 */
	private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION));

	/**
	 * This is the current trapezoid profile setpoint for the elevator. It is not the final target, that
	 * is in the {@link #target} variable.
	 */
	private TrapezoidProfile.State currentSetpoint;

	/**
	 * The elevator target in meters, This is within the outer bounds of the elevator but the danger
	 * zone at the bottom has not been accounted for.
	 */
	@Logged
	private double target = 0;

	/**
	 * Creates a new Elevator.
	 */
	public Elevator() {
		// Set up the motors
		SparkMaxConfig baseElevatorConfig = new SparkMaxConfig();

		baseElevatorConfig.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
				.inverted(false);

		baseElevatorConfig.encoder
				.positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

		baseElevatorConfig.closedLoop
				.p(ElevatorConstants.KP)
				.i(ElevatorConstants.KI)
				.d(ElevatorConstants.KD)
				.outputRange(ElevatorConstants.KMIN_OUTPUT, ElevatorConstants.KMAX_OUTPUT);

		leaderMotor.configure(baseElevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkBaseConfig followerElevatorMotorConfig = new SparkMaxConfig()
				.apply(baseElevatorConfig)
				.follow(leaderMotor, true);
		followerMotor.configure(followerElevatorMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		// Set up the trapezoidal profile
		currentSetpoint = new TrapezoidProfile.State(getPosition(), 0);

		// Enable brake mode on robot enable
		RobotModeTriggers.disabled()
				.onFalse(enableBrakeModeCommand());

		// Put some handy things on SmartDashboard
		SmartDashboard.putData("Toggle Elevator Brake Mode", toggleBrakeModeCommand());
		SmartDashboard.putData("Stop Elevator Command", doNothing());
	}

	/**
	 * This method is responsible for they safety of the elevator.
	 * <p>
	 * This uses the {@link #target} to determine the target position of the elevator, and
	 * constrain said target to the safe bounds.
	 */
	@Override
	public void periodic() {
		// Ensure the elevator target is within bounds
		target = MathUtil.clamp(target, ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION);

		// Calculate the PID setpoint using the trapezoidal profile
		currentSetpoint = trapezoidProfile.calculate(0.02, currentSetpoint, new TrapezoidProfile.State(target, 0));

		// Calculate a feedforward for the motor to keep the elevator up
		// TODO: Compensate for pivot position here
		double elevatorFeedForwardValue = feedforward.calculate(currentSetpoint.velocity); // this is technically supposed to be the velocity setpoint

		leaderMotor.getClosedLoopController()
				.setReference(currentSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, elevatorFeedForwardValue, ArbFFUnits.kVoltage);
	}

	/**
	 * Sets up the elevator. Should be called pre-enable.
	 */
	public void init() {
		setTarget(getPosition());
		currentSetpoint = new TrapezoidProfile.State(getPosition(), 0);
	}

	/**
	 * A method to move the elevator to a position.
	 *
	 * @param position
	 *            The position as a distance.
	 */
	public void setTarget(Distance position) {
		setTarget(position.in(Meters));
	}

	/**
	 * A method to move the elevator to a position in meters.
	 *
	 * @param position
	 *            The position in meters.
	 */
	public void setTarget(double position) {
		target = MathUtil.clamp(position, ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION);
	}

	/**
	 * Gets the current {@link #target}.
	 *
	 * @return
	 *         The elevator target in meters.
	 */
	public double getTarget() {
		return target;
	}

	/**
	 * Gets the current position of the elevator.
	 *
	 * @return
	 *         The elevator position in meters.
	 */
	public double getPosition() {
		return relativeEncoder.getPosition();
	}

	/**
	 * Check if the elevator is within the tolerance to it's target position. This is used to determine
	 * if the {@link Arm#setPositionCommand(ArmPosition)} command is finished.
	 */
	public boolean isAtTarget() {
		return Math.abs(getPosition() - target) < ElevatorConstants.TOLERANCE;
	}

	/**
	 * A command to set the speed of the elevator. Used for relative control.
	 *
	 * @param elevatorSpeed
	 *            The speed of the elevator as a percentage of max speed. [-1, 1]
	 * @return
	 *         {@link Command} to run.
	 */
	public Command setSpeedCommand(DoubleSupplier elevatorSpeed) {
		Command command = new Command() {
			private boolean isJoystickCentered = false;

			@Override
			public void execute() {
				double targetElevatorSpeed = MathUtil.clamp(elevatorSpeed.getAsDouble() * ElevatorConstants.MAX_VELOCITY, -ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_VELOCITY);

				if (targetElevatorSpeed != 0) {
					setTarget(target + (targetElevatorSpeed / 50)); // divide the speed by 50 because their are 50 loops per second
					isJoystickCentered = false;
				} else if (!isJoystickCentered) {
					setTarget(getPosition());
					currentSetpoint = new TrapezoidProfile.State(getPosition(), 0);
					isJoystickCentered = true;
				}
			}

			@Override
			public boolean isFinished() {
				return false;
			}
		};
		command.addRequirements(this);
		return command;
	}

	/**
	 * A command that does nothing. This command requires the elevator subsystem so it will kill any
	 * other command. This is used to let the drivers regain controll of the elevator if it is unable to
	 * reach its target.
	 *
	 * @return Command to run.
	 */
	public Command doNothing() {
		Command command = Commands.none();
		command.addRequirements(this);
		return command;
	}

	/**
	 * Toggles brake mode on the elevator motors.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command toggleBrakeModeCommand() {
		return this.runOnce(() -> {
			SparkBaseConfig newElevatorConfig = new SparkMaxConfig();
			if (leaderMotor.configAccessor.getIdleMode() == IdleMode.kBrake) {
				newElevatorConfig.idleMode(IdleMode.kCoast);
			} else {
				newElevatorConfig.idleMode(IdleMode.kBrake);
			}
			leaderMotor.configure(newElevatorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
			followerMotor.configure(newElevatorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
		}).ignoringDisable(true);
	}

	/**
	 * Enables brake mode on the elevator motors. Called on enable to ensure the elevator is in brake
	 * mode for the match.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command enableBrakeModeCommand() {
		return this.runOnce(() -> {
			SparkBaseConfig newElevatorConfig = new SparkMaxConfig()
					.idleMode(IdleMode.kBrake);
			leaderMotor.configure(newElevatorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
			followerMotor.configure(newElevatorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
		}).ignoringDisable(true);
	}
}
