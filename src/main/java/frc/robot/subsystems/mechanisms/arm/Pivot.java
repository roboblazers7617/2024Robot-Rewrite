package frc.robot.subsystems.mechanisms.arm;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

/**
 * Subsystem used to control the elevator pivot.
 */
@Logged
public class Pivot extends SubsystemBase {
	/**
	 * The FeedForward used for the pivot.
	 */
	private final ArmFeedforward feedforward = new ArmFeedforward(PivotConstants.KS, PivotConstants.KG, PivotConstants.KV);
	/**
	 * The pivot motor.
	 */
	private final SparkMax motor = new SparkMax(PivotConstants.MOTOR_ID, MotorType.kBrushless);
	/**
	 * The absolute encoder on the pivot.
	 */
	private final AbsoluteEncoder absoluteEncoder;
	/**
	 * The trapezoidal profile used to profile the pivot's movement.
	 */
	private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCELERATION));

	/**
	 * This is the current trapezoid profile setpoint for the pivot. It is not the final target, that is
	 * in the {@link #target} variable.
	 */
	private TrapezoidProfile.State currentSetpoint;

	/**
	 * The pivot target in degrees, This is within the outer bounds of the pivot but the danger zone at
	 * the bottom has not been accounted for.
	 */
	private double target;

	/**
	 * Creates a new Pivot.
	 */
	public Pivot() {
		// Set up the motor
		SparkMaxConfig motorConfig = new SparkMaxConfig();
		motorConfig.idleMode(IdleMode.kBrake);
		motorConfig.smartCurrentLimit(PivotConstants.CURRENT_LIMIT);
		motorConfig.inverted(true);

		motorConfig.absoluteEncoder
				.positionConversionFactor(360.0)
				.velocityConversionFactor(360 * 60)
				.zeroOffset(PivotConstants.ZERO_OFFSET)
				.inverted(true)
				.zeroCentered(true);
		absoluteEncoder = motor.getAbsoluteEncoder();

		motorConfig.encoder
				.positionConversionFactor(PivotConstants.POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR);

		motorConfig.closedLoop
				.p(PivotConstants.KP)
				.i(PivotConstants.KI)
				.d(PivotConstants.KD)
				.outputRange(PivotConstants.KMIN_OUTPUT, PivotConstants.KMAX_OUTPUT)
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

		motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		// Initialize things
		setTarget(absoluteEncoder.getPosition());
		currentSetpoint = new TrapezoidProfile.State(target, 0);

		// Put some useful things on SmartDashboard
		SmartDashboard.putData("Toggle Pivot Brake Mode", toggleBrakeModesCommand());
		SmartDashboard.putData("Stop Pivot Command", doNothing());
	}

	/**
	 * This method is responsible for they safety of the pivot.
	 * <p>
	 * This uses the {@link #target} to determine the target position of the pivot.
	 */
	@Override
	public void periodic() {
		// Ensure the pivot target is within bounds
		target = MathUtil.clamp(target, PivotConstants.MIN_POSITION, PivotConstants.MAX_POSITION);

		// Calculate the PID setpoint using the trapezoidal profile
		currentSetpoint = trapezoidProfile.calculate(0.02, currentSetpoint, new TrapezoidProfile.State(target, 0));

		// Calculate a feedforward for the motor to keep the pivot up
		double feedForwardValue = feedforward.calculate(Math.toRadians(currentSetpoint.position), Math.toRadians(currentSetpoint.velocity));

		motor.getClosedLoopController()
				.setReference(currentSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForwardValue, ArbFFUnits.kVoltage);
	}

	/**
	 * Sets up the pivot. Should be called pre-enable.
	 */
	public void init() {
		setTarget(absoluteEncoder.getPosition());
		currentSetpoint = new TrapezoidProfile.State(absoluteEncoder.getPosition(), 0);
	}

	/**
	 * A method to move the pivot to a position.
	 *
	 * @param position
	 *            The position as a angle.
	 */
	public void setTarget(Angle position) {
		setTarget(position.in(Degrees));
	}

	/**
	 * A function to move the pivot to a position in degrees.
	 *
	 * @param position
	 *            The position in degrees.
	 */
	public void setTarget(double position) {
		target = MathUtil.clamp(position, PivotConstants.MIN_POSITION, PivotConstants.MAX_POSITION);
	}

	/**
	 * Check if the pivot is within the tolerance to it's target position. This is used to determine if
	 * the {@link Arm#setPositionCommand(ArmPosition)} command is finished.
	 */
	public boolean isAtTarget() {
		return Math.abs(absoluteEncoder.getPosition() - target) < PivotConstants.TOLERANCE;
	}

	/**
	 * A command to set the speed of the pivot. Used for relative control.
	 *
	 * @param pivotSpeed
	 *            The speed of the pivot as a percentage of max speed. [-1, 1]
	 * @return
	 *         {@link Command} to run.
	 */
	public Command setSpeedCommand(DoubleSupplier pivotSpeed) {
		Command command = new Command() {
			private boolean isJoystickCentered = false;

			@Override
			public void execute() {
				double targetSpeed = MathUtil.clamp(pivotSpeed.getAsDouble() * PivotConstants.MAX_VELOCITY, -PivotConstants.MAX_VELOCITY, PivotConstants.MAX_VELOCITY);
				if (targetSpeed != 0) {
					setTarget(target + (targetSpeed / 50)); // divide the speed by 50 because their are 50 loops per second
					isJoystickCentered = false;
				} else if (!isJoystickCentered) {
					setTarget(absoluteEncoder.getPosition());
					currentSetpoint = new TrapezoidProfile.State(absoluteEncoder.getPosition(), 0);
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
	 * Toggles brake mode on the pivot motor.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command toggleBrakeModesCommand() {
		return this.runOnce(() -> {
			SparkBaseConfig newPivotConfig = new SparkMaxConfig();
			if (motor.configAccessor.getIdleMode() == IdleMode.kBrake) {
				newPivotConfig.idleMode(IdleMode.kCoast);
			} else {
				newPivotConfig.idleMode(IdleMode.kBrake);
			}
			motor.configure(newPivotConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
		}).ignoringDisable(true);
	}
}
