// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Shooter and intake.
 */
@Logged
public class Head extends SubsystemBase {
	/**
	 * Bottom shooter motor.
	 */
	private final SparkMax shooterLeaderMotor = new SparkMax(ShooterConstants.MOTOR_ID_BOTTOM, MotorType.kBrushless);
	/**
	 * Top shooter motor.
	 */
	private final SparkMax shooterFollowerMotor = new SparkMax(ShooterConstants.MOTOR_ID_TOP, MotorType.kBrushless);
	/**
	 * Encoder for the bottom shooter motor.
	 */
	private final RelativeEncoder shooterEncoder = shooterLeaderMotor.getEncoder();
	/**
	 * Closed loop controller for the shooter.
	 */
	private final SparkClosedLoopController shooterController = shooterLeaderMotor.getClosedLoopController();

	/**
	 * Motor that handles intaking notes.
	 */
	private final SparkMax intakeMotor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
	/**
	 * Sensor to tell whether the note is aligned to shoot.
	 */
	private final DigitalInput isNoteInShootPosition = new DigitalInput(IntakeConstants.NOTE_SENSOR_DIO);
	/**
	 * Sensor to tell whether the note is in the intake. Used to slow down the intake so the note is
	 * aligned correctly.
	 */
	private final DigitalInput isNoteInIntake = new DigitalInput(IntakeConstants.NOTE_ALIGNMENT_SENSOR_DIO);

	/**
	 * Shooter setpoint in RPM.
	 */
	private double shooterSetPoint = 0;

	/**
	 * Creates a new Head.
	 */
	public Head() {
		// Shooter motor config
		SparkBaseConfig baseShooterConfig = new SparkMaxConfig()
				.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
				.idleMode(IdleMode.kCoast);

		baseShooterConfig.closedLoop
				.p(ShooterConstants.KP)
				.i(ShooterConstants.KI)
				.d(ShooterConstants.KD)
				.outputRange(ShooterConstants.KMIN_OUTPUT, ShooterConstants.KMAX_OUTPUT);

		shooterLeaderMotor.configure(baseShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkBaseConfig followerShooterConfig = new SparkMaxConfig()
				.apply(baseShooterConfig)
				.follow(shooterLeaderMotor, false);
		shooterFollowerMotor.configure(followerShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Intake motor
		SparkBaseConfig intakeMotorConfig = new SparkMaxConfig()
				.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT)
				.idleMode(IdleMode.kBrake)
				.inverted(true);

		intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/**
	 * Sets the intake speed.
	 *
	 * @param intakeSpeed
	 *            New intake speed [-1.0,1.0]
	 */
	private void setIntakeSpeed(double intakeSpeed) {
		intakeMotor.set(intakeSpeed);
	}

	/**
	 * Starts outtaking.
	 *
	 * @return
	 *         {@link Command} to run
	 */
	public Command startOuttakeCommand() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.OUTTAKE_SPEED);
		}, this);
	}

	/**
	 * Stops the intake.
	 *
	 * @return
	 *         {@link Command} to run
	 */
	public Command stopIntakeCommand() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(0);
		}, this);
	}

	/**
	 * Intakes a piece and stops.
	 *
	 * @return
	 *         {@link Command} to run
	 */
	public Command intakePieceCommand() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> isNoteWithinAlignmentSensor()))
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeed(IntakeConstants.ALIGMNMENT_SPEED);
				}))
				.andThen(Commands.waitUntil(() -> isNoteWithinShootingSensor()))
				.andThen(() -> {
					setIntakeSpeed(0);
				});
	}

	/**
	 * Outtakes a piece and stops.
	 *
	 * @return
	 *         {@link Command} to run
	 */
	public Command outtakePieceCommand() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.OUTTAKE_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> !isNoteWithinShootingSensor()))
				.andThen(Commands.waitSeconds(3))
				.finallyDo(() -> {
					setIntakeSpeed(0);
				});
	}

	/**
	 * Sets the shooter speed.
	 *
	 * @param rpm
	 *            new speed in RPM
	 */
	private void setShooterSpeed(double rpm) {
		shooterSetPoint = rpm;
		shooterController.setReference(shooterSetPoint, ControlType.kVelocity);
	}

	/**
	 * Stops the shooter.
	 *
	 * @implNote
	 *           This removes power from the motors, allowing them to spin down freely and reducing the
	 *           shock on the mechanism, so the shooter will take some time to spin down after calling
	 *           this method.
	 */
	public void spinDownShooter() {
		shooterSetPoint = 0.0;
		shooterLeaderMotor.setVoltage(0);
		shooterFollowerMotor.setVoltage(0);
	}

	/**
	 * Gets the speed of the bottom shooter motor.
	 *
	 * @return
	 *         Motor velocity in RPM
	 */
	public double getShooterSpeed() {
		return shooterEncoder.getVelocity();
	}

	/**
	 * Gets the current setpoint of the shooter.
	 *
	 * @return
	 *         Setpoint in RPM
	 */
	public double getShooterSetPoint() {
		return shooterSetPoint;
	}

	/**
	 * Spins up the shooter to a given speed.
	 *
	 * @param rpm
	 *            New speed in RPM
	 * @return
	 *         {@link Command} to run
	 */
	public Command spinUpShooterCommand(double rpm) {
		return Commands.runOnce(() -> {
			setShooterSpeed(rpm);
		}, this);
	}

	/**
	 * Spins down the shooter.
	 *
	 * @return
	 *         command to run
	 */
	public Command spinDownShooterCommand() {
		return Commands.runOnce(() -> {
			spinDownShooter();
		}, this);
	}

	/**
	 * Checks if the shooter speed is within the range to shoot.
	 *
	 * @return
	 *         Is the shooter ready to shoot?
	 */
	public boolean isReadyToShoot() {
		return Math.abs(getShooterSpeed() - shooterSetPoint) < ShooterConstants.VELOCITY_TOLERANCE;
	}

	/**
	 * Shoots.
	 *
	 * @param stopShooter
	 *            If true, shooter is spun down after shoot
	 * @return
	 *         {@link Command} to run
	 */
	public Command shootCommand(boolean stopShooter) {
		return Commands.waitUntil(() -> isReadyToShoot())
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeed(IntakeConstants.FEEDER_SPEED);
				}))
				.andThen(Commands.waitUntil(() -> isNoteWithinShootingSensor()))
				.andThen(Commands.waitUntil(() -> !isNoteWithinShootingSensor()))
				.andThen(Commands.waitSeconds(0.2))
				.andThen(Commands.either(spinDownShooterCommand().andThen(() -> setIntakeSpeed(0.0)), Commands.none(), () -> stopShooter));
	}

	/**
	 * Gets the value of the alignment sensor.
	 *
	 * @return
	 *         Is the note in the alignment sensor?
	 */
	public boolean isNoteWithinAlignmentSensor() {
		return !isNoteInIntake.get();
	}

	/**
	 * Gets the value of the shooting position sensor.
	 *
	 * @return
	 *         Is the note in the position to shoot?
	 */
	public boolean isNoteWithinShootingSensor() {
		return !isNoteInShootPosition.get();
	}

	/**
	 * Toggles brake mode on the intake motor.
	 *
	 * @return
	 *         {@link Command} to run
	 */
	public Command toggleBrakeModeCommand() {
		return new InstantCommand(() -> {
			SparkBaseConfig newConfig = new SparkMaxConfig();
			if (intakeMotor.configAccessor.getIdleMode() == IdleMode.kBrake) {
				newConfig.idleMode(IdleMode.kCoast);
			} else {
				newConfig.idleMode(IdleMode.kBrake);
			}
			intakeMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		}).ignoringDisable(true);
	}

	/**
	 * Enables brake mode on the intake motor.
	 *
	 * @return
	 *         {@link Command} to run
	 */
	public Command enableBrakeModeCommand() {
		return new InstantCommand(() -> {
			SparkBaseConfig newConfig = new SparkMaxConfig()
					.idleMode(IdleMode.kBrake);
			intakeMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		});
	}
}
