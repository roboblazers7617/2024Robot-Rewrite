// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.mechanisms.arm.ArmPosition;
import io.github.roboblazers7617.limelight.LimelightSettings.ImuMode;
import io.github.roboblazers7617.limelight.PoseEstimator.PoseEstimators;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	/**
	 * Constants used by the {@link frc.robot.subsystems.drivetrain.Drivetrain}.
	 */
	public static class DrivetrainConstants {
		/**
		 * Maximum speed of the robot in meters per second.
		 */
		public static final double MAX_SPEED = FeetPerSecond.of(5).in(MetersPerSecond);
		/**
		 * Directory that contains the YAGSL configuration.
		 */
		public static final File CONFIG_DIR = new File(Filesystem.getDeployDirectory(), "swerve");
		/**
		 * YAGSL telemetry verbosity when in debug mode.
		 */
		public static final TelemetryVerbosity TELEMETRY_VERBOSITY_DEBUG = TelemetryVerbosity.HIGH;
		/**
		 * YAGSL telemetry verbosity when in normal mode.
		 */
		public static final TelemetryVerbosity TELEMETRY_VERBOSITY_NORMAL = TelemetryVerbosity.POSE;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in fast mode.
		 */
		public static final double TRANSLATION_SCALE_FAST = 1;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in normal mode.
		 */
		public static final double TRANSLATION_SCALE_NORMAL = 0.8;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in slow mode.
		 */
		public static final double TRANSLATION_SCALE_SLOW = 0.3;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain when in slide mode.
		 */
		public static final double TRANSLATION_SCALE_SLIDE = 0.3;
		/**
		 * Starting pose.
		 */
		public static final Pose2d STARTING_POSITION = new Pose2d(new Translation2d(Meters.of(1), Meters.of(4)), Rotation2d.fromDegrees(0));
		/**
		 * Enables {@link swervelib.SwerveDrive#headingCorrection heading correction}. Should only be used
		 * while controlling the robot via angle.
		 */
		public static final boolean ENABLE_HEADING_CORRECTION = false;
		/**
		 * Enables {@link swervelib.parser.SwerveModuleConfiguration#useCosineCompensator cosine
		 * compensation}.
		 */
		public static final boolean ENABLE_COSINE_COMPENSATION = false;

		/**
		 * Angular velocity skew correction configuration.
		 *
		 * @see swervelib.SwerveDrive#setAngularVelocityCompensation
		 */
		public static final class AngularVelocityCompensation {
			/**
			 * Enables angular velocity correction in teleop.
			 */
			public static final boolean USE_IN_TELEOP = true;
			/**
			 * Enables angular velocity correction in autonomous.
			 */
			public static final boolean USE_IN_AUTO = true;
			/**
			 * The angular velocity coefficient.
			 */
			public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.1;
		}

		/**
		 * Configure auto synchronization for encoders during a match.
		 *
		 * @see swervelib.SwerveDrive#setModuleEncoderAutoSynchronize
		 */
		public static final class EncoderAutoSynchronization {
			/**
			 * Enable auto synchronization.
			 */
			public static final boolean ENABLED = false;
			/**
			 * Deadband in degrees.
			 */
			public static final double DEADBAND = 1;
		}

		/**
		 * Configure pathfinding to poses.
		 */
		public static final class Pathfinding {
			/**
			 * Maximium linear acceleration.
			 */
			public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(2.0);
			/**
			 * Maximum angular acceleration.
			 */
			public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(720);
		}

		/**
		 * SysId configuration.
		 */
		public static final class SysId {
			/**
			 * The maximum voltage to apply to the drive motors.
			 */
			public static final double MAX_VOLTS = 12.0;
			/**
			 * Delay in seconds between each section. This time allows for things to settle (allow motors to
			 * spin down, etc.).
			 */
			public static final double DELAY = 3.0;
			/**
			 * Time in seconds to run Quasistatic routines. This prevents the robot from going too far.
			 */
			public static final double QUASI_TIMEOUT = 5.0;
			/**
			 * Time in seconds to run Dynamic routines.
			 */
			public static final double DYNAMIC_TIMEOUT = 3.0;
			/**
			 * Spin in place instead of driving forward.
			 */
			public static final boolean TEST_WITH_SPINNING = false;
		}
	}

	/**
	 * Constants used to configure the operator controllers.
	 */
	public static class OperatorConstants {
		/**
		 * Controller port index where the driver controller is connected.
		 */
		public static final int DRIVER_CONTROLLER_PORT = 0;
		/**
		 * Controller port index where the operator controller is connected.
		 */
		public static final int OPERATOR_CONTROLLER_PORT = 1;
		/**
		 * Joystick deadband.
		 */
		public static final double DEADBAND = 0.1;
	}

	/**
	 * Constants used to configure the autonomous program.
	 */
	public static class AutoConstants {
		/**
		 * PID constants used for translation.
		 */
		public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
		/**
		 * PID constants used for rotation.
		 */
		public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
	}

	/**
	 * Constants used to configure logging.
	 * <p>
	 * During a competition debug mode should be false to reduce network and CPU usage. All data will
	 * still be logged it just won't be accessible until after the match.
	 * <p>
	 * During testing debug mode should be true to allow for real-time data viewing.
	 */
	public static class LoggingConstants {
		/**
		 * Send logging data to NetworkTables. Data is written to storage when set to false.
		 */
		public static final boolean DEBUG_MODE = true;
		/**
		 * Log all data above specified level.
		 */
		public static final Logged.Importance DEBUG_LEVEL = Logged.Importance.DEBUG;
	}

	/**
	 * Constants used to configure the driver dashboard.
	 */
	public static class DashboardConstants {
		/**
		 * The name of the tab used in Auto.
		 */
		public static final String AUTO_TAB_NAME = "Autonomous";
		/**
		 * The name of the tab used in Teleop.
		 */
		public static final String TELEOP_TAB_NAME = "Teleoperated";
	}

	/**
	 * Constants used to configure vision.
	 */
	public static class VisionConstants {
		/**
		 * The name of the front Limelight on NetworkTables.
		 */
		public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
		/**
		 * The name of the back Limelight on NetworkTables.
		 */
		public static final String BACK_LIMELIGHT_NAME = "limelight-back";
		/**
		 * Enable vision odometry updates.
		 */
		public static final boolean ENABLE_VISION = true;
		/**
		 * Use MegaTag2 for pose estimation.
		 */
		public static final PoseEstimators POSE_ESTIMATOR_TYPE = PoseEstimators.BLUE_MEGATAG2;
		/**
		 * The frequency of processed vision frames while disabled.
		 */
		public static final int DISABLED_UPDATE_FREQUENCY = 60;
		/**
		 * The {@link ImuMode} to use while disabled.
		 */
		public static final ImuMode DISABLED_IMU_MODE = ImuMode.SyncInternalImu;
		/**
		 * The {@link ImuMode} to use while enabled.
		 */
		public static final ImuMode ENABLED_IMU_MODE = ImuMode.ExternalAssistInternalIMU;
		/**
		 * The AprilTag IDs to use on the blue alliance.
		 */
		public static final List<Integer> BLUE_TAG_ID_FILTER = List.of();
		/**
		 * The AprilTag IDs to use on the Red alliance.
		 */
		public static final List<Integer> RED_TAG_ID_FILTER = List.of();
	}

	/**
	 * Constants that describe the physical layout of the field.
	 */
	public static class FieldConstants {
		/**
		 * AprilTag Field Layout for the current game.
		 */
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
	}

	/**
	 * Constants used to configure the {@link frc.robot.subsystems.mechanisms.arm.Elevator}.
	 */
	public static class ElevatorConstants {
		/**
		 * CAN ID for the right elevator motor.
		 */
		public static final int RIGHT_MOTOR_ID = 28;
		/**
		 * CAN ID for the left elevator motor.
		 */
		public static final int LEFT_MOTOR_ID = 27;

		// TODO: Tune PID constants
		/**
		 * Elevator kP.
		 */
		public static final double KP = 0.0;
		/**
		 * Elevator kI.
		 */
		public static final double KI = 0.0;
		/**
		 * Elevator kD.
		 */
		public static final double KD = 0.0;
		/**
		 * Elevator kS.
		 */
		public static final double KS = 0.0;
		/**
		 * Elevator kG.
		 */
		public static final double KG = 0.0; // 0.14
		/**
		 * Elevator kV.
		 */
		public static final double KV = 0.0;// 6.9

		/**
		 * Elevator kMinOutput as a percentage.
		 */
		public static final double KMIN_OUTPUT = 0.0;
		/**
		 * Elevator kMaxOutput as a percentage.
		 */
		public static final double KMAX_OUTPUT = 0.0;
		/**
		 * Maximum velocity in m/s.
		 */
		// TODO: Update with accurate number. Is the elevator really going to travel 3 feet in one second?
		// Use reca.lc
		public static final double MAX_VELOCITY = 2;
		/**
		 * Maximum acceleration in m/s^2.
		 */
		// TODO: Update with accurate number Use reca.lc
		public static final double MAX_ACCELERATION = 2.5;

		/**
		 * Minimum position in meters.
		 */
		public static final double MIN_POSITION = 0.0;
		/**
		 * Maximum position in meters.
		 */
		// TODO: Update with accurate number
		public static final double MAX_POSITION = 1.44;
		/**
		 * Position at which the head clears the robot chassis, in meters.
		 */
		// TODO: Update with accurate number
		public static final double HEAD_CLEAR_POSITION = 0.5;
		/**
		 * Conversion factor from rotation to meters. 3.81cm diameter spool, 16:1 gear ratio
		 */
		// TODO: Update with accurate number
		public static final double POSITION_CONVERSION_FACTOR = (1 / .2845) / 200;
		/**
		 * Conversion factor from rotation to meters per second.
		 */
		public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;
		/**
		 * Zero offset, meters.
		 */
		public static final double ZERO_OFFSET = 0;
		/**
		 * Current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 40;
		/**
		 * Tolerance for the target to be considered reached in meters.
		 */
		public static final double TOLERANCE = .02;
	}

	/**
	 * Constants used to configure the pivot.
	 */
	public static class PivotConstants {
		/**
		 * Left motor CAN ID.
		 */
		public static final int LEFT_MOTOR_ID = 26;
		/**
		 * Right motor CAN ID.
		 */
		public static final int RIGHT_MOTOR_ID = 25;

		// TODO: Tune PID constants
		/**
		 * Pivot kP.
		 */
		public static final double KP = 0.0;
		/**
		 * Pivot kI.
		 */
		public static final double KI = 0.0;
		/**
		 * Pivot kD.
		 */
		public static final double KD = 0.0;

		/**
		 * Pivot kS.
		 */
		public static final double KS = 0.0;
		/**
		 * Pivot kG.
		 */
		public static final double KG = 0.0;
		/**
		 * Pivot kV.
		 */
		public static final double KV = 0; // Leave as zero, Max motion will take care of this

		/**
		 * Pivot kMinOutput.
		 */
		public static final double KMIN_OUTPUT = 0.0;
		/**
		 * Pivot kMaxOutput.
		 */
		public static final double KMAX_OUTPUT = 0.0;
		/**
		 * Maximum velocity in degrees/s.
		 */
		// TODO: Update with accurate number
		public static final double MAX_VELOCITY = 200.0;
		/**
		 * Maximum acceleration in degrees/s^2.
		 */
		// TODO: Update with accurate number
		public static final double MAX_ACCELERATION = 200.0;

		/**
		 * Maximum position in degrees.
		 */
		public static final double MAX_POSITION = 90.0;
		/**
		 * Minimum position when the elevator is not lowered, in degrees.
		 */
		public static final double MIN_POSITION = 2.5;
		/**
		 * Minimum safe position while the elevator is below {@link ElevatorConstants#HEAD_CLEAR_POSITION},
		 * in degrees.
		 */
		public static final double SAFE_MIN_POSITION = 18.5;

		/**
		 * Conversion factor from rotation to degrees.
		 */
		public static final double POSITION_CONVERSION_FACTOR = 360;
		/**
		 * Conversion factor from rotation to degrees per second.
		 */
		public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
		/**
		 * Zero offset, in rotations, because reasons.
		 */
		public static final double ZERO_OFFSET = Degrees.of(124.5).in(Rotations);

		/**
		 * Current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 40;

		/**
		 * Tolerance for the target to be considered reached in degrees.
		 */
		public static final double TOLERANCE = 5;
	}

	/**
	 * Constants used by the {@link frc.robot.subsystems.mechanisms.Head}'s intake.
	 */
	public static class IntakeConstants {
		/**
		 * The CAN ID for the intake motor.
		 */
		// TODO: Update
		public static final int MOTOR_ID = 21;

		/**
		 * The DIO pin for the beam break that senses when a note has entered the intake.
		 */
		public static final int NOTE_SENSOR_DIO = 9;
		/**
		 * The DIO pin for the beam break that senses when a note is in the position to shoot.
		 */
		public static final int NOTE_ALIGNMENT_SENSOR_DIO = 6;

		/**
		 * Motor current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 40;

		/**
		 * The speed at which the intake spins to intake notes.
		 */
		public static final double INTAKE_SPEED = 0.75; // 0.95
		/**
		 * The speed at which the intake spins to align notes for shooting.
		 */
		public static final double ALIGMNMENT_SPEED = 0.2; // 0.08
		/**
		 * The speed at which the intake spins to outtake notes.
		 */
		public static final double OUTTAKE_SPEED = -0.25;
		/**
		 * The speed at which the intake spins to feed notes into the shooter.
		 */
		public static final double FEEDER_SPEED = 0.25; // What speed should a note be fed into the shooter at?
	}

	/**
	 * Constants used by the {@link frc.robot.subsystems.mechanisms.Head}'s shooter.
	 */
	public static class ShooterConstants {
		/**
		 * CAN ID for the bottom shooter motor.
		 */
		// TODO: Update
		public static final int MOTOR_ID_BOTTOM = 23;
		/**
		 * CAN ID for the top shooter motor.
		 */
		// TODO: Update
		public static final int MOTOR_ID_TOP = 25;

		/**
		 * Motor current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 20;

		// TODO: Tune these
		/**
		 * Shooter kP.
		 */
		public static final double KP = 0.00026;
		/**
		 * Shooter kI.
		 */
		public static final double KI = 0;
		/**
		 * Shooter kD.
		 */
		public static final double KD = 0.0;
		/**
		 * Shooter kF.
		 */
		public static final double KF = 0.004;
		/**
		 * Shooter kMinOutput.
		 */
		public static final double KMIN_OUTPUT = -1;
		/**
		 * Shooter kMaxOutput.
		 */
		public static final double KMAX_OUTPUT = 1;

		// TODO: Put a real value here
		/**
		 * How close does the shooter have to be to its target velocity before it's considered to be at
		 * speed?
		 */
		public static final double VELOCITY_TOLERANCE = 0;
	}

	/**
	 * Position presets for the Arm.
	 */
	public static class ArmPositions {
		/**
		 * A neutral position with the pivot low and the elevator retracted. The head is completely within
		 * the frame of the robot in this position.
		 */
		public static final ArmPosition STOW = new ArmPosition(Meters.of(0), Degrees.of(20));
		/**
		 * A position used to pick up notes from the floor.
		 */
		public static final ArmPosition INTAKE_FLOOR = new ArmPosition(Meters.of(ElevatorConstants.MAX_POSITION), Degrees.of(3.75));
		/**
		 * A position used to pick up notes from the source.
		 */
		public static final ArmPosition INTAKE_SOURCE = new ArmPosition(Meters.of(ElevatorConstants.MAX_POSITION), Degrees.of(64.5));
	}
}
