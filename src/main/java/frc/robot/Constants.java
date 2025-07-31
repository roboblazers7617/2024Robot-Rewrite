// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.Filesystem;
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
		public static final double MAX_SPEED = FeetPerSecond.of(14.5).in(MetersPerSecond);
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
		 * Enables {@link swervelib.SwerveDrive#headingCorrection heading correction}. Should only be used while controlling the robot via angle.
		 */
		public static final boolean ENABLE_HEADING_CORRECTION = false;
		/**
		 * Enables {@link swervelib.parser.SwerveModuleConfiguration#useCosineCompensator cosine compensation}.
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
			 * Delay in seconds between each section. This time allows for things to settle (allow motors to spin down, etc.).
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
	 * During a competition debug mode should be false to reduce network and CPU usage. All data will still be logged it just won't be accessible until after the match.
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
		 * The {@link io.github.roboblazers7617.limelight.LimelightSettings#withArilTagIdFilter(List)} to use on the blue alliance.
		 */
		public static final List<Double> BLUE_TAG_ID_FILTER = List.of();
		/**
		 * The {@link io.github.roboblazers7617.limelight.LimelightSettings#withArilTagIdFilter(List)} to use on the Red alliance.
		 */
		public static final List<Double> RED_TAG_ID_FILTER = List.of();
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
}
