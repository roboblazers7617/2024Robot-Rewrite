// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.Elastic;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainControls;
import frc.robot.subsystems.drivetrain.Drivetrain.TranslationOrientation;
import frc.robot.subsystems.Auto;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
	/**
	 * The sendable chooser for the autonomous command. This is added in the setAutoChooser method which is run when autobuilder is created after an alliance is selected.
	 */
	private SendableChooser<Command> autoChooser;

	// The robot's subsystems and commands are defined here...
	@NotLogged
	private final Drivetrain drivetrain = new Drivetrain(DrivetrainConstants.CONFIG_DIR);
	@NotLogged
	private final DrivetrainControls drivetrainControls = new DrivetrainControls(drivetrain);

	/**
	 * The Controller used by the Driver of the robot, primarily controlling the drivetrain.
	 */
	@NotLogged
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	/**
	 * The Controller used by the Operator of the robot, primarily controlling the superstructure.
	 */
	@NotLogged
	private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Publish version metadata
		VersionConstants.publishNetworkTables(NetworkTableInstance.getDefault().getTable("/Metadata"));

		// Configure the trigger bindings
		configureNamedCommands();
		configureDriverControls();
		configureOperatorControls();
	}

	/**
	 * This method is run at the start of Auto.
	 */
	public void autoInit() {
		// Set the Elastic tab
		if (!LoggingConstants.DEBUG_MODE) {
			Elastic.selectTab(DashboardConstants.AUTO_TAB_NAME);
		}

		// Configure AutoBuilder if not already configured
		Auto.setupPathPlannerFailsafe(drivetrain);
	}

	/**
	 * This method is run at the start of Teleop.
	 */
	public void teleopInit() {
		// Reset the last angle so the robot doesn't try to spin.
		drivetrain.resetLastAngleScalar();

		// Set the Elastic tab
		if (!LoggingConstants.DEBUG_MODE) {
			Elastic.selectTab(DashboardConstants.TELEOP_TAB_NAME);
		}
		// if (StubbedCommands.EndEffector.isHoldingAlage()) {
		// gamepieceMode = GamepieceMode.ALGAE_MODE;
		// }

		// else {
		// gamepieceMode = GamepieceMode.CORAL_MODE;
		// }

		// Configure AutoBuilder if not already configured
		Auto.setupPathPlannerFailsafe(drivetrain);
	}

	/**
	 * Sets up the {@link NamedCommands} used by the autonomous routine.
	 */
	private void configureNamedCommands() {}

	/**
	 * Configures {@link Trigger Triggers} to bind Commands to the Driver Controller buttons.
	 */
	private void configureDriverControls() {
		// Set the default drivetrain command (used for the driver controller)
		if (RobotBase.isSimulation()) {
			// Heading control
			drivetrain.setDefaultCommand(drivetrainControls.driveDirectAngleSimCommand(driverController, TranslationOrientation.FIELD_RELATIVE));
		} else {
			// Heading control
			drivetrain.setDefaultCommand(drivetrainControls.driveDirectAngleCommand(driverController, TranslationOrientation.FIELD_RELATIVE));
			// Angular velocity control
			driverController.leftBumper()
					.whileTrue(drivetrainControls.driveAngularVelocityCommand(driverController, TranslationOrientation.FIELD_RELATIVE));
		}

		driverController.b().whileTrue(drivetrainControls.setSpeedMultiplierCommand(() -> DrivetrainConstants.TRANSLATION_SCALE_FAST));
		driverController.x().whileTrue(drivetrain.lockCommand());

		driverController.rightBumper().whileTrue(drivetrainControls.setSpeedMultiplierCommand(() -> DrivetrainConstants.TRANSLATION_SCALE_SLOW));

		driverController.start().onTrue(drivetrain.zeroGyroWithAllianceCommand());
	}

	/**
	 * Configures {@link Triggers} to bind Commands to the Operator Controller buttons.
	 */
	private void configureOperatorControls() {}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// resetLastAngleScalar stops the robot from trying to turn back to its original angle after the auto ends
		if (autoChooser == null) {
			return Commands.runOnce(() -> System.out.println("Auto builder not made! Is the alliance set?"));
		}
		return autoChooser.getSelected()
				.finallyDo(drivetrain::resetLastAngleScalar);
	}

	/**
	 * Set the auto chooser
	 *
	 * @param auto
	 *            a sendable chooser with Commands for the autos
	 */
	public void setAutoChooser(SendableChooser<Command> auto) {
		autoChooser = auto;
	}
}
