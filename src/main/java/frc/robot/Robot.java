// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LoggingConstants;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
	/**
	 * Command that contains the autonomous routine. Set and run at the start of
	 * {@link #autonomousInit()}.
	 */
	private Command autonomousCommand;
	/**
	 * Class that contains most of the robot initialization and control logic.
	 */
	private final RobotContainer robotContainer;

	@Logged
	private final PowerDistribution pdh = new PowerDistribution();

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());
		Epilogue.configure(config -> {
			// if not in debug mode write data to a file
			if (!LoggingConstants.DEBUG_MODE) {
				config.backend = new FileBackend(DataLogManager.getLog());
			}

			config.root = "Telemetry";
			config.minimumImportance = LoggingConstants.DEBUG_LEVEL;
		});
		Epilogue.bind(this);

		// Start a WebServer that hosts the deploy directory
		// This is used for the Elastic layout
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

		// Instantiate our RobotContainer. This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		robotContainer = new RobotContainer();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods. This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		robotContainer.init();
		robotContainer.autoInit();

		autonomousCommand = robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		robotContainer.init();
		robotContainer.teleopInit();

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		robotContainer.init();

		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {}

	/**
	 * This function is called once when the robot is first started up.
	 */
	@Override
	public void simulationInit() {}

	/**
	 * This function is called periodically whilst in simulation.
	 */
	@Override
	public void simulationPeriodic() {}
}
