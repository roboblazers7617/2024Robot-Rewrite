package frc.robot.subsystems.mechanisms.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPositions;
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
	 * The ArmPosition that the Arm is currently trying to get to.
	 */
	private ArmPosition targetPosition = ArmPositions.STOW;

	/**
	 * The gearbox on the pivot.
	 */
	public final DCMotor pivotGearbox = DCMotor.getNEO(1);
	/**
	 * Simulation object for the arm.
	 */
	private final SingleJointedArmSim armSim = new SingleJointedArmSim(pivotGearbox, PivotConstants.POSITION_CONVERSION_FACTOR, SingleJointedArmSim.estimateMOI(ElevatorConstants.MAX_POSITION, PivotConstants.ARM_WEIGHT), ElevatorConstants.MAX_POSITION, Degrees.of(PivotConstants.MIN_POSITION).in(Radians), Degrees.of(PivotConstants.MAX_POSITION).in(Radians), true, 0);
	/**
	 * Simulation object for the pivot motor.
	 */
	public final SparkMaxSim sim;
	/**
	 * Mechanism2d that represents the state of the arm.
	 */
	private final Mechanism2d mechanism;
	/**
	 * Mechanism Ligament that represents the elevator.
	 */
	private final MechanismLigament2d mechanismElevator;

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

		// Set up simulation
		sim = new SparkMaxSim(pivot.motor, pivotGearbox);

		// Set up the Mechanism2d
		mechanism = new Mechanism2d(3, 3);
		// The mechanism root node
		MechanismRoot2d root = mechanism.getRoot("climber", 2, 2);

		// MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
		// off the root node or another ligament object
		mechanismElevator = root.append(new MechanismLigament2d("elevator", 0.5, 90));

		// Add a brake toggle button to SmartDashboard.
		SmartDashboard.putData("Toggle Arm Brake Mode", toggleBrakeModesCommand());

		// Put the mechanism on SmartDashboard.
		SmartDashboard.putData("Arm Sim", mechanism);
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
		elevator.setTarget(targetPosition.getElevatorPosition());
		pivot.setTarget(targetPosition.getPivotAngle());

		// If the elevator target or position is too low for the pivot target, set the pivot higher so the
		// head can clear the body of the robot
		boolean elevatorSafe = (elevator.getPosition() < ElevatorConstants.HEAD_CLEAR_POSITION) && (elevator.getTarget() < ElevatorConstants.HEAD_CLEAR_POSITION);
		if (elevatorSafe && pivot.getTarget() < PivotConstants.SAFE_MIN_POSITION) {
			pivot.setTarget(PivotConstants.SAFE_MIN_POSITION);
		}

		// If the pivot target or position is too low for the elevator target, set the elevator higher so
		// the head can clear the body of the robot
		boolean pivotSafe = (pivot.getPosition() < PivotConstants.SAFE_MIN_POSITION) && (pivot.getTarget() < PivotConstants.SAFE_MIN_POSITION);
		if (pivotSafe && elevator.getTarget() < ElevatorConstants.HEAD_CLEAR_POSITION) {
			elevator.setTarget(ElevatorConstants.HEAD_CLEAR_POSITION);
		}

		mechanismElevator.setAngle(90 - pivot.getPosition());
		// mechanismElevator.setLength(elevator.getPosition());

		System.out.println(pivot.getPosition() + ", " + RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RPM));
	}

	/**
	 * Periodic method used in simulation.
	 */
	@Override
	public void simulationPeriodic() {
		// In this method, we update our simulation of what our arm is doing
		// First, we set our "inputs" (voltages)
		armSim.setInput(pivot.motor.getAppliedOutput() * RobotController.getBatteryVoltage());

		// Next, we update it. The standard loop time is 20ms.
		armSim.update(0.020);

		// SimBattery estimates loaded battery voltages
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

		// Update the Mechanism Arm angle based on the simulated arm angle
		sim.iterate(RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RPM), RoboRioSim.getVInVoltage(), 0.020);
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
				targetPosition = position;
			}

			@Override
			public boolean isFinished() {
				return isAtTarget();
			}

			@Override
			public void end(boolean interrupted) {
				System.out.println("Arm move done. Interrupted?: " + interrupted);
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
