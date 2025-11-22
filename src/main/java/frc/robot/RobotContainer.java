// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructureConstants.SuperStructureState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOReal;
import frc.robot.subsystems.gripper.GripperIOSim;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Drive drive;
	private final Arm arm;
	private final Gripper gripper;
	// private final SuperStructure superStructure;

	// Controller
	private final CommandXboxController controller = new CommandXboxController(0);

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		switch (Constants.currentMode) {
			case REAL:
				// Real robot, instantiate hardware IO implementations
				drive = new Drive(new DriveIOTalonSRX());
				arm = new Arm(new ArmIOReal());
				gripper = new Gripper(new GripperIOReal());
				break;

			case SIM:
				// Sim robot, instantiate physics sim IO implementations
				drive = new Drive(new DriveIOSim());
				arm = new Arm(new ArmIOSim());
				gripper = new Gripper(new GripperIOSim());
				break;

			default:
				// Replayed robot, disable IO implementations
				drive = new Drive(new DriveIO() {
				});
				arm = new Arm(new ArmIO() {
				});
				gripper = new Gripper(new GripperIO() {
				});
				break;
		}
		// superStructure = new SuperStructure(arm, gripper);

		// Set up auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices");
		autoChooser.addDefaultOption("Drive forward 3 seconds", DriveCommands.driveForward(drive, Seconds.of(3)).withName("Drive Forward 3 Seconds"));
		autoChooser.addOption("None", Commands.none().withName("If this is running, we were either too rushed to add any autos or for some reason the driver didn't want to run one"));



		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick}
	 * or {@link XboxController}), and then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Default command, normal arcade drive
		drive.setDefaultCommand(
				DriveCommands.arcadeDrive(
						drive, controller::getLeftY, controller::getRightX).withName("Arcade Drive"));
		controller.y().whileTrue(arm.setVoltage(Volts.of(12)));
		controller.a().whileTrue(arm.setVoltage(Volts.of(-4)));
		controller.a().negate().and(controller.y().negate()).whileTrue(arm.setVoltage(Volts.of(3.5)));
		controller.leftTrigger().whileTrue(gripper.setGripper(false));
		controller.rightTrigger().whileTrue(gripper.setGripper(true));
		// controller.leftTrigger().whileTrue(
		// 		superStructure.gotoState(
		// 				SuperStructureState.INTAKE_UPRIGHT).withName("Driver Intake Upright"))
		// 		.onFalse(
		// 				Commands.sequence(
		// 						superStructure.gotoState(
		// 								SuperStructureState.INTAKE_UPRIGHT_GRAB).withName("Driver Intake Upright Grab"),
		// 						superStructure.gotoState(SuperStructureState.HOLDING).withName("Driver Holding")));
		// controller.rightTrigger().whileTrue(
		// 		Commands.sequence(
		// 				superStructure.gotoState(SuperStructureState.OUTTAKING_HOLD).withName("Driver Outtaking Hold"),
		// 				superStructure.gotoState(SuperStructureState.OUTTAKING).withName("Driver Outtaking"),
		// 				superStructure.gotoState(SuperStructureState.DEFAULT_STATE).withName("Driver Default State")));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
