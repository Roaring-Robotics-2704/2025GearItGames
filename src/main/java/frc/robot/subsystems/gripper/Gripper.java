// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.GRIPPER_CLOSE_VOLTAGE;
import static frc.robot.subsystems.gripper.GripperConstants.GRIPPER_OPEN_VOLTAGE;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.gripper.GripperIO.GripperIOInputs;

public class Gripper extends SubsystemBase {
	
	GripperIO gripperIO;
	
	private GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

	/** Creates a new Gripper. */
	public Gripper(GripperIO io) {
		this.gripperIO = io;
	}

	@Override
	public void periodic() {
		gripperIO.updateInputs(inputs);
		Logger.processInputs("Gripper", inputs); // Send input data to the logging framework (or update from the log during replay)
		// This method will be called once per scheduler run
	}
	public boolean atSetpoint() {
		return gripperIO.atSetpoint();
	}
	public void setVoltage(Voltage voltage) {
		gripperIO.setVoltage(voltage);
	}
	public void setGripperClosed(boolean closed) {
		if (closed) {
			setVoltage(GRIPPER_CLOSE_VOLTAGE);
		} else {
			setVoltage(GRIPPER_OPEN_VOLTAGE);
		}
	}
}
