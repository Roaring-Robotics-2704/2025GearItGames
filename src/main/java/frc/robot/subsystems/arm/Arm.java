// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	/** Creates a new Arm. */
	ArmIO armIO;
	private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();


	public Arm(ArmIO armIO) {
		this.armIO = armIO;
	}

	@Override
	public void periodic() {
		armIO.updateInputs(inputs);
		Logger.processInputs("Arm", inputs); // Send input data to the logging framework (or update from the log during replay)
		// This method will be called once per scheduler run
	}

	public void setArmAngle(Angle angle) {
		armIO.setAngle(angle);
	}

	public boolean atSetpoint() {
		return armIO.atSetpoint();
	}
}
