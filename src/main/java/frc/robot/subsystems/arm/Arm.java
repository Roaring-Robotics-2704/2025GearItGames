// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Arm extends SubsystemBase {
	/** Creates a new Arm. */
	ArmIO armIO;
	private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
	Angle setpoint = Degrees.of(0);
	private SysIdRoutine routine;
	public Arm(ArmIO armIO) {
		this.armIO = armIO;
		routine = new SysIdRoutine(
			new SysIdRoutine.Config(Volts.of(1).per(Second),Volts.of(7),Seconds.of(10), state -> Logger.recordOutput("Arm/SysIDState", state.toString())),
			new Mechanism(armIO::setVoltage, null, this));

	
	}

	@Override
	public void periodic() {
		armIO.updateInputs(inputs);
		Logger.processInputs("Arm", inputs); // Send input data to the logging framework (or update from the log during replay)
		// This method will be called once per scheduler run
		Logger.recordOutput("Arm/setpoint", setpoint.in(Degrees));
	}

	public void setArmAngle(Angle angle) {
		setpoint = angle;
		armIO.setAngle(angle);
	}

	public boolean atSetpoint() {
		return inputs.atSetpoint;
	}
	public Command setVoltage(Voltage voltage) {
		return Commands.run(() -> armIO.setVoltage(voltage));
	}
	// public Command quasistatic(Direction direction) {
	// 	return routine.quasistatic(direction);
	// }
	// public Command dynamic(Direction direction) {
	// 	return routine.dynamic(direction);
	// }
}
