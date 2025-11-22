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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

/**
 * This drive implementation is for Spark devices. It defaults to brushless
 * control, but can be
 * easily adapted for brushed motors and external encoders. Spark Flexes can be
 * used by swapping all
 * instances of "SparkMax" with "SparkFlex".
 */
public class DriveIOSpark implements DriveIO {
	private final SparkMax leftLeader = new SparkMax(leftLeaderCanId, MotorType.kBrushless);
	private final SparkMax rightLeader = new SparkMax(rightLeaderCanId, MotorType.kBrushless);
	private final SparkClosedLoopController leftController = leftLeader.getClosedLoopController();
	private final SparkClosedLoopController rightController = rightLeader.getClosedLoopController();

	public DriveIOSpark() {
		// Create config
		var config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).smartCurrentLimit((int) currentLimit.in(Amps) /*this cast to an int might not be the safest idea, but it fixes a type mismatch*/).voltageCompensation(12.0);
		config.closedLoop.pidf(REAL_D, 0.0, REAL_D, 0.0);
		config.encoder
				.positionConversionFactor(2 * Math.PI / motorReduction) // Rotor Rotations -> Wheel Radians
				.velocityConversionFactor(
						(2 * Math.PI) / 60.0 / motorReduction) // Rotor RPM -> Wheel Rad/Sec
				.uvwMeasurementPeriod(10)
				.uvwAverageDepth(2);

		// Apply config to leaders
		config.inverted(leftInverted);
		tryUntilOk(
				leftLeader,
				5,
				() -> leftLeader.configure(
						config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
		config.inverted(rightInverted);
		tryUntilOk(
				rightLeader,
				5,
				() -> rightLeader.configure(
						config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
	}

	@Override
	public void updateInputs(DriveIOInputs inputs) {
		ifOk(
				leftLeader,
				new DoubleSupplier[] { leftLeader::getAppliedOutput, leftLeader::getBusVoltage },
				values -> inputs.leftAppliedVoltage.mut_replace(Volts.of(values[0] * values[1])));
		ifOk(leftLeader, leftLeader::getOutputCurrent, value -> inputs.leftCurrent.mut_replace(Amps.of(value)));

		ifOk(
				rightLeader,
				new DoubleSupplier[] { rightLeader::getAppliedOutput, rightLeader::getBusVoltage },
				values -> inputs.rightAppliedVoltage.mut_replace(Volts.of(values[0] * values[1])));
		ifOk(rightLeader, rightLeader::getOutputCurrent, value -> inputs.rightCurrent.mut_replace(Amps.of(value)));
	}

	@Override
	public void setVoltage(Voltage leftVoltage, Voltage rightVoltage) {
		leftLeader.setVoltage(leftVoltage.in(Volts));
		rightLeader.setVoltage(rightVoltage.in(Volts));
	}

	@Override
	public void setVelocity(
			AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity, Voltage leftFFVoltage,
			Voltage rightFFVoltage) {
		leftController.setReference(
				leftAngularVelocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0,
				leftFFVoltage.in(Volts));
		rightController.setReference(
				rightAngularVelocity.in(RadiansPerSecond), ControlType.kVelocity, ClosedLoopSlot.kSlot0,
				rightFFVoltage.in(Volts));
	}
}
