// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.arm.ArmConstants.PID;

import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR_CAN_ID;
import static frc.robot.subsystems.arm.ArmConstants.ARM_ENCODER_PORT;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.arm.ArmConstants.ARM_CONSTRAINTS;
import static frc.robot.subsystems.arm.ArmConstants.PID;
import static frc.robot.subsystems.drive.DriveConstants.currentLimit;

/** Add your docs here. */
public class ArmIOReal implements ArmIO {
	TalonSRX armMotor;
	AnalogEncoder armEncoder;
	Encoder armRelativeEncoder;
	ProfiledPIDController armController;
	Angle previousPosition = Degrees.zero();
	Angle currentPosition = Degrees.zero();
	Angle targetPosition = Degrees.zero();

	public ArmIOReal() {
		armMotor = new TalonSRX(ARM_MOTOR_CAN_ID);
		armEncoder = new AnalogEncoder(ARM_ENCODER_PORT);
		armRelativeEncoder = new Encoder(ARM_ENCODER_PORT + 1, ARM_ENCODER_PORT + 2);
		armController = new ProfiledPIDController(PID.ARM_P, PID.ARM_I, PID.ARM_D, ARM_CONSTRAINTS);
		armMotor.configFactoryDefault();
		armMotor.enableVoltageCompensation(true);

	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		inputs.Position.mut_replace(Rotations.of(armEncoder.get()));

		currentPosition = Rotations.of(armEncoder.get());
		inputs.Velocity.mut_replace(RotationsPerSecond.of(armRelativeEncoder.getRate()));
		previousPosition = currentPosition;

		inputs.Voltage.mut_replace(Volts.of(armMotor.getMotorOutputVoltage()));
		inputs.Current.mut_replace(Amps.of(armMotor.getStatorCurrent()));
	}

	@Override
	public void setVoltage(Voltage voltage) {
		armMotor.set(ControlMode.PercentOutput, voltage.in(Volts) / 12.0);
	}

	@Override
	public void setAngle(Angle angle) {
		targetPosition = angle;
		armController.setGoal(targetPosition.in(Degrees));
		double output = armController.calculate(armEncoder.get());
		armMotor.set(ControlMode.PercentOutput, output);
	}

	@Override
	public boolean atSetpoint() {
		return MathUtil.isNear(targetPosition.in(Degrees), currentPosition.in(Degrees),
				ArmConstants.tolerance.in(Degrees));
	}

}
