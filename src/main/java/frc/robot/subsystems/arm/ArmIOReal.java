// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.arm.ArmConstants.PID;

import static frc.robot.subsystems.arm.ArmConstants.ARM_MOTOR_CAN_ID;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.arm.ArmConstants.ARM_CONSTRAINTS;
import static frc.robot.subsystems.arm.ArmConstants.PID;
import static frc.robot.subsystems.arm.ArmConstants.kArmEncoderDistPerPulse;
import static frc.robot.subsystems.arm.ArmConstants.tolerance;
import static frc.robot.subsystems.drive.DriveConstants.currentLimit;

import org.littletonrobotics.junction.Logger;


/** Add your docs here. */
public class ArmIOReal implements ArmIO {
	TalonSRX armMotor;
	// AnalogEncoder armEncoder;
	Encoder armRelativeEncoder;
	
	ProfiledPIDController armController;
	Angle previousPosition = Degrees.zero();
	Angle currentPosition = Degrees.zero();
	Angle targetPosition = Degrees.zero();
	ArmFeedforward feedforward;

	public ArmIOReal() {
		armMotor = new TalonSRX(ARM_MOTOR_CAN_ID);

		// armEncoder = new AnalogEncoder(6);
		armRelativeEncoder = new Encoder(9, 8);
		armController = new ProfiledPIDController(PID.ARM_P, PID.ARM_I, PID.ARM_D, ArmConstants.ARM_CONSTRAINTS);
		feedforward = new ArmFeedforward(PID.ARM_KS, PID.ARM_KG, PID.ARM_KV);
		armMotor.configFactoryDefault();
		TalonSRXConfiguration config = new TalonSRXConfiguration();
		config.continuousCurrentLimit = 35;
		config.peakCurrentLimit = 40;
		armMotor.enableVoltageCompensation(true);
		armController.setTolerance(tolerance.in(Rotations));
		SmartDashboard.putData(armController);
		armMotor.setInverted(true);
		armController.setTolerance(tolerance.in(Rotations));
		armController.enableContinuousInput(-1, 1);
		// armRelativeEncoder.setDistancePerPulse(1/8192);
		armRelativeEncoder.setSamplesToAverage(5);
		armRelativeEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);

	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		inputs.Position.mut_replace(Rotations.of(armRelativeEncoder.getDistance()));

		currentPosition = Rotations.of(armRelativeEncoder.getDistance());
		inputs.Velocity.mut_replace(RotationsPerSecond.of(armRelativeEncoder.getRate()));
		previousPosition = currentPosition;

		inputs.Voltage.mut_replace(Volts.of(armMotor.getMotorOutputVoltage()));
		inputs.Current.mut_replace(Amps.of(armMotor.getStatorCurrent()));
		inputs.atSetpoint = armController.atGoal();
		
	}

	@Override
	public void setVoltage(Voltage voltage) {
		armMotor.set(ControlMode.PercentOutput, voltage.in(Volts) / 12.0);
	}

	@Override
	public void setAngle(Angle angle) {
		targetPosition = angle;
		double output = armController.calculate(Rotations.of(armRelativeEncoder.getDistance()).in(Degrees),targetPosition.in(Degrees));
		output += feedforward.calculate(Units.rotationsToRadians(armController.getSetpoint().position), armController.getSetpoint().velocity);
		Logger.recordOutput("Arm PID/output", output);
		Logger.recordOutput("Arm PID/measure", Rotations.of(armRelativeEncoder.getDistance()).in(Degrees));
		Logger.recordOutput("Arm PID/goalSetpoint", angle.in(Degrees));
		Logger.recordOutput("Arm PID/controllerSetpoint", armController.getSetpoint().position);
		armMotor.set(ControlMode.PercentOutput, output);
	}



}
