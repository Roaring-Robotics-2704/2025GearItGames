// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.arm.ArmConstants.tolerance;
import static frc.robot.subsystems.gripper.GripperConstants.*;

/** Add your docs here. */
public class GripperIOReal implements GripperIO {
	TalonSRX gripperMotor = new TalonSRX(GRIPPER_MOTOR_ID);
    Current currentCurrent = Amps.zero(); // which came first, the current or the current?


	public GripperIOReal() {
        gripperMotor.configFactoryDefault();
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.continuousCurrentLimit = (int)CURRENT_LIMIT.in(Amps);
        config.peakCurrentLimit = (int)CURRENT_LIMIT.in(Amps) + 10;
        gripperMotor.configAllSettings(config);
        gripperMotor.enableVoltageCompensation(true);
        
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        currentCurrent = Amps.of(gripperMotor.getStatorCurrent());
        inputs.voltage.mut_replace(Volts.of(gripperMotor.getMotorOutputVoltage()));
        inputs.current.mut_replace(Amps.of(gripperMotor.getStatorCurrent()));
        inputs.atSetpoint = (Math.abs(currentCurrent.in(Amps)) >= 50);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        gripperMotor.set(ControlMode.PercentOutput, voltage.in(Volts) / 12.0);
    }

}
