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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** This drive implementation is for Talon SRXs driving brushed motors (e.g. CIMS) with encoders. */
public class DriveIOTalonSRX implements DriveIO {
  private static final double tickPerRevolution = 1440;

  private final TalonSRX leftLeader = new TalonSRX(leftLeaderCanId);
  private final TalonSRX rightLeader = new TalonSRX(rightLeaderCanId);

  public DriveIOTalonSRX() {
    var config = new TalonSRXConfiguration();
    config.peakCurrentLimit = (int)currentLimit.in(Amps);
    config.continuousCurrentLimit = (int)currentLimit.in(Amps) - 15;
    config.peakCurrentDuration = 250;
    config.voltageCompSaturation = 12.0;
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

    tryUntilOkV5(5, () -> leftLeader.configAllSettings(config));
    tryUntilOkV5(5, () -> rightLeader.configAllSettings(config));

    leftLeader.setInverted(leftInverted);
    rightLeader.setInverted(rightInverted);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPosition.mut_replace(Rotations.of(leftLeader.getSelectedSensorPosition() / tickPerRevolution));
    inputs.leftAngularVelocity.mut_replace(
          RotationsPerSecond.of(
            leftLeader.getSelectedSensorVelocity()
                / tickPerRevolution
                * 10.0)); // Raw units are ticks per 100ms :(
    inputs.leftAppliedVoltage.mut_replace(Volts.of(leftLeader.getMotorOutputVoltage()));
    inputs.leftCurrent.mut_replace(Amps.of(leftLeader.getStatorCurrent()));

    inputs.rightPosition.mut_replace(
        Rotations.of(rightLeader.getSelectedSensorPosition() / tickPerRevolution));
    inputs.rightAngularVelocity.mut_replace(
        RotationsPerSecond.of(
            rightLeader.getSelectedSensorVelocity()
                / tickPerRevolution
                * 10.0)); // Raw units are ticks per 100ms :(
    inputs.rightAppliedVoltage.mut_replace(Volts.of(rightLeader.getMotorOutputVoltage()));
    inputs.rightCurrent.mut_replace(Amps.of(rightLeader.getStatorCurrent()));
  }

  @Override
  public void setVoltage(Voltage leftVoltage, Voltage rightVoltage) {
    // OK to just divide by 12 because voltage compensation is enabled
    leftLeader.set(TalonSRXControlMode.PercentOutput, leftVoltage.in(Volts) / 12.0);
    rightLeader.set(TalonSRXControlMode.PercentOutput, rightVoltage.in(Volts) / 12.0);
  }

  @Override
  public void setVelocity(
      AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity, Voltage leftFFVoltage, Voltage rightFFVoltage) {
    // OK to just divide FF by 12 because voltage compensation is enabled
    leftLeader.set(
        TalonSRXControlMode.Velocity,
        leftAngularVelocity.in(RotationsPerSecond) / 10.0, // Raw units are ticks per 100ms :(
        DemandType.ArbitraryFeedForward,
        leftFFVoltage.in(Volts) / 12.0);
    rightLeader.set(
        TalonSRXControlMode.Velocity,
      rightAngularVelocity.in(RotationsPerSecond) / 10.0, // Raw units are ticks per 100ms :(
        DemandType.ArbitraryFeedForward,
        rightFFVoltage.in(Volts) / 12.0);
  }
}
