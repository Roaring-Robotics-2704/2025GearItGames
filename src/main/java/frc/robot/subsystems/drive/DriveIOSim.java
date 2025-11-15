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
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
  private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kSingleCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  private Voltage leftAppliedVoltage = Volts.zero();
  private Voltage rightAppliedVoltage = Volts.zero();
  private boolean closedLoop = false;
  private PIDController leftPID = new PIDController(REAL_P, 0.0, REAL_D);
  private PIDController rightPID = new PIDController(REAL_P, 0.0, REAL_D);
  private Voltage leftFFVoltage = Volts.zero();
  private Voltage rightFFVoltage = Volts.zero();

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVoltage = 
          leftFFVoltage.plus(Volts.of(leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / wheelRadius.in(Meters))));
      rightAppliedVoltage =
          rightFFVoltage.plus(Volts.of(rightPID.calculate(sim.getRightVelocityMetersPerSecond() / wheelRadius.in(Meters))));
    }

    // Update simulation state
    sim.setInputs(
        MathUtil.clamp(leftAppliedVoltage.in(Volts), -12.0, 12.0),
        MathUtil.clamp(rightAppliedVoltage.in(Volts), -12.0, 12.0));
    sim.update(0.02);

    inputs.leftPosition.mut_replace(Radians.of(sim.getLeftPositionMeters() / wheelRadius.in(Meters)));
    inputs.leftAngularVelocity.mut_replace(RadiansPerSecond.of(sim.getLeftVelocityMetersPerSecond() / wheelRadius.in(Meters)));
    inputs.leftAppliedVoltage.mut_replace(leftAppliedVoltage);
    inputs.leftCurrent.mut_replace(Amps.of(sim.getLeftCurrentDrawAmps()));

    inputs.rightPosition.mut_replace(Radians.of(sim.getRightPositionMeters() / wheelRadius.in(Meters)));
    inputs.rightAngularVelocity.mut_replace(RadiansPerSecond.of(sim.getRightVelocityMetersPerSecond() / wheelRadius.in(Meters)));
    inputs.rightAppliedVoltage.mut_replace(rightAppliedVoltage);
    inputs.rightCurrent.mut_replace(Amps.of(sim.getRightCurrentDrawAmps()));
  }

  @Override
  public void setVoltage(Voltage leftVoltage, Voltage rightVoltage) {
    closedLoop = false;
    leftAppliedVoltage = leftVoltage;
    rightAppliedVoltage = rightVoltage;
  }

  @Override
  public void setVelocity(
      AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity, Voltage leftFFVoltage, Voltage rightFFVoltage) {
    closedLoop = true;
    this.leftFFVoltage = leftFFVoltage;
    this.rightFFVoltage = rightFFVoltage;
    leftPID.setSetpoint(leftAngularVelocity.in(RadiansPerSecond));
    rightPID.setSetpoint(rightAngularVelocity.in(RadiansPerSecond));
  }
}
