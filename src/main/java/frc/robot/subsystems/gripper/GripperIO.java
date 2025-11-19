// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;


public interface GripperIO {
  @AutoLog
  public class GripperIOInputs {
    public MutVoltage voltage = Volts.zero().mutableCopy();
    public MutCurrent current = Amps.zero().mutableCopy();

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GripperIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(Voltage voltage) {}
  @AutoLogOutput
  public default boolean atSetpoint() {
    return true;
  }
}