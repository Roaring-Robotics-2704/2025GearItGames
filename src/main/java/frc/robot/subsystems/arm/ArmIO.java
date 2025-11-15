// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

//import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
//import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public Angle Position = Degrees.zero();
    public AngularVelocity Velocity = DegreesPerSecond.zero();
    public Voltage Voltage = Volts.zero();
    public Current Current = Amps.zero();


  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(Voltage voltage) {}

  public default void setAngle(Angle angle) {}

  public default boolean atSetpoint() {
    return true;
  }
}
