// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
/** Add your docs here. */
public class GripperConstants {
    public static final int GRIPPER_MOTOR_ID = 4;

    public static final Voltage GRIPPER_OPEN_VOLTAGE = Volts.of(2.0);
    public static final Voltage GRIPPER_CLOSE_VOLTAGE = Volts.of(-2.0);
    public static final Current CURRENT_LIMIT = Amps.of(5.0);
    public static final Current TOLERANCE = Amps.of(1.0);
}
