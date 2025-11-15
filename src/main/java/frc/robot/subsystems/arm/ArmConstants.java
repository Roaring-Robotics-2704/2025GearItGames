// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
//import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ArmConstants { 
    public static final int ARM_MOTOR_CAN_ID = 5;
    public static final int ARM_ENCODER_PORT = 0;

    //PID Constants
    static class PID {
        public static final double ARM_P = 1.0;
        public static final double ARM_I = 0.0;
        public static final double ARM_D = 0.0;
    }


    // Motion Profile Constraints
    public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS =
        new TrapezoidProfile.Constraints(90.0, 180.0); // degrees per second, degrees per second^2

    //Sim Constants
    public static final DCMotor armMotor = DCMotor.getCIM(1).withReduction(9).withReduction((double)36/16); // Example motor, replace with actual motor
    public static final double armLength = 1.0; // meters
    public static final double kArmMass = 5.0; // kg
    public static final double kMinAngleRads = -Math.toRadians(-5); // Minimum angle in radians
    public static final double kMaxAngleRads = Math.toRadians(135); // Maximum angle in radians
    public static final double kArmEncoderDistPerPulse = 360.0 / 1024.0; // degrees per pulse, example value
}
