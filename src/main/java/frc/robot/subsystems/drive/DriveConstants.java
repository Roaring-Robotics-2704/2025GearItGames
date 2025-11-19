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
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;

public class DriveConstants {
	public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.0);
	public static final Distance trackWidth = Inches.of(20);

	// Device CAN IDs
	public static final int pigeonCanId = 9;
	public static final int leftLeaderCanId = 1;
	public static final int rightLeaderCanId = 2;

	// Motor configuration
	public static final Current currentLimit = Amps.of(60);
	public static final Distance wheelRadius = Inches.of(3.0);
	public static final double motorReduction = 10.71;
	public static final boolean leftInverted = false;
	public static final boolean rightInverted = true;
	public static final DCMotor gearbox = DCMotor.getCIM(1);

	// Velocity PID configuration
	public static final double REAL_P = 0.0;
	public static final double REAL_D = 0.0;
	public static final double REAL_S = 0.0;
	public static final double REAL_V = 0.1;

	// PathPlanner configuration
	public static final Mass robotMass = Kilograms.of(74.088);
	public static final double robotMOI = 6.883;
	public static final double wheelCOF = 1.2;
	public static final RobotConfig ppConfig = new RobotConfig(
			robotMass.in(Kilograms),
			robotMOI,
			new ModuleConfig(
					wheelRadius.in(Meters),
					maxSpeed.in(MetersPerSecond),
					wheelCOF,
					gearbox.withReduction(motorReduction),
					currentLimit.in(Amps),
					1),
			trackWidth.in(Meters));
}
