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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import javax.sound.sampled.Line;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
	private final DriveIO io;
	private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
	private static final double kS = REAL_S;
	private static final double kV = REAL_V;


	public Drive(DriveIO io) {
		this.io = io;

		// Configure AutoBuilder for PathPlanner
		// AutoBuilder.configure(
		// 		this::getPose,
		// 		this::setPose,
		// 		() -> kinematics.toChassisSpeeds(
		// 				new DifferentialDriveWheelSpeeds(
		// 						getLeftVelocity().in(RadiansPerSecond), getRightVelocity().in(RadiansPerSecond))),
		// 		(ChassisSpeeds speeds) -> runClosedLoop(speeds),
		// 		new PPLTVController(0.02, maxSpeed.in(MetersPerSecond)),
		// 		ppConfig,
		// 		() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
		// 		this);
		// Pathfinding.setPathfinder(new LocalADStarAK());
		// PathPlannerLogging.setLogActivePathCallback(
		// 		activePath -> Logger.recordOutput(
		// 				"Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
		// PathPlannerLogging.setLogTargetPoseCallback(
		// 		targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

		// Configure SysId
		// sysId = new SysIdRoutine(
		// 		new SysIdRoutine.Config(
		// 				null,
		// 				null,
		// 				null,
		// 				state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
		// 		new SysIdRoutine.Mechanism(
		// 				voltage -> runOpenLoop(voltage, voltage), null, this));
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		// gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive", inputs);
		Logger.processInputs("Drive/Gyro", inputs);
	}

	// /** Runs the drive at the desired velocity. */
	// public void runClosedLoop(ChassisSpeeds speeds) {
	// 	var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
	// 	runClosedLoop(MetersPerSecond.of(wheelSpeeds.leftMetersPerSecond),
	// 			MetersPerSecond.of(wheelSpeeds.rightMetersPerSecond));
	// }

	// /** Runs the drive at the desired left and right velocities. */
	// public void runClosedLoop(LinearVelocity leftVelocity, LinearVelocity rightVelocity) {
	// 	AngularVelocity leftAngularVelocity = RadiansPerSecond
	// 			.of(leftVelocity.in(MetersPerSecond) / wheelRadius.in(Meters));
	// 	AngularVelocity rightAngularVelocity = RadiansPerSecond
	// 			.of(rightVelocity.in(MetersPerSecond) / wheelRadius.in(Meters));
	// 	Logger.recordOutput("Drive/LeftSetpointRadPerSec", leftAngularVelocity);
	// 	Logger.recordOutput("Drive/RightSetpointRadPerSec", rightAngularVelocity);

	// 	Voltage leftFFVoltage = Volts.of(kS * Math.signum(leftAngularVelocity.in(RadiansPerSecond))
	// 			+ kV * leftAngularVelocity.in(RadiansPerSecond));
	// 	Voltage rightFFVoltage = Volts.of(kS * Math.signum(rightAngularVelocity.in(RadiansPerSecond))
	// 			+ kV * rightAngularVelocity.in(RadiansPerSecond));
	// 	Logger.recordOutput("Drive/Outputs/LeftAngualarVelocity", leftAngularVelocity);
	// 	Logger.recordOutput("Drive/Outputs/RightAngularVelocity", rightAngularVelocity);
	// 	Logger.recordOutput("Drive/Outputs/LeftFFVoltage", leftFFVoltage);
	// 	Logger.recordOutput("Drive/Outputs/RightFFVoltage", rightFFVoltage);
	// 	io.setVelocity(leftAngularVelocity, rightAngularVelocity, leftFFVoltage, rightFFVoltage);
	// }

	/** Runs the drive in open loop. */
	public void runOpenLoop(Voltage leftVoltage, Voltage rightVoltage) {
		io.setVoltage(leftVoltage, rightVoltage);
	}

	/** Stops the drive. */
	public void stop() {
		runOpenLoop(Volts.of(0.0), Volts.of(0.0));
	}




}
