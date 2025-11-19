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
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
	private static final double kS = REAL_S;
	private static final double kV = REAL_V;
	private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
			new Rotation2d(), 0.0, 0.0, new Pose2d());
	private final SysIdRoutine sysId;
	private Rotation2d rawGyroRotation = new Rotation2d();
	private Distance lastLeftPosition = Meter.zero();
	private Distance lastRightPosition = Meter.zero();

	public Drive(DriveIO io, GyroIO gyroIO) {
		this.io = io;
		this.gyroIO = gyroIO;

		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configure(
				this::getPose,
				this::setPose,
				() -> kinematics.toChassisSpeeds(
						new DifferentialDriveWheelSpeeds(
								getLeftVelocity().in(RadiansPerSecond), getRightVelocity().in(RadiansPerSecond))),
				(ChassisSpeeds speeds) -> runClosedLoop(speeds),
				new PPLTVController(0.02, maxSpeed.in(MetersPerSecond)),
				ppConfig,
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
				this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback(
				activePath -> Logger.recordOutput(
						"Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
		PathPlannerLogging.setLogTargetPoseCallback(
				targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

		// Configure SysId
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(
						null,
						null,
						null,
						state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism(
						voltage -> runOpenLoop(voltage, voltage), null, this));
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive", inputs);
		Logger.processInputs("Drive/Gyro", inputs);

		// Update gyro angle
		if (gyroInputs.connected) {
			// Use the real gyro angle
			rawGyroRotation = gyroInputs.yawPosition;
		} else {
			// Use the angle delta from the kinematics and module deltas
			Twist2d twist = kinematics.toTwist2d(
					getLeftPosition().minus(lastLeftPosition).in(Meters),
					getRightPosition().minus(lastRightPosition).in(Meters));
			rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			lastLeftPosition = getLeftPosition();
			lastRightPosition = getRightPosition();
		}

		// Update odometry
		poseEstimator.update(rawGyroRotation, getLeftPosition().in(Meters), getRightPosition().in(Meters));
	}

	/** Runs the drive at the desired velocity. */
	public void runClosedLoop(ChassisSpeeds speeds) {
		var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
		runClosedLoop(MetersPerSecond.of(wheelSpeeds.leftMetersPerSecond),
				MetersPerSecond.of(wheelSpeeds.rightMetersPerSecond));
	}

	/** Runs the drive at the desired left and right velocities. */
	public void runClosedLoop(LinearVelocity leftVelocity, LinearVelocity rightVelocity) {
		AngularVelocity leftAngularVelocity = RadiansPerSecond
				.of(leftVelocity.in(MetersPerSecond) / wheelRadius.in(Meters));
		AngularVelocity rightAngularVelocity = RadiansPerSecond
				.of(rightVelocity.in(MetersPerSecond) / wheelRadius.in(Meters));
		Logger.recordOutput("Drive/LeftSetpointRadPerSec", leftAngularVelocity);
		Logger.recordOutput("Drive/RightSetpointRadPerSec", rightAngularVelocity);

		Voltage leftFFVoltage = Volts.of(kS * Math.signum(leftAngularVelocity.in(RadiansPerSecond))
				+ kV * leftAngularVelocity.in(RadiansPerSecond));
		Voltage rightFFVoltage = Volts.of(kS * Math.signum(rightAngularVelocity.in(RadiansPerSecond))
				+ kV * rightAngularVelocity.in(RadiansPerSecond));
		io.setVelocity(leftAngularVelocity, rightAngularVelocity, leftFFVoltage, rightFFVoltage);
	}

	/** Runs the drive in open loop. */
	public void runOpenLoop(Voltage leftVoltage, Voltage rightVoltage) {
		io.setVoltage(leftVoltage, rightVoltage);
	}

	/** Stops the drive. */
	public void stop() {
		runOpenLoop(Volts.of(0.0), Volts.of(0.0));
	}

	/** Returns a command to run a quasistatic test in the specified direction. */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction);
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction);
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/** Returns the current odometry rotation. */
	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	/** Resets the current odometry pose. */
	public void setPose(Pose2d pose) {
		poseEstimator.resetPosition(
				rawGyroRotation, getLeftPosition().in(Meters), getRightPosition().in(Meters), pose);
	}

	/**
	 * Adds a vision measurement to the pose estimator.
	 *
	 * @param visionPose The pose of the robot as measured by the vision camera.
	 * @param timestamp  The timestamp of the vision measurement in seconds.
	 */
	public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
		poseEstimator.addVisionMeasurement(visionPose, timestamp);
	}

	/** Returns the position of the left wheels in meters. */
	@AutoLogOutput
	public Distance getLeftPosition() {
		return Meters.of(inputs.leftPosition.in(Radians) * wheelRadius.in(Meters));
	}

	/** Returns the position of the right wheels in meters. */
	@AutoLogOutput
	public Distance getRightPosition() {
		return Meters.of(inputs.rightPosition.in(Radians) * wheelRadius.in(Meters));
	}

	/** Returns the velocity of the left wheels in meters/second. */
	@AutoLogOutput
	public AngularVelocity getLeftVelocity() {
		return inputs.leftAngularVelocity.times(wheelRadius.in(Meters));
	}

	/** Returns the velocity of the right wheels in meters/second. */
	@AutoLogOutput
	public AngularVelocity getRightVelocity() {
		return inputs.rightAngularVelocity.times(wheelRadius.in(Meters));
	}

	/** Returns the average velocity in radians/second. */
	public AngularVelocity getCharacterizationVelocity() {
		return (inputs.leftAngularVelocity.plus(inputs.rightAngularVelocity)).div(2.0);
	}

}
