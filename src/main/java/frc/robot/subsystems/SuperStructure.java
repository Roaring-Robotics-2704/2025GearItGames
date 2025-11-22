// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripper.Gripper;

import static frc.robot.subsystems.SuperStructureConstants.SuperStructureState;
import static frc.robot.subsystems.SuperStructureConstants.SuperStructureStatus;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;

public class SuperStructure extends SubsystemBase {

	private SuperStructureState currentState = SuperStructureState.DEFAULT_STATE;
	private SuperStructureState desiredState = SuperStructureState.DEFAULT_STATE;
	private SuperStructureStatus currentStatus = SuperStructureStatus.IDLE;

	// Subsystems
	Arm arm;
	Gripper gripper;

	/** Creates a new SuperStructure. */
	public SuperStructure(Arm arm, Gripper gripper) {
		this.arm = arm;
		this.gripper = gripper;
	}

	public void setDesiredState(SuperStructureState state) {
		if (state != currentState) {
			currentStatus = SuperStructureStatus.TRANSITIONING;
		}
		this.desiredState = state;
	}

	public Command gotoState(SuperStructureState state) {
		return Commands.run(() -> setDesiredState(state), this).until(this::atDesiredState);
	}

	public SuperStructureState getDesiredState() {
		return this.desiredState;
	}

	public SuperStructureState getCurrentState() {
		return this.currentState;
	}

	public boolean atDesiredState() {
		return (this.currentState == this.desiredState)
				&& (this.currentStatus == SuperStructureStatus.IDLE);
	}

	@Override
	public void periodic() {
		Logger.recordOutput("SuperStructure/Current State", currentState);
		Logger.recordOutput("SuperStructure/Desired State", desiredState);
		Logger.recordOutput("SuperStructure/Current Status", currentStatus);
		

			if (arm.atSetpoint() && gripper.atSetpoint()) {
				currentState = desiredState;
				currentStatus = SuperStructureStatus.IDLE;
			}
			// Maintain current state
			arm.setArmAngle(desiredState.getArmAngle());
			gripper.setGripperClosed(desiredState.getGripperClosed());
		
	}
}
