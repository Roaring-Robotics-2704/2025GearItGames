// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import static frc.robot.subsystems.SuperStructureConstants.SuperStructureState;
import static frc.robot.subsystems.SuperStructureConstants.SuperStructureStatus;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperStructure extends SubsystemBase {

  private SuperStructureState currentState = SuperStructureState.DEFAULT_STATE;
  private SuperStructureState desiredState = SuperStructureState.DEFAULT_STATE;
  private SuperStructureStatus currentStatus = SuperStructureStatus.IDLE;

  //Subsystems
  Arm arm;

    /** Creates a new SuperStructure. */
    public SuperStructure(Arm arm) {
        this.arm = arm;
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
    if (currentState != desiredState ) {
      if (!arm.atSetpoint()) {
        arm.setArmAngle(desiredState.getArmAngle());
      }
    }

      if (arm.atSetpoint()) {
        currentState = desiredState;
        currentStatus = SuperStructureStatus.IDLE;
      }

    
    }
  }

