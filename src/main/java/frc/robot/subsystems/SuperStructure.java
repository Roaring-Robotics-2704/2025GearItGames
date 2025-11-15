// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {

  private SuperStructureState currentState = SuperStructureState.DEFAULT_STATE;
  private SuperStructureState desiredState = SuperStructureState.DEFAULT_STATE;
  private SuperStructureStatus currentStatus = SuperStructureStatus.IDLE;

    /** Creates a new SuperStructure. */
    public SuperStructure() {}

  public enum SuperStructureState {
    DEFAULT_STATE,
    INTAKE_UPRIGHT,
    INTAKE_UPSIDE_DOWN,
    HOLDING,
    OUTTAKING,
  }

  public enum SuperStructureStatus {
    IDLE,
    TRANSITIONING,
    ERROR,
  }

  public void setDesiredState(SuperStructureState state) {
    if (state != currentState) {
      currentStatus = SuperStructureStatus.TRANSITIONING;
    }
    this.desiredState = state;
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
    // This method will be called once per scheduler run
  }
}
