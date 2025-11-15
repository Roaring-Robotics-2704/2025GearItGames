// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class SuperStructureConstants {
  /** Creates a new SuperStructureConstants. */
  private SuperStructureConstants() {}

  public enum SuperStructureState {
    DEFAULT_STATE(Degrees.of(45), false),
    INTAKE_UPRIGHT(Degrees.of(0), false),
    INTAKE_UPRIGHT_GRAB(INTAKE_UPRIGHT.getArmAngle(), true),
    INTAKE_UPSIDE_DOWN(INTAKE_UPRIGHT.getArmAngle(), false),
    INTAKE_UPSIDE_DOWN_GRAB(INTAKE_UPSIDE_DOWN.getArmAngle(), true),
    HOLDING(Degrees.of(45), true),
    OUTTAKING_HOLD(Degrees.of(90), true),
    OUTTAKING(OUTTAKING_HOLD.getArmAngle(), false);

    private Angle armAngle;
    private boolean gripperClosed;
    private SuperStructureState(Angle armAngle, boolean gripperClosed) {
      this.armAngle = armAngle;
      this.gripperClosed = gripperClosed;
    }
    public Angle getArmAngle() {
      return this.armAngle;
    }
    public boolean getGripperClosed() {
      return this.gripperClosed;
    }
  }

  public enum SuperStructureStatus {
    IDLE,
    TRANSITIONING,
    ERROR,
  }

}
