// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

/** Add your docs here. */
public class GripperIOSim implements GripperIO {

    public GripperIOSim() {
    }
    
    @Override
    public void updateInputs(GripperIOInputs inputs) {
    }

    @Override
    public void setVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    }

    @Override
    public boolean atSetpoint() {
        return true;
    }
}
