// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.ARM_CONSTRAINTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmConstants.PID;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {
    // Simulation display
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d armMech;

    // Visualization constants
    private final double BASE_WIDTH = 40.0;
    private final double BASE_HEIGHT = 20.0;
    private final double TOWER_HEIGHT = 30.0;
    private final double ARM_WIDTH = 10.0;
    Encoder armEncoder = new Encoder(0, 1);

    // Arm parameters
    private final double armLength = 1;
    private final double visualScaleFactor = 200.0 / armLength; // Scale arm length to fit in the mechanism display
        ProfiledPIDController armController;

    PWMTalonSRX armMotor = new PWMTalonSRX(0);


    private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          ArmConstants.armMotor,
          1,
          SingleJointedArmSim.estimateMOI(ArmConstants.armLength, ArmConstants.kArmMass),
          ArmConstants.armLength,
          ArmConstants.kMinAngleRads,
          ArmConstants.kMaxAngleRads,
          true,
          0,
          ArmConstants.kArmEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
          );
      EncoderSim armEncoderSim = new EncoderSim(armEncoder);

    public ArmIOSim() {

    armEncoder.setDistancePerPulse(ArmConstants.kArmEncoderDistPerPulse);

    // Create the simulation display
    mech = new Mechanism2d(400, 400);
    root = mech.getRoot("ArmRoot", 200, 200);
    armController = new ProfiledPIDController(PID.ARM_P, PID.ARM_I, PID.ARM_D, ARM_CONSTRAINTS);


    // Add arm base
    MechanismLigament2d armBase = root.append(
      new MechanismLigament2d(
        "Base",
        BASE_WIDTH,
        0,
        BASE_HEIGHT,
        new Color8Bit(Color.kDarkGray)
      )
    );

    // Add tower
    MechanismLigament2d tower = armBase.append(
      new MechanismLigament2d(
        "Tower",
        TOWER_HEIGHT,
        90,
        BASE_HEIGHT / 2,
        new Color8Bit(Color.kGray)
      )
    );

    // Add the arm pivot point
    MechanismLigament2d pivot = tower.append(
      new MechanismLigament2d("Pivot", 5, 0, 5, new Color8Bit(Color.kBlack))
    );

    // Add the arm
    armMech = pivot.append(
      new MechanismLigament2d(
        "Arm",
        armLength * visualScaleFactor,
        0,
        ARM_WIDTH,
        new Color8Bit(Color.kBlue)
      )
    );

    // Initialize visualization
    SmartDashboard.putData("Arm Sim", mech);
    
    }

    @Override 
    public void updateInputs(ArmIOInputs inputs) {
        // Update the simulation
        m_armSim.setInput(armMotor.get() * 12.0); // Convert motor output to voltage
        m_armSim.update(0.02);

        // Update encoder simulation
        armEncoderSim.setDistance(m_armSim.getAngleRads() * (180.0 / Math.PI)); // Convert radians to degrees
        armEncoderSim.setRate(m_armSim.getVelocityRadPerSec() * (180.0 / Math.PI)); // Convert radians/sec to degrees/sec

        // Update inputs
        inputs.Position = edu.wpi.first.units.Units.Degrees.of(armEncoder.getDistance());
        inputs.Velocity = edu.wpi.first.units.Units.DegreesPerSecond.of(armEncoder.getRate());
        inputs.Voltage = edu.wpi.first.units.Units.Volts.of(armMotor.get() * 12.0);
        inputs.Current = edu.wpi.first.units.Units.Amps.of(m_armSim.getCurrentDrawAmps());
         RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

        // Update visualization
        armMech.setAngle(inputs.Position.in(edu.wpi.first.units.Units.Degrees));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        armMotor.setVoltage(voltage);
    }

    @Override
    public void setAngle(edu.wpi.first.units.measure.Angle angle) {
        armController.setGoal(angle.in(edu.wpi.first.units.Units.Degrees));
        double output = armController.calculate(armEncoder.getDistance());
        armMotor.setVoltage(output*12);
    }

    //TODO add atSetpoint method
}
