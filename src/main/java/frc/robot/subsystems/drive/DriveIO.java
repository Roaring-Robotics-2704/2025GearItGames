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
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface DriveIO {
	@AutoLog
	public static class DriveIOInputs {
		public MutAngle leftPosition = Degrees.zero().mutableCopy();
		public MutAngularVelocity leftAngularVelocity = DegreesPerSecond.zero().mutableCopy();
		public MutVoltage leftAppliedVoltage = Volts.zero().mutableCopy();
		public MutCurrent leftCurrent = Amps.zero().mutableCopy();

		public MutAngle rightPosition = Degrees.zero().mutableCopy();
		public MutAngularVelocity rightAngularVelocity = DegreesPerSecond.zero().mutableCopy();
		public MutVoltage rightAppliedVoltage = Volts.zero().mutableCopy();
		public MutCurrent rightCurrent = Amps.zero().mutableCopy();
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(DriveIOInputs inputs) {
	}

	/** Run open loop at the specified voltage. */
	public default void setVoltage(Voltage leftVoltage, Voltage rightVoltage) {
	}

	/** Run closed loop at the specified velocity. */
	public default void setVelocity(AngularVelocity leftAngularVelocity, AngularVelocity rightAngularVelocity,
			Voltage leftFFVoltage, Voltage rightFFVoltage) {
	}
}
