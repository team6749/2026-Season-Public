// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public Angle position = Radians.of(0);
        public AngularVelocity velocity = RadiansPerSecond.of(0);
        public Voltage appliedVolts = Volts.of(0);
        public Current currentAmps = Amps.of(0);

    }

    public default void updateInputs(SpindexerIOInputs inputs) {
    }

    public default void setTargetVolts(Voltage volts) {
    }

    public default void setTargetVelocity(AngularVelocity velocity) {
    }
}
