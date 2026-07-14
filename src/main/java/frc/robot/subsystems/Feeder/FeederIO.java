// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public Voltage feederVoltage = Volts.of(0);
        public AngularVelocity angularVelocity = RadiansPerSecond.of(0);
        public LinearVelocity linearVelocity = MetersPerSecond.of(0);
    }

    public default void updateInputs(FeederIOInputs inputs) {
    }

    public default void setFeederTargetState(LinearVelocity target) {
    }

    public default void setVoltage(Voltage volts) {
    }
}