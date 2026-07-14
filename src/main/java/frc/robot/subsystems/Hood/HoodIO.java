// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public boolean hoodConnected = false;
        public Angle hoodPosition;
        public AngularVelocity hoodMotorVelocityRadPerSec;
        public Voltage hoodMotorAppliedVolts;
        public Current hoodMotorCurrentAmps;

    }

    public default void updateInputs(HoodIOInputs inputs) {
    }

    public default void setHoodMotorVoltage(Voltage volts) {
    }

    public default void setHoodTargetState(Angle target, AngularVelocity velocityTarget) {
    }

    public default void setHoodPositionToHome() {
    }
}
