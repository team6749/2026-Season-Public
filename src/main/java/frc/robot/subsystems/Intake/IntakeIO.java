// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeConnected = false;
        public Angle intakeMotorPosition = Radians.of(0);
        public AngularVelocity intakeMotorVelocityRadPerSec = RadiansPerSecond.of(0);
        public Voltage intakeMotorAppliedVolts = Volts.of(0);
        public Current intakeMotorCurrentAmps = Amps.of(0);

        public Angle intakePivotMotorPosition = Radians.of(0);
        public AngularVelocity intakePivotMotorVelocityRadPerSec = RadiansPerSecond.of(0);
        public Voltage intakePivotMotorAppliedVolts = Volts.of(0);
        public Current intakePivotMotorCurrentAmps = Amps.of(0);

    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setIntakeMotorVelocity(AngularVelocity velocity) {
    }

    public default void setIntakePivotMotorVoltage(Voltage volts) {
    }

    public default void setIntakeMotorVoltage(Voltage volts) {
    }
}
