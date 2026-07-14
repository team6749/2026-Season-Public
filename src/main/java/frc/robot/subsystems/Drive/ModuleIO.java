// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {

    public boolean driveConnected = false;
    public Angle drivePosition = Radians.of(0);
    public Distance drivePositionMeters = Meter.of(0.0);
    public LinearVelocity driveVelocity = MetersPerSecond.of(0.0);
    public Voltage driveAppliedVolts = Volts.of(0.0);
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public AngularVelocity turnVelocityDegreesPerSec = DegreesPerSecond.of(0.0);
    public Voltage turnAppliedVolts = Volts.of(0.0);
    public double turnCurrentAmps = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {
  }

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {
  }

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {
  }

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(LinearVelocity velocityMeterPerSec) {
  }

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {
  }

  public default void stop() {
  }

}
