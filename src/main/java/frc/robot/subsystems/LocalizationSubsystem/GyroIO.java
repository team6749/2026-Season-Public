// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LocalizationSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

public interface GyroIO {
  @AutoLog
  static class GyroIOInputs {
    public Rotation2d gyroAngle = Rotation2d.kZero;
    public AngularVelocity gyroAngularVelocity = RadiansPerSecond.of(0.0);
    public AngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.of(0);
    public LinearAcceleration accelerationX = MetersPerSecondPerSecond.of(0);
    public LinearAcceleration accelerationY = MetersPerSecondPerSecond.of(0);
    public LinearAcceleration accelerationZ = MetersPerSecondPerSecond.of(0);
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }
}
