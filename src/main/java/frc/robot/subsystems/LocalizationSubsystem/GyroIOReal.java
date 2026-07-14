// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LocalizationSubsystem;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class GyroIOReal implements GyroIO {
  Pigeon2 gyro = new Pigeon2(48);

  AngularVelocity lastAngularVelocity = RadiansPerSecond.of(0);

  /** Creates a new Localization. */
  public GyroIOReal() {
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.gyroAngle = Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    inputs.gyroAngularVelocity = DegreesPerSecond.of(gyro.getAngularVelocityXDevice().getValueAsDouble());
    inputs.accelerationX = MetersPerSecondPerSecond.of(gyro.getAccelerationX().getValueAsDouble());
    inputs.accelerationY = MetersPerSecondPerSecond.of(gyro.getAccelerationY().getValueAsDouble());
    inputs.accelerationZ = MetersPerSecondPerSecond.of(gyro.getAccelerationZ().getValueAsDouble());
    inputs.angularAcceleration = RadiansPerSecondPerSecond
        .of(((inputs.gyroAngularVelocity.in(RadiansPerSecond) - lastAngularVelocity.in(RadiansPerSecond)))
            / Constants.simulationTimestep.in(Seconds));
    lastAngularVelocity = inputs.gyroAngularVelocity;
  }
}