// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LocalizationSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

public class GyroIOSim implements GyroIO {
  Rotation2d rotation = Rotation2d.kZero;
  Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  LinearVelocity lastXVelocity = MetersPerSecond.of(0);
  LinearVelocity lastYVelocity = MetersPerSecond.of(0);
  AngularVelocity lastAngularVelocity = RadiansPerSecond.of(0);

  /** Creates a new Localization. */
  public GyroIOSim(Supplier<ChassisSpeeds> supplier) {
    chassisSpeedsSupplier = supplier;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.gyroAngularVelocity = RadiansPerSecond.of(chassisSpeedsSupplier.get().omegaRadiansPerSecond);
    rotation = Rotation2d.fromRadians(rotation.getRadians()
        + (inputs.gyroAngularVelocity.in(RadiansPerSecond) * Constants.simulationTimestep.in(Seconds)));
    inputs.gyroAngle = rotation;
    inputs.accelerationX = MetersPerSecondPerSecond
        .of((chassisSpeedsSupplier.get().vxMetersPerSecond - lastXVelocity.in(MetersPerSecond))
            / Constants.simulationTimestep.in(Seconds));
    inputs.accelerationY = MetersPerSecondPerSecond
        .of((chassisSpeedsSupplier.get().vyMetersPerSecond - lastYVelocity.in(MetersPerSecond))
            / Constants.simulationTimestep.in(Seconds));
    inputs.accelerationZ = MetersPerSecondPerSecond.of(0);
    inputs.angularAcceleration = RadiansPerSecondPerSecond
        .of((inputs.gyroAngularVelocity.in(RadiansPerSecond) - lastAngularVelocity.in(RadiansPerSecond))
            / Constants.simulationTimestep.in(Seconds));
    lastXVelocity = MetersPerSecond.of(chassisSpeedsSupplier.get().vxMetersPerSecond);
    lastYVelocity = MetersPerSecond.of(chassisSpeedsSupplier.get().vyMetersPerSecond);
    lastAngularVelocity = inputs.gyroAngularVelocity;
  }
}
