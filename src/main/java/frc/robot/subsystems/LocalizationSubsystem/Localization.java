// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LocalizationSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.SwerveChassis;

public class Localization extends SubsystemBase {
  public Translation3d robotRelativeTurretPosition = new Translation3d(-0.13, 0.14, 0.46);

  // not sure if this should be a sdpe but we'll try it.
  SwerveDrivePoseEstimator poseEstimator;
  Field2d dashboardField = new Field2d();
  GyroIO io;
  Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  /** Creates a new Localization. */
  public Localization(GyroIO io, SwerveChassis chassis, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.io = io;
    poseEstimator = new SwerveDrivePoseEstimator(chassis.kinematics, Rotation2d.kZero, Constants.startingPositions,
        Pose2d.kZero);
    chassis.setOdometryConsumer(this::addOdometryMeasurement);
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      Logger.recordOutput("Auto/TargetPose", pose);
    });
    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      Logger.recordOutput("Auto/Path", poses.toArray(new Pose2d[0]));
    });
    SmartDashboard.putData("field", dashboardField);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters,
        timestampSeconds,
        visionMeasurementStdDevs);
  }

  public void addOdometryMeasurement(SwerveModulePosition[] wheelPositions, double time) {
    poseEstimator.updateWithTime(time, inputs.gyroAngle, wheelPositions);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Localization", inputs);

    dashboardField.setRobotPose(getFieldPose().getX(), getFieldPose().getY(), getFieldPose().getRotation());
  }

  @AutoLogOutput(key = "MeasuredPose")
  public Pose2d getFieldPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // only gets gyro angle
  public Rotation2d getGyroAngle() {
    return inputs.gyroAngle;
  }

  @AutoLogOutput(key = "Turret/turretPose")
  public Translation3d getTurretFieldPose() {
    return new Pose3d(getFieldPose()).transformBy(new Transform3d(robotRelativeTurretPosition, new Rotation3d(0, 0, 0)))
        .getTranslation();
  }

  public AngularVelocity getGyroVelocity() {
    return inputs.gyroAngularVelocity;
  }

  @AutoLogOutput(key = "Turret/TurretVelo")
  public Translation3d getTurretFieldVelocity() {
    ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

    double robotTurretRelativeAngleRad = Math.atan2(robotRelativeTurretPosition.getY(),
        robotRelativeTurretPosition.getX());
    LinearVelocity tangentialVelocity = MetersPerSecond
        .of(Math.hypot(robotRelativeTurretPosition.getX(), robotRelativeTurretPosition.getY())
            * inputs.gyroAngularVelocity.in(RadiansPerSecond));
    // return new Translation3d(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,
    // inputs.robotAngle).vxMetersPerSecond +
    // (tangentialVelocity.in(MetersPerSecond) *
    // Math.cos(getFieldPose().getRotation().getRadians())),
    // ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,
    // inputs.robotAngle).vyMetersPerSecond + tangentialVelocity.in(MetersPerSecond)
    // * Math.sin(getFieldPose().getRotation().getRadians()), 0);
    return new Translation3d(
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getFieldPose().getRotation()).vxMetersPerSecond,
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getFieldPose().getRotation()).vyMetersPerSecond,
        0).plus(
            new Translation3d(
                // idk why adding math.PI fixes it but now it correctly goes further away when
                // the turrets velocity is going away from the target
                // in field space.
                tangentialVelocity.in(MetersPerSecond) * Math
                    .cos(getFieldPose().getRotation().getRadians() + (robotTurretRelativeAngleRad + Math.PI / 2d)),
                tangentialVelocity.in(MetersPerSecond) * Math
                    .sin(getFieldPose().getRotation().getRadians() + (robotTurretRelativeAngleRad + Math.PI / 2d)),
                0));
  }

  public void resetPose(Pose2d newPose) {
    poseEstimator.resetPose(newPose);
  }

  public AccelerationData getAcceleration() {
    return new AccelerationData(inputs.accelerationX, inputs.accelerationY, inputs.accelerationZ,
        inputs.angularAcceleration);
  }
}
