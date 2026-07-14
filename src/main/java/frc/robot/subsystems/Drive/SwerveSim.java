// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveSim implements ModuleIO {
  private static final DCMotor driveMotorModel = DCMotor.getKrakenX60(1);
  private static final DCMotor turnMotorModel = DCMotor.getKrakenX60(1);

  private final DCMotorSim driveSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(driveMotorModel, 0.03, SwerveConstants.driveReduction),
      driveMotorModel);

  private final DCMotorSim turnSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(turnMotorModel, 0.005, SwerveConstants.angleReduction),
      turnMotorModel);

  // Consider making these contants set in swerve constants so there's no
  // inconsistency between the simulator implmentation and the physical hardware.
  private PIDController driveController = new PIDController(2, 0, 0);
  private PIDController turnController = new PIDController(4, 0, 0);
  public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.18, 2.45);

  private boolean isClosedLoop;
  private Voltage driveAppliedVolts = Volts.of(0.0);
  private Voltage turnAppliedVolts = Volts.of(0.0);

  private LinearVelocity targetVelocity = MetersPerSecond.of(0);

  double FFOutput = 0;

  public SwerveSim() {
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // seperate into drive open loop and angle.
  public void runOpenLoop(Voltage drive, Voltage turn) {
    isClosedLoop = false;
    driveSim.setInputVoltage(drive.in(Volts));
    turnSim.setInputVoltage(turn.in(Volts));
  }

  public void stop() {
    driveSim.setInputVoltage(0);
    turnSim.setInputVoltage(0);
  }

  public void setBrakeMode(boolean brake) {
    // Virtual Swerve module has no brake mode.
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec()).in(RotationsPerSecond))
        .times(SwerveConstants.wheelCircumference.in(Meters));
  }

  public Rotation2d getAngle() {
    return new Rotation2d(turnSim.getAngularPositionRad());
  }

  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    

    // Update Simulator
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts.in(Volts), -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts.in(Volts), -12.0, 12.0));
    driveSim.update(Constants.simulationTimestep.in(Seconds));
    turnSim.update(Constants.simulationTimestep.in(Seconds));

    inputs.driveConnected = true;
    inputs.drivePositionMeters = Meters
        .of(Radians.of(driveSim.getAngularPositionRad()).in(Rotations) * SwerveConstants.wheelCircumference.in(Meters));
    inputs.driveVelocity = getVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityDegreesPerSec = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    if (isClosedLoop) {
      driveAppliedVolts = Volts.of(FFOutput + driveController
          .calculate(RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec()).in(RotationsPerSecond))
          * (SwerveConstants.wheelCircumference.in(Meters)));
    } else {
      driveController.reset();
    }
    if (isClosedLoop) {
      turnAppliedVolts = Volts.of(turnController.calculate(turnSim.getAngularPositionRad()));
    } else {
      turnController.reset();
    }
  }

  @Override
  public void setDriveVelocity(LinearVelocity velocityMetersPerSec) {
    isClosedLoop = true;
    FFOutput = driveFeedForward.calculate(velocityMetersPerSec.in(MetersPerSecond));
    driveController.setSetpoint(velocityMetersPerSec.in(MetersPerSecond));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    isClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
