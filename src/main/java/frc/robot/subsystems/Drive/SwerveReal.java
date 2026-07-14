// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SwerveReal implements ModuleIO {

  public TalonFX driveMotor;
  public TalonFX angleMotor;
  public CANcoder encoder;

  public PIDController drivePID = new PIDController(1.8, 0, 0);
  public PIDController anglePID = new PIDController(3.7, 0, 0);
  public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.18, 2.20);

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePositionRot;
  private final StatusSignal<AngularVelocity> driveVelocityRot;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPositionRot;
  private final StatusSignal<AngularVelocity> turnVelocityRot;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  private LinearVelocity targetVelocity = MetersPerSecond.of(0);

  public SwerveReal(int angleMotorPort, int encoderPort, int driveMotorPort) {

    driveMotor = new TalonFX(driveMotorPort);
    angleMotor = new TalonFX(angleMotorPort);
    anglePID.enableContinuousInput(-Math.PI, Math.PI);
    encoder = new CANcoder(encoderPort);
    angleMotor.setNeutralMode(NeutralModeValue.Brake);

    drivePositionRot = driveMotor.getPosition();
    driveVelocityRot = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();

    // Create turn status signals
    turnAbsolutePosition = encoder.getAbsolutePosition();
    turnPositionRot = angleMotor.getPosition();
    turnVelocityRot = angleMotor.getVelocity();
    turnAppliedVolts = angleMotor.getMotorVoltage();
    turnCurrent = angleMotor.getStatorCurrent();

    var angleMotorCurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60))
        .withStatorCurrentLimitEnable(true);
    angleMotor.getConfigurator().apply(angleMotorCurrentLimits);
  }

  public void stop() {
    driveMotor.stopMotor();
    angleMotor.stopMotor();
  }

  ModuleIOInputs myInputs = new ModuleIOInputs();

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // Refresh all signals
    var driveStatus = BaseStatusSignal.refreshAll(drivePositionRot, driveVelocityRot, driveAppliedVolts, driveCurrent);
    var turnStatus = BaseStatusSignal.refreshAll(turnPositionRot, turnVelocityRot, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePosition = Rotations.of(drivePositionRot.getValueAsDouble());
    inputs.drivePositionMeters = SwerveConstants.wheelCircumference
        .times(drivePositionRot.getValueAsDouble() / SwerveConstants.driveReduction);
    inputs.driveVelocity = MetersPerSecond.of(driveVelocityRot.getValueAsDouble() / SwerveConstants.driveReduction
        * SwerveConstants.wheelCircumference.in(Meters));
    inputs.driveAppliedVolts = Volts.of(driveAppliedVolts.getValueAsDouble());
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d
        .fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnVelocityDegreesPerSec = DegreesPerSecond
        .of(turnVelocityRot.getValueAsDouble() / SwerveConstants.angleReduction);
    inputs.turnAppliedVolts = Volts.of(turnAppliedVolts.getValueAsDouble());
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    double PIDDriveOutput = drivePID.calculate(myInputs.driveVelocity.in(MetersPerSecond),
        targetVelocity.in(MetersPerSecond));
    double FFoutput = driveFeedForward.calculate(targetVelocity.in(MetersPerSecond));

    // These should already be logged but I want to play it safe
    Logger.recordOutput("drivePID", PIDDriveOutput);
    Logger.recordOutput("driveFF", FFoutput);
    driveMotor.setVoltage(PIDDriveOutput + FFoutput);

    myInputs = inputs;
  }

  @Override
  public void setDriveVelocity(LinearVelocity velocityMetersPerSec) {
    targetVelocity = velocityMetersPerSec;
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double PIDAngleOutput = anglePID.calculate(myInputs.turnAbsolutePosition.getRadians(), rotation.getRadians());

    Logger.recordOutput("myInputs", myInputs.turnAbsolutePosition.getRadians());
    Logger.recordOutput("rotation", rotation.getRadians());
    Logger.recordOutput("anglePID", PIDAngleOutput);
    angleMotor.setVoltage(-PIDAngleOutput);
  }
}
