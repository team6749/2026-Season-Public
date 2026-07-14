// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.SwerveConstants.ModuleDefinition;

public class Module extends SubsystemBase {

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  public final ModuleDefinition definition;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;

  // im a goat
  /** Creates a new Module. */
  public Module(ModuleIO io, ModuleDefinition definition) {

    this.io = io;
    this.definition = definition;

    // Disconnect alerts
    driveDisconnectedAlert = new Alert(
        "Disconnected drive motor on module " + definition.label + ".",
        AlertType.kError);
    turnDisconnectedAlert = new Alert(
        "Disconnected turn motor on module " + definition.label + ".", AlertType.kError);
    turnEncoderDisconnectedAlert = new Alert(
        "Disconnected turn encoder on module " + definition.label + ".",
        AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + definition.label, inputs);

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  public void setClosedLoopGoal(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(getAngle());

    // Apply setpoints
    io.setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
    io.setTurnPosition(state.angle);
  }

  // getters and setters
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.stop();
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /** Returns the current drive position of the module in meters. */
  public Distance getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public LinearVelocity getVelocityMetersPerSec() {
    return inputs.driveVelocity;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position in Radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePosition.in(Radians);
  }

}
