// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spindexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  /** Creates a new Intake. */
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  // double rad;
  // double line;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public Command setVelocity(AngularVelocity velocity) {
    return runEnd(() -> {
      io.setTargetVelocity(velocity);
    },
        () -> {

        });
  }

  public void setVolts(Voltage voltage) {
    io.setTargetVolts(voltage);
  }

  public Command setVoltsCommand(Voltage voltage) {
    return runEnd(() -> {
      io.setTargetVolts(voltage);
    },
        () -> {

        });
  }
}
