// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final Alert intakeDisconnectedAlert;
  /** Creates a new Intake. */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  double magicConversionRatio = 2.5 * Math.PI;
  boolean isHomedAsterisk = false;
  Distance magicCircumference = Inches.of(2.5 / 6.0 * Math.PI);
  Timer timer = new Timer();

  public Intake(IntakeIO io) {
    intakeDisconnectedAlert = new Alert(
        "Disconnected intakeRun motor.",
        AlertType.kError);
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeDisconnectedAlert.set(!inputs.intakeConnected);
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/isHomed", isHomedAsterisk);
    if (DriverStation.isEnabled()) {
      timer.start();
      if (!isHomedAsterisk) {
        io.setIntakePivotMotorVoltage(Volts.of(2));
        if (timer.get() > 2) {
          isHomedAsterisk = true;
        }
      } else {
        io.setIntakePivotMotorVoltage(Volts.of(0));
      }
      Logger.recordOutput("Intake/timer", timer.get());
    }
  }

  public Command setVelocityCommand(LinearVelocity velocity) {
    return runEnd(() -> {
      io.setIntakeMotorVelocity(LinearToAngularVelocity(velocity));
    },
        () -> {
          io.setIntakeMotorVelocity(RadiansPerSecond.of(0));
        });
  }
    public Command setVoltageCommand(Voltage votls) {
    return runEnd(() -> {
      io.setIntakeMotorVoltage(votls);
    },
        () -> {
          io.setIntakeMotorVoltage(Volts.of(0));
        });
  }

  public void setVelocity(LinearVelocity velocity) {
    //io.setIntakeMotorVelocity(LinearToAngularVelocity(velocity));
  }

  public AngularVelocity LinearToAngularVelocity(LinearVelocity velocity) {
    return RadiansPerSecond.of(velocity.in(MetersPerSecond) / magicCircumference.in(Meters));
  }

  public Command waitForIntakeDeploy() {
    return Commands.waitSeconds(1);
  }
}
