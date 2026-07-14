// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Intake. */
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  boolean closedLoop = false;

  Debouncer m_debouncer = new Debouncer(0.1, DebounceType.kRising);

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public Command setFeederLinearSpeedCommand(LinearVelocity target) {
    return runEnd(() -> {
      io.setFeederTargetState(target);
    },
        () -> {
      io.setVoltage(Volts.of(0));
        });
  }

  //In future subsystems, we should set a target which we set in periodic.
  public void setFeederLinearSpeed(LinearVelocity target) {
    io.setFeederTargetState(target);
  }

  public void setVoltage(Voltage votlage) {
    io.setVoltage(votlage);
  }
}