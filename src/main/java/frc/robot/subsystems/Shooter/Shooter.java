// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Intake. */
  private final Alert shooterDisconnectedAlert;
  private final ShooterIO io;
  public boolean closedLoop = false;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public Voltage voltsTarget = Volts.of(0);
  public AngularVelocity velocityTarget = RadiansPerSecond.of(0);
  InterpolatingDoubleTreeMap toRPM = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap toLinear = new InterpolatingDoubleTreeMap();

  public Shooter(ShooterIO io, List<Pair<Double, Double>> radiansPerMinuteToLinearVelocity) {
    this.io = io;
    // TODO do the linear interpolation at some point
    for (Pair<Double, Double> item : radiansPerMinuteToLinearVelocity) {
      toLinear.put(item.getFirst(), item.getSecond());
      toRPM.put(item.getSecond(), item.getFirst());
    }
    shooterDisconnectedAlert = new Alert(
        "Disconnected shooter motor.",
        AlertType.kError);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterDisconnectedAlert.set(!inputs.shooterConnected);
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if(closedLoop) {
    io.setTargetVelocity(velocityTarget);
    } else {
      io.setVoltage(voltsTarget);
    }
  }

  public Command setCommandVelocity(AngularVelocity velocity) {
    return runEnd(() -> {
      setVelocity(velocity);
      System.out.println(velocity.in(RadiansPerSecond));
      Logger.recordOutput("Shooter/rotpersec", velocity.in(RotationsPerSecond));
    },
        () -> {
          setVelocity(RadiansPerSecond.of(0));
        });
  }

  public Command setCommandVoltage(Voltage volts) {
    return runEnd(() -> {
      setShooterVoltage(volts);
    }, () -> {
      setShooterVoltage(Volts.of(0));
    });
  }

  public void setShooterVoltage(Voltage volts) {
    closedLoop = false;
    voltsTarget = volts;
  }

  public void setVelocity(AngularVelocity velocity) {
    closedLoop = true;
    velocityTarget = velocity;
    Logger.recordOutput("Shooter/targetVelocityRad", velocity.in(RadiansPerSecond));
  }

  @AutoLogOutput(key = "Shooter/desiredAngularVelocity")
  public AngularVelocity calcBallAngularV(LinearVelocity velocity) {
    return RadiansPerSecond.of(toRPM.get(velocity.in(MetersPerSecond)));
  }

  public LinearVelocity calcBallLinearV(AngularVelocity velocity) {
    return MetersPerSecond.of(toLinear.get(velocity.in(RadiansPerSecond)));
  }

  public LinearVelocity getExitVelocity() {
    return calcBallLinearV(inputs.velocityRadPerSec);
  }
}
