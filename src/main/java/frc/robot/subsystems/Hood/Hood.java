// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ComponentVisualizer;
import frc.robot.Constants;
import frc.robot.utils.FieldMap;

public class Hood extends SubsystemBase {

  private final Alert hoodDisconnectedAlert;
  /** Creates a new Intake. */
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final FieldMap map;
  private final Supplier<Translation3d> supplier;

  private boolean isHomed = false;
  public Voltage voltsTarget = Volts.of(0);
  Debouncer m_debouncer = new Debouncer(0.1, DebounceType.kRising);

  AngularVelocity maxVelocity = DegreesPerSecond.of(400);
  boolean closedLoop = false;
  AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(400);
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      maxVelocity.in(RadiansPerSecond), maxAcceleration.in(RadiansPerSecondPerSecond)));
  TrapezoidProfile.State targetState = new State(HoodConstants.minPosition.in(Radians), 0);
  TrapezoidProfile.State setpointState = targetState;
  private final ComponentVisualizer visualizer;

  public Hood(HoodIO io, FieldMap map, Supplier<Translation3d> supplier, ComponentVisualizer visualizer) {
    this.io = io;
    this.map = map;
    this.supplier = supplier;
    this.visualizer = visualizer;

    hoodDisconnectedAlert = new Alert(
        "Disconnected hood motor.",
        AlertType.kError);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    visualizer.setHood(inputs.hoodPosition);
    hoodDisconnectedAlert.set(!inputs.hoodConnected);

    if (DriverStation.isEnabled()) {
      if (!isHomed) {
        io.setHoodMotorVoltage(Volts.of(-0.5));
        boolean isStalled = inputs.hoodMotorCurrentAmps.abs(Amps) > 1;
        if (m_debouncer.calculate(isStalled)) {
          isHomed = true;
          io.setHoodMotorVoltage(Volts.of(0));
          io.setHoodPositionToHome();
        }
      } else {
        if (targetState.position > HoodConstants.maxPosition.in(Radians)) {
          targetState = new State((HoodConstants.maxPosition).in(Radians), 0.0);
          System.err.println("Target is above maximum!");
        }
        if (targetState.position < HoodConstants.minPosition.in(Radians)) {
          targetState = new State((HoodConstants.minPosition).in(Radians), 0.0);
          System.err.println("Target is below minimum!");
        }
        if (isInTrench()) {
          targetState = new State(HoodConstants.minPosition.in(Radians), 0);
        }

        Logger.recordOutput("Hood/isClosedLoop-Hood", closedLoop);
        if (closedLoop) {
          var nextState = trapezoidProfile.calculate(Constants.simulationTimestep.in(Seconds), setpointState,
              targetState);
          io.setHoodTargetState(Radians.of(setpointState.position), RadiansPerSecond.of(setpointState.velocity));
          setpointState = nextState;
        } else {
          io.setHoodMotorVoltage(voltsTarget);
        }
      }
    } else {
      m_debouncer.calculate(false);
    }
    Logger.recordOutput("Hood/SetPoint", setpointState);
    Logger.recordOutput("Hood/Target", targetState);
    Logger.recordOutput("Hood/IsHomed", isHomed);
  }

  public Command setHoodAngleCommand(Angle target) {
    return Commands.runEnd(() -> {
      setHoodAngle(target);
    },
        () -> {
          setHoodAngle(inputs.hoodPosition);
        }, this);
  }

  public void setHoodAngle(Angle target) {
    closedLoop = true;
    targetState = new State(target.in(Radians), 0);
  }

  public Angle getAngle() {
    return inputs.hoodPosition;
  }

  public Command setHoodVoltCommand(Voltage volts) {
    return Commands.run(() -> {
      setHoodVolts(volts);
    }, this);
  }

  public void setHoodVolts(Voltage volts) {
    closedLoop = false;
    voltsTarget = volts;
  }

  public Boolean isInTrench() {
    Translation3d turret3d = supplier.get();
    Translation2d turretPosition = new Translation2d(turret3d.getX(), turret3d.getY());
    Logger.recordOutput("Hood/TurretPos", turret3d);
    return map.getRegion("hood/intake_restricted_1").contains(turretPosition)
        || map.getRegion("hood/intake_restricted_2").contains(turretPosition)
        || map.getRegion("hood/intake_restricted_3").contains(turretPosition)
        || map.getRegion("hood/intake_restricted_4").contains(turretPosition);
  }

}
