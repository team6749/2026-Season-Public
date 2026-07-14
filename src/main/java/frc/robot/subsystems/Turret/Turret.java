// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
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

public class Turret extends SubsystemBase {
  private final Alert turretDisconnectedAlert;
  boolean closedLoop = false;
  Voltage voltsTarget = Volts.of(0);

  State targetState = new State(0, 0);
  State setPointState = targetState;

  TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  TrapezoidProfile profile = new TrapezoidProfile(
      new Constraints(TurretConstants.maxTurretVelo.in(RadiansPerSecond),
          TurretConstants.maxTurretAccel.in(RadiansPerSecondPerSecond))); // Put contraints on max accel and max velo
  private final ComponentVisualizer visualizer;

  /** Creates a new Turret. */
  public Turret(TurretIO io, ComponentVisualizer visualizer) {
    turretDisconnectedAlert = new Alert(
        "Disconnected turret motor.",
        AlertType.kError);
    this.io = io;
    this.visualizer = visualizer;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("turret", inputs);
    turretDisconnectedAlert.set(!inputs.turretConnected);

    visualizer.setTurret(inputs.angle);

    if (DriverStation.isEnabled()) {
      // TODO handle wrap around adjust target state

      // TODO use current turret state instead of the setpointState (to handle real
      // world conditions)
      // Presuming this uses and returns radians.
      if (closedLoop) {
        setPointState = profile.calculate(Constants.simulationTimestep.in(Seconds),
            setPointState, targetState);
            new State(inputs.angle.in(Radians), inputs.angularVelocity.in(RadiansPerSecond));
        if (setPointState.position > TurretConstants.maxAngle.in(Radians)) {
          setPointState = new State(TurretConstants.maxAngle.in(Radians), 0);
        }
        if (setPointState.position < TurretConstants.minAngle.in(Radians)) {
          setPointState = new State(TurretConstants.minAngle.in(Radians), 0);
        }
        if (!inputs.angle.isNear(Radians.of(setPointState.position), Degrees.of(90))) {
          System.err.println("WARNING: The inputs are too far away!");
          setPointState = new State(inputs.angle.in(Radians), inputs.angularVelocity.in(RadiansPerSecond));
        }
        io.setClosedLoopGoal(setPointState);
      } else {
        io.setVolts(voltsTarget);
      }
    } else {
      io.stop();
    }

    Logger.recordOutput("Turret/TargetState", targetState);
    Logger.recordOutput("Turret/NextState", setPointState);
  }

  public Command setTargetHardwareStateCommand(Rotation2d angle) {
    // angle is a value between -180 and 180 that is robot relative
    Command command = Commands.runEnd(() -> {
      setTargetHardwareState(angle, RadiansPerSecond.of(0));
    }, () -> io.stop(), this);
    command.setName("Turret/StateSetter");
    return command;
  }

  public Command setVoltsTarget(Voltage volts) {
    return Commands.runEnd(() -> {
      closedLoop = false;
      voltsTarget = volts;
    }, () -> {
      voltsTarget = Volts.of(0);
    }, this);
  }

  public void setTargetHardwareState(Rotation2d targetYaw, AngularVelocity velocity) {
    closedLoop = true;
    targetState = new State(wrapAroundHandle(targetYaw).in(Radians), velocity.in(RadiansPerSecond));
  }

  public void setTargetRobotRelativeState(Rotation2d targetYaw, AngularVelocity velocity) {
    closedLoop = true;
    targetState = new State(
        robotAngleToHardwareAngle(Rotation2d.fromRadians(wrapAroundHandle(targetYaw).in(Radians))).getRadians(),
        velocity.in(RadiansPerSecond));
  }

  // convert to radians later
  private Angle wrapAroundHandle(Rotation2d inAngle) {
    return Radians
        .of(((inAngle.getRadians() + Degrees.of(180).in(Radians)) % (2 * Math.PI) - Degrees.of(180).in(Radians)));
  }

  public Rotation2d getHardwareAngle() {
    return Rotation2d.fromRadians(inputs.angle.in(Radians));
  }

  public Rotation2d getRobotRelativeAngle() {
    return hardwareRelativeToRobotRelativeAngle(getHardwareAngle());
  }

  private Rotation2d hardwareRelativeToRobotRelativeAngle(Rotation2d hardwareInput) {
    return hardwareInput.plus(TurretConstants.turretAxisOffset);
  }

  private Rotation2d robotAngleToHardwareAngle(Rotation2d robotRelativeInput) {
    return robotRelativeInput.minus(TurretConstants.turretAxisOffset);
  }
}
