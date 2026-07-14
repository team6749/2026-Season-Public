// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class TurretIOSim implements TurretIO {
  private static final DCMotor turretMotorModel = DCMotor.getKrakenX60(1);

  private final DCMotorSim turretMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(turretMotorModel, 0.03, TurretConstants.gearReduction),
      turretMotorModel);

  public Voltage voltsTarget = Volts.of(0);
  public boolean closedLoop = false;

  public State targetState = new State(0, 0);

  public PIDController positionPID = new PIDController(1, 0, 0);
  public SimpleMotorFeedforward velocityFeedForward = new SimpleMotorFeedforward(0.0, 0.25);

  /** Creates a new TurretIOReal. */
  public TurretIOSim() {
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.angle = Rotations.of(turretMotorSim.getAngularPositionRotations());
    inputs.angularVelocity = RadiansPerSecond.of(turretMotorSim.getAngularVelocityRadPerSec());
    inputs.voltTarget = voltsTarget;

    Logger.recordOutput("Turret/isClosedLoop", closedLoop);
    if(closedLoop) {
          double x = positionPID.calculate(inputs.angle.in(Radians), targetState.position);
    double y = velocityFeedForward.calculate(targetState.velocity);
    
      Logger.recordOutput("Turret/PID", x);
      Logger.recordOutput("Turret/FeedForward", y);

    inputs.voltTarget = Volts.of(x + y);
    turretMotorSim.setInputVoltage(MathUtil.clamp(inputs.voltTarget.in(Volts), -12.0, 12.0));
    } else {
      turretMotorSim.setInputVoltage(voltsTarget.in(Volts));
    }
    turretMotorSim.update(Constants.simulationTimestep.in(Seconds));

  }

  // Something about this implementation feels wrong to me and bug prone. Consider
  // reworking.
  public void setClosedLoopGoal(State goalState) {
    closedLoop = true;
    targetState = goalState;
  }

  public void setVolts(Voltage volts) {
    closedLoop = false;
    voltsTarget = volts;
  }

  public void stop() {
    turretMotorSim.setInputVoltage(0);
  }

}
