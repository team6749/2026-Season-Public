// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spindexer;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class SpindexerIOSim implements SpindexerIO {
  private DCMotorSim shooterMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getNEO(1), 0.007, SpindexerConstants.gearReduction),
      DCMotor.getNEO(1));

  private double appliedVolts = 0;
  private PIDController motorPID = new PIDController(0.2, 0, 0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0.0);
  private AngularVelocity targetVelocity = RadiansPerSecond.of(0);
  private Voltage targetVoltage = Volts.of(0);
  private boolean closedLoop = false;

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    

    inputs.position = Radians.of(shooterMotorSim.getAngularPositionRad());
    inputs.velocity = RadiansPerSecond.of(shooterMotorSim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.currentAmps = Amps.of(shooterMotorSim.getCurrentDrawAmps());

    if (closedLoop) {
      double pid = motorPID.calculate(inputs.velocity.in(RadiansPerSecond),
          targetVelocity.in(RadiansPerSecond));
      appliedVolts = ff.calculate(targetVelocity.in(RadiansPerSecond)) + pid;
      Logger.recordOutput("Spindexer/pid", pid);
    } else {
      appliedVolts = targetVoltage.in(Volts);
    }
    shooterMotorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    shooterMotorSim.update(0.02);

    Logger.recordOutput("Spindexer/TargetVelocity", targetVelocity);
  }

  public void setTargetVolts(Voltage volts) {
    targetVoltage = volts;
    closedLoop = false;
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
    closedLoop = true;
  }
}
