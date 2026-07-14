// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

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
public class ShooterIOSim implements ShooterIO {
  public boolean closedLoop = false;
  private DCMotorSim shooterMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60(2), 0.007, ShooterConstants.gearReduction),
      DCMotor.getKrakenX60(2));

  private double appliedVolts = 0;
  private PIDController motorPID = new PIDController(0.25, 0, 0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0185);
  private AngularVelocity targetVelocity = RadiansPerSecond.of(0);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.position = Radians.of(shooterMotorSim.getAngularPositionRad());
    inputs.velocityRadPerSec = RadiansPerSecond.of(shooterMotorSim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.currentAmps = Amps.of(shooterMotorSim.getCurrentDrawAmps());

    Logger.recordOutput("Shooter/isClosedLoopSIM", closedLoop);
    if (closedLoop) {
      double pid = motorPID.calculate(inputs.velocityRadPerSec.in(RadiansPerSecond),
          targetVelocity.in(RadiansPerSecond));
      appliedVolts = ff.calculateWithVelocities(inputs.velocityRadPerSec.in(RadiansPerSecond),
          targetVelocity.in(RadiansPerSecond)) + pid;
      Logger.recordOutput("Shooter/pid", pid);
      shooterMotorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    } else {
      shooterMotorSim.setInputVoltage(appliedVolts);
    }
    shooterMotorSim.update(0.02);

    Logger.recordOutput("Shooter/TargetVelocityRads", targetVelocity);
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    closedLoop = true;
    targetVelocity = velocity;
  }

  public void setTargetVolts(Voltage volts) {
    closedLoop = false;
    appliedVolts = volts.in(Volts);
  }
}
