// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class HoodIOSim implements HoodIO {
  static final double magicStallFactor = 3;
  private DCMotorSim hoodMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getFalcon500(1), 0.004, HoodConstants.gearReduction),
      DCMotor.getFalcon500(1));

  private double hoodMotorAppliedVolts = 0.0;
  private PIDController hoodMotorPID = new PIDController(3, 0, 0);
  public boolean closedLoop = false;
  public Angle target = Radians.of(0);

  public HoodIOSim() {
    hoodMotorSim.setAngle(HoodConstants.maxPosition.in(Radians));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodPosition = Radians.of(hoodMotorSim.getAngularPositionRad());
    inputs.hoodMotorVelocityRadPerSec = RadiansPerSecond.of(hoodMotorSim.getAngularVelocityRadPerSec());
    inputs.hoodMotorAppliedVolts = Volts.of(hoodMotorAppliedVolts);
    inputs.hoodMotorCurrentAmps = Amps.of(hoodMotorSim.getCurrentDrawAmps());

    if (inputs.hoodPosition.gte(HoodConstants.maxPosition)) {
      hoodMotorSim.setState(HoodConstants.maxPosition.in(Radians), 0.0);
      inputs.hoodMotorCurrentAmps = Amps
          .of(hoodMotorSim.getCurrentDrawAmps() * inputs.hoodMotorAppliedVolts.in(Volts) * magicStallFactor);
    }
    if (inputs.hoodPosition.lte(HoodConstants.minPosition)) {
      hoodMotorSim.setState(HoodConstants.minPosition.in(Radians), 0.0);
      inputs.hoodMotorCurrentAmps = Amps
          .of(hoodMotorSim.getCurrentDrawAmps() * inputs.hoodMotorAppliedVolts.in(Volts) * magicStallFactor);
    }
    Logger.recordOutput("Hood/isClosedLoop-SIM", closedLoop);
    if (closedLoop) {
      double pid = hoodMotorPID.calculate(inputs.hoodPosition.in(Radians), target.in(Radians));
      Logger.recordOutput("Hood/Sim-PID", pid);
      hoodMotorSim.setInputVoltage(MathUtil.clamp(pid, -12.0, 12.0));
    } else {
      hoodMotorSim.setInputVoltage(MathUtil.clamp(hoodMotorAppliedVolts, -12.0, 12.0));
    }
    hoodMotorSim.update(0.02);
  }

  public void setHoodMotorVoltage(Voltage voltage) {
    closedLoop = false;
    hoodMotorAppliedVolts = voltage.in(Volts);
  }

  public void setHoodTargetState(Angle position, AngularVelocity velocity) {
    closedLoop = true;
    target = position;
  }

  public void setHoodPositionToHome() {
    hoodMotorSim.setAngle(HoodConstants.homePosition.in(Radians));
    hoodMotorSim.setAngularVelocity(0);
  }
}
