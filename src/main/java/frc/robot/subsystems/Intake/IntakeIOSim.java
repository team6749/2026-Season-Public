// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, IntakeConstants.intakeMotorGearReduction),
      DCMotor.getKrakenX60(1));
  private DCMotorSim intakePivotMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60(1), 0.004, IntakeConstants.intakePivotMotorGearReduction),
      DCMotor.getKrakenX60(1));

  private double intakeMotorAppliedVolts = 0.0;
  private PIDController mpid = new PIDController(0.01, 0, 0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, 0);
  private double intakePivotMotorAppliedVolts = 0.0;
  private AngularVelocity targetVelocity = RadiansPerSecond.of(0);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    

    inputs.intakeMotorPosition = Radians.of(intakeMotorSim.getAngularPositionRad());
    inputs.intakeMotorVelocityRadPerSec = RadiansPerSecond.of(intakeMotorSim.getAngularVelocityRadPerSec());
    inputs.intakeMotorAppliedVolts = Volts.of(intakeMotorAppliedVolts);
    inputs.intakeMotorCurrentAmps = Amps.of(intakeMotorSim.getCurrentDrawAmps());

    inputs.intakePivotMotorPosition = Radians.of(intakePivotMotorSim.getAngularPositionRad());
    inputs.intakePivotMotorVelocityRadPerSec = RadiansPerSecond.of(intakePivotMotorSim.getAngularVelocityRadPerSec());
    inputs.intakePivotMotorAppliedVolts = Volts.of(intakePivotMotorAppliedVolts);
    inputs.intakePivotMotorCurrentAmps = Amps.of(intakePivotMotorSim.getCurrentDrawAmps());

    double pid = mpid.calculate(inputs.intakeMotorVelocityRadPerSec.in(RadiansPerSecond),
        targetVelocity.in(RadiansPerSecond));
    intakeMotorAppliedVolts = ff.calculate(targetVelocity.in(RadiansPerSecond)) + pid;

    intakeMotorSim.setInputVoltage(intakeMotorAppliedVolts);
    intakeMotorSim.update(0.02);

    intakePivotMotorSim.setInputVoltage(intakePivotMotorAppliedVolts);
    intakePivotMotorSim.update(0.02);
    Logger.recordOutput("Intake/targetVelocity", targetVelocity);
  }

  public void setIntakeMotorVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
  }

  public void setIntakePivotMotorVoltage(Voltage volts) {
    intakePivotMotorAppliedVolts = volts.in(Volts);
  }
}
