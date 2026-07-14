package frc.robot.subsystems.Feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class FeederIOSim implements FeederIO {
  private DCMotorSim feederMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getNEO(1), 0.004, FeederConstants.gearReduction),
      DCMotor.getNEO(1));

  private Voltage feederMotorAppliedVolts = Volts.of(0.0);
  private PIDController feederMotorPID = new PIDController(2, 0, 0);
  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.75, 0);

  public LinearVelocity target = MetersPerSecond.of(0);

  public FeederIOSim() {
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.angularVelocity = RadiansPerSecond.of(feederMotorSim.getAngularVelocityRadPerSec());
    inputs.feederVoltage = feederMotorAppliedVolts;
    inputs.linearVelocity = MetersPerSecond.of(feederMotorSim.getAngularVelocityRPM() / 60d)
        .times(FeederConstants.magicWheelRatio.in(Meters));

    Voltage pid = Volts.of(feederMotorPID.calculate(inputs.linearVelocity.in(MetersPerSecond),
        target.in(MetersPerSecond)));
    Voltage ff = Volts.of(feedForward.calculate(target.in(MetersPerSecond)));

    feederMotorAppliedVolts = pid.plus(ff);
    feederMotorSim.setInputVoltage(MathUtil.clamp(feederMotorAppliedVolts.in(Volts), -12.0, 12.0));
    feederMotorSim.update(0.02);

  }

  public void setFeederMotorVoltage(Voltage voltage) {

  }

  public void setFeederTargetState(LinearVelocity targetLinearVelocity) {
    target = targetLinearVelocity;

  }

}