package frc.robot.subsystems.Feeder;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.SparkUtil.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class FeederIOReal implements FeederIO {
  private SparkMax feederMotor = new SparkMax(30, MotorType.kBrushless);
  SendableChooser<Boolean> disableOutput = new SendableChooser<Boolean>();
  SparkMaxConfig config = new SparkMaxConfig();

  private RelativeEncoder encoder = feederMotor.getEncoder();
  private LinearVelocity target = MetersPerSecond.of(0);
  private boolean closedLoop = false;
  private Voltage voltsTarget = Volts.of(0);

  private Voltage feederMotorAppliedVolts = Volts.of(0.0);
  private PIDController feederMotorPID = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 8.5);

  public FeederIOReal() {
    disableOutput.setDefaultOption("enabled", false);
    disableOutput.addOption("disabled", true);
    SmartDashboard.putData("Disable/disableFeeder", disableOutput);

    config.inverted(true);

    feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feederVoltage = feederMotorAppliedVolts;
    Logger.recordOutput("Feeder/target", target);

    ifOk(feederMotor, encoder::getVelocity, (value) -> inputs.linearVelocity = MetersPerSecond
        .of(value / (FeederConstants.gearReduction * 60d) * FeederConstants.magicWheelRatio.in(Meters)));

    if (closedLoop) {
      Voltage pid = Volts.of(feederMotorPID.calculate(inputs.linearVelocity.in(MetersPerSecond),
          target.in(MetersPerSecond)));
      Voltage ff = Volts.of(feedForward.calculate(target.in(MetersPerSecond)));
      feederMotorAppliedVolts = pid.plus(ff);
      if (disableOutput.getSelected() == true) {
        feederMotor.setVoltage(0);
      } else {
        feederMotor.setVoltage(MathUtil.clamp(pid.plus(ff).in(Volts), -12.0, 12.0));
      }
    } else {
      feederMotor.setVoltage(voltsTarget.in(Volts));
    }
  }

  public void setFeederTargetState(LinearVelocity targetLinearVelocity) {
    closedLoop = true;
    target = targetLinearVelocity;
  }

  public void setVoltage(Voltage volts) {
    closedLoop = false;
    voltsTarget = volts;
  }

}