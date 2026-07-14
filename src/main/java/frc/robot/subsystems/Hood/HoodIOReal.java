// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HoodIOReal implements HoodIO {
  private final Debouncer hoodConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.motorCanId);
  SendableChooser<Boolean> disableOutput = new SendableChooser<Boolean>();
  public Voltage voltTarget = Volts.of(0);
  private final StatusSignal<Angle> hoodMotorPositionRot = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodMotorVelocityRotPerSec = hoodMotor.getVelocity();
  private final StatusSignal<Voltage> hoodMotorAppliedVolts = hoodMotor.getMotorVoltage();
  private final StatusSignal<Current> hoodMotorCurrentAmps = hoodMotor.getSupplyCurrent();
  private final PIDController hoodMotorPID = new PIDController(5.75, 0, 0);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0.51);
  public Angle target = Radians.of(0);
  public AngularVelocity targetVelo = RadiansPerSecond.of(0);
  public boolean closedLoop = false;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  /** Creates a new IntakeSubsystem. */

  public HoodIOReal() {
    disableOutput.setDefaultOption("enabled", false);
    disableOutput.addOption("disabled", true);
    SmartDashboard.putData("Disable/disableHood", disableOutput);

    var hoodMotorConfig = new TalonFXConfiguration();
    hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.motorCurrentLimit;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodMotorConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        hoodMotorPositionRot,
        hoodMotorVelocityRotPerSec,
        hoodMotorAppliedVolts,
        hoodMotorCurrentAmps);
    ParentDevice.optimizeBusUtilizationForAll(hoodMotor);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    StatusCode hoodStatus = BaseStatusSignal.refreshAll(
        hoodMotorPositionRot,
        hoodMotorVelocityRotPerSec,
        hoodMotorAppliedVolts,
        hoodMotorCurrentAmps);

    inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());
    inputs.hoodPosition = Rotations.of(hoodMotorPositionRot.getValueAsDouble() / HoodConstants.gearReduction);
    inputs.hoodMotorVelocityRadPerSec = RotationsPerSecond
        .of(hoodMotorVelocityRotPerSec.getValueAsDouble() / HoodConstants.gearReduction);
    inputs.hoodMotorAppliedVolts = Volts.of(hoodMotorAppliedVolts.getValueAsDouble());
    inputs.hoodMotorCurrentAmps = Amps.of(hoodMotorCurrentAmps.getValueAsDouble());

    if(disableOutput.getSelected()) {
      hoodMotor.setVoltage(0);
    } else {
    if (closedLoop) {
      double pid = -hoodMotorPID.calculate(target.in(Radians), inputs.hoodPosition.in(Radians));
      double ffCalc = ff.calculate(targetVelo.in(RadiansPerSecond));
      hoodMotor.setVoltage(pid + ffCalc);
    } else {
      hoodMotor.setVoltage(voltTarget.in(Volts));
    }
  }
  }

  public void setHoodMotorVoltage(Voltage volts) {
    closedLoop = false;
    voltTarget = volts;
  }

  public void setHoodTargetState(Angle targetAngle, AngularVelocity targetAngularVelocity) {
    closedLoop = true;
    target = targetAngle;
    targetVelo = targetAngularVelocity;
  }

  public void setHoodPositionToHome() {
    hoodMotor.setPosition(HoodConstants.homePosition.in(Rotations) * HoodConstants.gearReduction);
    Logger.recordOutput("Hood/homed", HoodConstants.homePosition.in(Rotations) * HoodConstants.gearReduction);
  }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK())
        break;
    }
  }
}
