// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private final Debouncer intakeConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorCanId);
  private final StatusSignal<Angle> positionRot = intakeMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = intakeMotor.getVelocity();
  SendableChooser<Boolean> disableOutput = new SendableChooser<Boolean>();
  private final StatusSignal<Voltage> appliedVolts = intakeMotor.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = intakeMotor.getSupplyCurrent();
  private final PIDController motorPid = new PIDController(0.06, 0, 0);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0.06, 0);
  private Voltage voltGoal = Volts.of(0);
  private Voltage pivotVoltGoal = Volts.of(0);
  private boolean closedLoop = false;

  private final TalonFX intakePivotMotor = new TalonFX(IntakeConstants.intakePivotMotorCanId);
  private final StatusSignal<Angle> intakePivotMotorPositionRot = intakePivotMotor.getPosition();
  private final StatusSignal<AngularVelocity> intakePivotMotorVelocityRotPerSec = intakePivotMotor.getVelocity();
  private final StatusSignal<Voltage> intakePivotMotorAppliedVolts = intakePivotMotor.getMotorVoltage();
  private final StatusSignal<Current> intakePivotMotorCurrentAmps = intakePivotMotor.getSupplyCurrent();

  private AngularVelocity targetVelocity = RadiansPerSecond.of(0);

  /** Creates a new IntakeSubsystem. */

  public IntakeIOReal() {
    disableOutput.setDefaultOption("enabled", false);
    disableOutput.addOption("disabled", true);
    SmartDashboard.putData("Disable/disableIntake", disableOutput);

    var intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.intakeMotorCurrentLimit;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeMotorConfig, 0.25));

    var intakePivotMotorConfig = new TalonFXConfiguration();
    intakePivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakePivotMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.intakePivotMotorCurrentLimit;
    intakePivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakePivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> intakePivotMotor.getConfigurator().apply(intakePivotMotorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRot,
        velocityRotPerSec,
        appliedVolts,
        currentAmps,
        intakePivotMotorPositionRot,
        intakePivotMotorVelocityRotPerSec,
        intakePivotMotorAppliedVolts,
        intakePivotMotorCurrentAmps);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, intakePivotMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    Logger.recordOutput("Intake/isClosedLoop", closedLoop);
    intakePivotMotor.setVoltage(pivotVoltGoal.in(Volts));
    StatusCode intakeStatus = BaseStatusSignal.refreshAll(
        positionRot,
        velocityRotPerSec,
        appliedVolts,
        currentAmps,
        intakePivotMotorPositionRot,
        intakePivotMotorVelocityRotPerSec,
        intakePivotMotorAppliedVolts,
        intakePivotMotorCurrentAmps);
    
    inputs.intakeConnected = intakeConnectedDebounce.calculate(intakeStatus.isOK());
    inputs.intakeMotorPosition = Rotations
        .of(positionRot.getValueAsDouble() / IntakeConstants.intakeMotorGearReduction);
    inputs.intakeMotorVelocityRadPerSec = RotationsPerSecond
        .of(velocityRotPerSec.getValueAsDouble() / IntakeConstants.intakeMotorGearReduction);
    inputs.intakeMotorAppliedVolts = Volts.of(appliedVolts.getValueAsDouble());
    inputs.intakeMotorCurrentAmps = Amps.of(currentAmps.getValueAsDouble());
    inputs.intakePivotMotorPosition = Rotations
        .of(intakePivotMotorPositionRot.getValueAsDouble() / IntakeConstants.intakePivotMotorGearReduction);
    inputs.intakePivotMotorVelocityRadPerSec = RotationsPerSecond
        .of(intakePivotMotorVelocityRotPerSec.getValueAsDouble() / IntakeConstants.intakePivotMotorGearReduction);
    inputs.intakePivotMotorAppliedVolts = Volts.of(intakePivotMotorAppliedVolts.getValueAsDouble());
    inputs.intakePivotMotorCurrentAmps = Amps.of(intakePivotMotorCurrentAmps.getValueAsDouble());

    if(disableOutput.getSelected()) {
      intakeMotor.setVoltage(0);
    } else {
      if(closedLoop) {
      double pid = motorPid.calculate(inputs.intakeMotorVelocityRadPerSec.in(RadiansPerSecond),
        targetVelocity.in(RadiansPerSecond));
        voltGoal = Volts.of(ff.calculate(targetVelocity.in(RadiansPerSecond)) + pid);
      }
    intakeMotor.setVoltage(voltGoal.in(Volts));
    }
  }

  public void setIntakeMotorVelocity(AngularVelocity velocity) {
    closedLoop = true;
    targetVelocity = velocity;
  }

  public void setIntakePivotMotorVoltage(Voltage volts) {
    pivotVoltGoal = volts;
  }

  public void setIntakeMotorVoltage(Voltage volts) {
    closedLoop = false;
    voltGoal = volts;
  }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK())
        break;
    }
  }
}
