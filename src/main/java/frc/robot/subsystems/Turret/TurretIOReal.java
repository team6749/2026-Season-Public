// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretIOReal implements TurretIO {
  public TalonFX turretMotor;
  SendableChooser<Boolean> disableOutput = new SendableChooser<Boolean>();
  public Voltage voltsTarget = Volts.of(0);
  public boolean closedLoop = false;
  private final Debouncer turretConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private StatusSignal<Angle> turretMotorPositionRot;
  private StatusSignal<AngularVelocity> turretMotorVelocityRotPerSec;
  private StatusSignal<Voltage> turretMotorAppliedVolts;
  private StatusSignal<Current> turretMotorCurrentAmps;

  public State targetState = new State(0, 0);
  public State PIDTargetState = new State();

  public PIDController positionPID = new PIDController(4.7, 0, 0.0); // 5.3
  public SimpleMotorFeedforward velocityFeedForward = new SimpleMotorFeedforward(0.2, 0.185, 0.065); // 0.27, 0.07, KA
                                                                                                     // is scary.
                                                                                                     // Resolves phase
                                                                                                     // delay though

  /** Creates a new TurretIOReal. */
  public TurretIOReal(int turretPort) {
    disableOutput.setDefaultOption("enabled", false);
    disableOutput.addOption("disabled", true);
    SmartDashboard.putData("Disable/disableTurret", disableOutput);
    turretMotor = new TalonFX(turretPort);

    turretMotorCurrentAmps = turretMotor.getSupplyCurrent();
    turretMotorAppliedVolts = turretMotor.getMotorVoltage();
    turretMotorVelocityRotPerSec = turretMotor.getVelocity();
    turretMotorPositionRot = turretMotor.getPosition();

    var turretMotorConfig = new TalonFXConfiguration();
    turretMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.motorCurrentLimit;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretMotorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        turretMotor.getPosition(),
        turretMotor.getVelocity(),
        turretMotor.getMotorVoltage());

    turretMotor.setPosition(0);

    ParentDevice.optimizeBusUtilizationForAll(turretMotor);
  }

  public void setClosedLoopGoal(State goalState) {
    closedLoop = true;
    targetState = goalState;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    StatusCode turretStatus = BaseStatusSignal.refreshAll(turretMotorPositionRot, turretMotorCurrentAmps,
        turretMotorAppliedVolts, turretMotorVelocityRotPerSec);
    inputs.turretConnected = turretConnectedDebounce.calculate(turretStatus.isOK());
    inputs.angle = Rotations.of(turretMotor.getPosition().getValueAsDouble() / TurretConstants.gearReduction);
    inputs.angularVelocity = RotationsPerSecond
        .of(turretMotor.getVelocity().getValueAsDouble() / TurretConstants.gearReduction);
    inputs.amps = Amps.of(turretMotorCurrentAmps.getValueAsDouble());

    Logger.recordOutput("Turret/isClosedLoop", closedLoop);
    if (closedLoop) {

      double pid = positionPID.calculate(inputs.angle.in(Radians), PIDTargetState.position);
      double ff = velocityFeedForward.calculate(targetState.velocity);

      Logger.recordOutput("Turret/PID", pid);
      Logger.recordOutput("Turret/FeedForward", ff);

      voltsTarget = Volts.of(ff + pid);
    }
    if (disableOutput.getSelected() == true) {
      turretMotor.setVoltage(0);
    } else {
      turretMotor.setVoltage(voltsTarget.in(Volts));
    }
    PIDTargetState = targetState;
    inputs.voltTarget = voltsTarget;
  }

  public void stop() {
    turretMotor.stopMotor();
  }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK())
        break;
    }
  }

  public void setVolts(Voltage volts) {
    closedLoop = false;
    voltsTarget = volts;
  }
}
