// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputFilter.Status;
import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOReal implements ShooterIO {
  SendableChooser<Boolean> disableOutput = new SendableChooser<Boolean>();
  private final Debouncer shooterConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.motorCanId);// Left
  private final TalonFX shooterMotorMirrored = new TalonFX(ShooterConstants.motorCanIdMirrored);// Right
  private Voltage voltTarget = Volts.of(0);
  private boolean closedLoop = false;
  private boolean isEnabled = false;
  private final StatusSignal<Angle> positionRot = shooterMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = shooterMotor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = shooterMotor.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = shooterMotor.getSupplyCurrent();
  private PIDController motorPID = new PIDController(0.02, 0, 0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.023, 0.019);
  private AngularVelocity targetVelocity = RadiansPerSecond.of(0);
  //Supposedly I can do these in line. Too lazy
  public double[] inputBuffer = new double[999];
  public double[] outputBuffer = new double[0];


  LinearFilter voltageAverage  = LinearFilter.movingAverage(999);
  InterpolatingDoubleTreeMap voltageSagGainMap = new InterpolatingDoubleTreeMap();

  // private final VoltageOut voltageRequest = new VoltageOut(0.0);
  /** Creates a new IntakeSubsystem. */

  public ShooterIOReal() {
    disableOutput.setDefaultOption("enabled", false);
    disableOutput.addOption("disabled", true);
    SmartDashboard.putData("Disable/disableShooter", disableOutput);
    var shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.motorCurrentLimit;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(shooterMotorConfig, 0.25));
    tryUntilOk(5, () -> shooterMotorMirrored.getConfigurator().apply(shooterMotorConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRot,
        velocityRotPerSec,
        appliedVolts,
        currentAmps);
    ParentDevice.optimizeBusUtilizationForAll(shooterMotor);
    ParentDevice.optimizeBusUtilizationForAll(shooterMotorMirrored);
    
    voltageSagGainMap.put(13.0, 1.0);
    voltageSagGainMap.put(10.0, 1.17);

  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double averageBatteryVoltage = voltageAverage.calculate(RobotController.getBatteryVoltage());
    Logger.recordOutput("AvgBatteryVoltage", averageBatteryVoltage);
    double gain = voltageSagGainMap.get(averageBatteryVoltage);
    Logger.recordOutput("Shooter/BatteryGain", gain);
    if(DriverStation.isEnabled() && !isEnabled) {
      var inputBuffer = new double[999];
      Arrays.fill(inputBuffer, RobotController.getBatteryVoltage());
      voltageAverage.reset(inputBuffer, new double[0]);
      isEnabled = true;
    }

    StatusCode shooterStatus = BaseStatusSignal.refreshAll(
        positionRot,
        velocityRotPerSec,
        appliedVolts,
        currentAmps);
    inputs.shooterConnected = shooterConnectedDebounce.calculate(shooterStatus.isOK());
    Logger.recordOutput("Shooter/Target", targetVelocity);

    inputs.position = Rotations.of(positionRot.getValueAsDouble() * ShooterConstants.gearReduction);
    inputs.velocityRadPerSec = RotationsPerSecond
        .of(velocityRotPerSec.getValueAsDouble() * ShooterConstants.gearReduction);
    inputs.appliedVolts = Volts.of(appliedVolts.getValueAsDouble());
    inputs.currentAmps = Amps.of(currentAmps.getValueAsDouble());
    Logger.recordOutput("Shooter/isClosedLoopReal", closedLoop);
    if (disableOutput.getSelected() == true) {
      shooterMotor.set(0);
      shooterMotorMirrored.set(0);
    } else {
            if(closedLoop) {
      double pid = motorPID.calculate(inputs.velocityRadPerSec.in(RadiansPerSecond), targetVelocity.in(RadiansPerSecond));
      double volts = ff.calculate(targetVelocity.in(RadiansPerSecond)) + pid;
      shooterMotor.setVoltage(volts * gain);
      shooterMotorMirrored.setVoltage(-volts * gain);
      } else {
        shooterMotor.setVoltage(voltTarget.in(Volts) * gain);
        shooterMotor.setVoltage(voltTarget.in(Volts) * gain);
      }
    }
  }

  public void setVoltage(Voltage volts) {
    closedLoop = false;
    voltTarget = volts;
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    closedLoop = true;
    targetVelocity = velocity;
  }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK())
        break;
    }
  }
}
