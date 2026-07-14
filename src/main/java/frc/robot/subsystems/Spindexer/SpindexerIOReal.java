// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spindexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SpindexerIOReal implements SpindexerIO {
  private final SparkMax spindexerMotor = new SparkMax(SpindexerConstants.motorCanId, MotorType.kBrushless);
  SendableChooser<Boolean> disableOutput = new SendableChooser<Boolean>();
  private final RelativeEncoder encoder = spindexerMotor.getEncoder();
  private PIDController motorPID = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0.0);
  private AngularVelocity targetVelocity = RadiansPerSecond.of(0);
  private Voltage targetVoltage = Volts.of(0);
  private boolean closedLoop = false;

  /** Creates a new IntakeSubsystem. */

  public SpindexerIOReal() {
    disableOutput.setDefaultOption("enabled", false);
    disableOutput.addOption("disabled", true);
    SmartDashboard.putData("Disable/disableSpindexer", disableOutput);
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SpindexerConstants.motorCurrentLimit)
        .voltageCompensation(12.0);
    config.encoder
        .positionConversionFactor(
            1d / SpindexerConstants.gearReduction) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor(1d / (60.0 * SpindexerConstants.gearReduction))
        .uvwMeasurementPeriod(32)
        .uvwAverageDepth(2);
    tryUntilOk(
        spindexerMotor,
        5,
        () -> spindexerMotor.configure(
            config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    //While Yes, the docs say RPM and rotations, I'm pretty sure it's actually in 
    ifOk(spindexerMotor, encoder::getPosition, (value) -> inputs.position = Rotations.of(value));
    ifOk(spindexerMotor, encoder::getVelocity, (value) -> inputs.velocity = RotationsPerSecond.of(value / 60));
    ifOk(
        spindexerMotor,
        new DoubleSupplier[] { spindexerMotor::getAppliedOutput, spindexerMotor::getBusVoltage },
        (values) -> inputs.appliedVolts = Volts.of(values[0] * values[1]));
    ifOk(spindexerMotor, spindexerMotor::getOutputCurrent, (value) -> inputs.currentAmps = Amps.of(value));
    if (disableOutput.getSelected() == true) {
      spindexerMotor.setVoltage(0);
    } else {
      if (closedLoop) {
        double pid = motorPID.calculate(inputs.velocity.in(RadiansPerSecond),
            targetVelocity.in(RadiansPerSecond));
        spindexerMotor.setVoltage(Volts.of(ff.calculate(targetVelocity.in(RadiansPerSecond)) + pid));
        Logger.recordOutput("Spindexer/pid", pid);
      } else {
        spindexerMotor.setVoltage(targetVoltage);
      }
    }
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

  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }
}
