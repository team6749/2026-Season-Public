// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.enums.DriveOrientation;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.LocalizationSubsystem.AccelerationData;
import frc.robot.utils.FieldMap;

public class SwerveChassis extends SubsystemBase {

    OdometryConsumer consumer;
    /** Creates a new SwerveChassis. */
    private final SendableChooser<DriveOrientation> orientationChooser = new SendableChooser<>();
    Module[] modules;
    private final SysIdRoutine sysId;
    public ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
    public ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

    public final SwerveDriveKinematics kinematics;

    public SwerveChassis(Module... modules) {
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        assert (modules.length >= 3) : "SwerveChassis requires at least 3 modules.";
        this.modules = modules;

        Translation2d[] moduleLocations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleLocations[i] = modules[i].definition.location;
        }
        kinematics = new SwerveDriveKinematics(moduleLocations);
        orientationChooser.setDefaultOption("Field Oriented", DriveOrientation.FieldOriented);
        orientationChooser.addOption("Robot Oriented", DriveOrientation.RobotOriented);
        SmartDashboard.putData("Drive mode", orientationChooser);
    }

    public void setOdometryConsumer(OdometryConsumer consumer) {
        this.consumer = consumer;
    }

    @Override
    public void periodic() {
        lastChassisSpeeds = getChassisSpeeds();
        getExpectedAcceleration();
        currentChassisSpeeds = getChassisSpeeds();

        // disabled logging.
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveChassisSpeeds/Setpoints", new ChassisSpeeds());
        }

        consumer.accept(getModulePositions(), Timer.getTimestamp());

    }

    public Command basicDriveCommand(CommandXboxController controller, Supplier<Pose2d> getRobotPose, FieldMap map,
            Hood hood) {

        Command command = Commands.runEnd(() -> {
            boolean isInDangerousHubLocation = map.getRegion("hub_vibrate_warning")
                    .contains(getRobotPose.get().getTranslation()) ||
                    map.getRegion("hub_vibrate_warning_2").contains(getRobotPose.get().getTranslation());
            Logger.recordOutput("DriveCommand/Is in hub drop zone", isInDangerousHubLocation);
            boolean isInHoodRestrictedLocation = hood.isInTrench();

            controller.setRumble(RumbleType.kBothRumble,
                    isInDangerousHubLocation ? 1 : isInHoodRestrictedLocation ? 0.1 : 0);
            Logger.recordOutput("DriveCommand/Is in trench", isInHoodRestrictedLocation);

            double xInput = -controller.getLeftY();
            double yInput = -controller.getLeftX();
            double zInput = -controller.getRightX();

            if (inputMagnitude(xInput, yInput) < SwerveConstants.deadZone) {
                xInput = 0;
                yInput = 0;
            }
            if (inputMagnitude(zInput, 0) < SwerveConstants.deadZone) {
                zInput = 0;
            }

            Rotation2d inputRotation;
            if (xInput == 0 && yInput == 0) {
                inputRotation = Rotation2d.kZero;
            } else {
                inputRotation = new Rotation2d(xInput, yInput);
            }
            double curvedMagnitude = exponentialResponseCurve(inputMagnitude(xInput, yInput));
            double xSpeed = curvedMagnitude * inputRotation.getCos();
            double ySpeed = curvedMagnitude * inputRotation.getSin();
            LinearVelocity currentMaxLinearVelocity;
            if (controller.x().getAsBoolean()) {
                currentMaxLinearVelocity = SwerveConstants.superLinearVelocity;
            } else {
                currentMaxLinearVelocity = SwerveConstants.maxLinearVelocity;
            }
            ChassisSpeeds targetSpeeds = new ChassisSpeeds(
                    MetersPerSecond.of(SwerveConstants.driveLimiterY.calculate(currentMaxLinearVelocity
                            .times(xSpeed).in(MetersPerSecond))),
                    MetersPerSecond.of(SwerveConstants.driveLimiterX.calculate(currentMaxLinearVelocity
                            .times(ySpeed).in(MetersPerSecond))),
                    RadiansPerSecond.of(SwerveConstants.driveLimiterTheta.calculate(SwerveConstants.maxAngularVelocity
                            .times(exponentialResponseCurve(zInput)).in(RadiansPerSecond))));
            var x = orientationChooser.getSelected() != null ? orientationChooser.getSelected()
                    : DriveOrientation.RobotOriented;

            if (controller.rightBumper().getAsBoolean()) {
                x = DriveOrientation.RobotOriented;
            }
            switch (x) {
                case FieldOriented:
                    Rotation2d robotRotation2d = getRobotPose.get().getRotation();
                    targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds,
                            DriverStation.getAlliance().get() == Alliance.Blue ? robotRotation2d
                                    : robotRotation2d.plus(Rotation2d.fromDegrees(180)));

                    break;
                default: // we should to implement regular drive.
                    break;
            }

            // Desaturate the input
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, currentMaxLinearVelocity);
            setModuleStates(states);

        }, () -> {
            stop();
        }, this);
        command.setName("Base Drive Command");
        return command;
    }

    public double inputMagnitude(double x, double y) {
        return Math.sqrt((x * x) + (y * y));
    }

    // reduce jumpiness. Can change value if needed.
    public double exponentialResponseCurve(double input) {
        return Math.pow(input, 3);
    }

    // Sends states over to modules. setClosedLoopGoal is MechAd's runSetpoint.
    public void setModuleStates(SwerveModuleState[] states) {
        Logger.recordOutput("SwerveStates/Setpoints", states);
        for (int i = 0; i < states.length; i++) {
            modules[i].setClosedLoopGoal(states[i]);
        }
    }

    // Sends states over to modules. setClosedLoopGoal is MechAd's runSetpoint.
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getMeasuredModuleStates());
    }

    public void stop() {
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", new ChassisSpeeds());
        for (Module module : modules) {
            module.stop();
        }
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getMeasuredModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    // single track implementation of our drive command. I.e, uses closed loop(ff
    // and PID) but skips auto generation of states.
    public Command constantChassisSpeedsCommand(ChassisSpeeds speeds) {
        Command command = Commands.runEnd(() -> {
            runChassisSpeeds(speeds);
        }, () -> {
            stop();
        }, this);
        command.setName("Constant Chasssis Speeds Command");
        return command;
    }

    public void runChassisSpeeds(ChassisSpeeds speeds) {
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);
        // Not sure how useful this actually is.
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

        setModuleStates(setpointStates);
    }

    public AccelerationData getExpectedAcceleration() {
        ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();
        return new AccelerationData(
                MetersPerSecondPerSecond
                        .of((currentChassisSpeeds.vxMetersPerSecond - lastChassisSpeeds.vxMetersPerSecond)),
                MetersPerSecondPerSecond
                        .of((currentChassisSpeeds.vyMetersPerSecond - lastChassisSpeeds.vyMetersPerSecond)),
                MetersPerSecondPerSecond.of(0),
                RadiansPerSecondPerSecond
                        .of((currentChassisSpeeds.omegaRadiansPerSecond - lastChassisSpeeds.omegaRadiansPerSecond)));
    }

    @FunctionalInterface
    public static interface OdometryConsumer {
        public void accept(
                SwerveModulePosition[] OdometryPositions,
                double time);
    }

    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the position of each module in meters. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

}
