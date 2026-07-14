// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Drive.Module;
import frc.robot.subsystems.Drive.ModuleIO;
import frc.robot.subsystems.Drive.SwerveChassis;
import frc.robot.subsystems.Drive.SwerveConstants;
import frc.robot.subsystems.Drive.SwerveReal;
import frc.robot.subsystems.Drive.SwerveSim;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Feeder.FeederIO;
import frc.robot.subsystems.Feeder.FeederIOReal;
import frc.robot.subsystems.Feeder.FeederIOSim;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOReal;
import frc.robot.subsystems.Turret.TurretIOSim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.LocalizationSubsystem.GyroIO;
import frc.robot.subsystems.LocalizationSubsystem.GyroIOReal;
import frc.robot.subsystems.LocalizationSubsystem.GyroIOSim;
import frc.robot.subsystems.LocalizationSubsystem.Localization;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.subsystems.Hood.HoodIOReal;
import frc.robot.subsystems.Hood.HoodIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShootingCommand.TargetParameters;
import frc.robot.ShootingCommand.ShootExport;
import frc.robot.Ballistics.BallisticSimulator;
import frc.robot.Ballistics.BallisticSimulator.ProjectileTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.Spindexer.SpindexerIO;
import frc.robot.subsystems.Spindexer.SpindexerIOReal;
import frc.robot.subsystems.Spindexer.SpindexerIOSim;
import frc.robot.utils.MatchTimer;
import frc.robot.utils.FieldMap;
import frc.robot.utils.HubTimer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  private final Turret turret;
  // The robot's subsystems and commands are defined here...
  private final Intake intake;
  private final Shooter shooter;
  private final ComponentVisualizer componentVisualizer;
  private final Feeder feeder;
  private final Spindexer spindexer;
  // terrible feeling implementation
  
  private final FieldMap map = new FieldMap("field_map.json");
  private final ShootingCommand shootingCommand = new ShootingCommand(map);

  private final SwerveChassis swerveChassis;
  private final Localization localization;
  private final Vision vision;
  private final Hood hood;
  private final SendableChooser<String> autoShootChooser = new SendableChooser<String>();

  private HubTimer hubTimer;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final List<Pair<Double, Double>> shooterPairs = List.of(
      new Pair<Double, Double>(0.0, 0.0),
      new Pair<Double, Double>(257d, 4.88d),
      new Pair<Double, Double>(360d, 7.61d),
      new Pair<Double, Double>(462d, 10.55d),
      new Pair<Double, Double>(565d, 13.40d));


  public RobotContainer(MatchTimer timer) {
    autoShootChooser.setDefaultOption("Auto Aim, Auto Feed", "auto");
    autoShootChooser.addOption("Auto Aim, Manual Feed", "semiAuto");
    autoShootChooser.addOption("Fixed Aim, Manual Feed", "manual");
    SmartDashboard.putData("AutoShootOption", autoShootChooser);
    componentVisualizer = new ComponentVisualizer();
    hubTimer = new HubTimer(timer);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        intake = new Intake(new IntakeIOReal());
        swerveChassis = new SwerveChassis(
            new Module(new SwerveReal(7, 8, 9), SwerveConstants.FLModule),
            new Module(new SwerveReal(1, 2, 3), SwerveConstants.FRModule),
            new Module(new SwerveReal(4, 5, 6), SwerveConstants.BLModule),
            new Module(new SwerveReal(10, 11, 12), SwerveConstants.BRModule));
        turret = new Turret(new TurretIOReal(26), componentVisualizer);

        shooter = new Shooter(new ShooterIOReal(), shooterPairs);

        localization = new Localization(new GyroIOReal(), swerveChassis, swerveChassis::getChassisSpeeds);

        spindexer = new Spindexer(new SpindexerIOReal());
        feeder = new Feeder(new FeederIOReal());
        hood = new Hood(new HoodIOReal(), map, localization::getTurretFieldPose, componentVisualizer);

        vision = new Vision(localization::addVisionMeasurement,
            new VisionIOLimelight("limelight-wilbur"),
            new VisionIOLimelight("limelight-wilson"),
            new VisionIOLimelight("limelight-dilbert"));

        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        intake = new Intake(new IntakeIOSim());
        swerveChassis = new SwerveChassis(
            new Module(new SwerveSim(), SwerveConstants.FLModule),
            new Module(new SwerveSim(), SwerveConstants.FRModule),
            new Module(new SwerveSim(), SwerveConstants.BLModule),
            new Module(new SwerveSim(), SwerveConstants.BRModule));

        turret = new Turret(new TurretIOSim(), componentVisualizer);
        shooter = new Shooter(new ShooterIOSim(), shooterPairs);

        localization = new Localization(new GyroIOSim(swerveChassis::getChassisSpeeds), swerveChassis,
            swerveChassis::getChassisSpeeds);
        feeder = new Feeder(new FeederIOSim());
        spindexer = new Spindexer(new SpindexerIOSim());

        hood = new Hood(new HoodIOSim(), map, localization::getTurretFieldPose, componentVisualizer);
        vision = new Vision(localization::addVisionMeasurement,
            new VisionIO() {
            },
            new VisionIO() {
            },
            new VisionIO() {
            });
        // Implement vision simulator or find a way to not have it. This is a hollow
        // implementation NOT a simulator.
        break;
      default:
        // Replayed robot, disable IO implementations
        intake = new Intake(new IntakeIO() {
        });
        // (Use same number of dummy implementations as the real robot)'
        // not sure why the dummy implementation is the real local system.
        swerveChassis = new SwerveChassis(
            new Module(new ModuleIO() {
            }, SwerveConstants.FLModule),
            new Module(new ModuleIO() {
            }, SwerveConstants.FRModule),
            new Module(new ModuleIO() {
            }, SwerveConstants.BLModule),
            new Module(new ModuleIO() {
            }, SwerveConstants.BRModule));

        turret = new Turret(new TurretIO() {
        }, componentVisualizer);
        // Replayed robot, disable IO implementations

        shooter = new Shooter(new ShooterIO() {
        }, shooterPairs);
        localization = new Localization(new GyroIO() {
        }, swerveChassis, swerveChassis::getChassisSpeeds);
        vision = new Vision(localization::addVisionMeasurement,
            new VisionIO() {
            },
            new VisionIO() {
            },
            new VisionIO() {
            });

        hood = new Hood(new HoodIO() {
        }, map, localization::getTurretFieldPose, componentVisualizer);
        feeder = new Feeder(new FeederIO() {
        });
        spindexer = new Spindexer(new SpindexerIO() {
        });
        // Replayed robot, disable IO implementations
        break;

    }
    NamedCommands.registerCommand("WaitIntake", intake.waitForIntakeDeploy());
    NamedCommands.registerCommand("AutoShoot", autoShoot());
    NamedCommands.registerCommand("FixedShoot", fixedShoot());

    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          localization::getFieldPose, // Robot pose supplier
          localization::resetPose, // Method to reset odometry (will be called if your auto has a starting
                                   // pose)
          swerveChassis::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> swerveChassis.runChassisSpeeds(speeds), // Method that will drive the robot given
                                                                            // ROBOT RELATIVE ChassisSpeeds. Also
                                                                            // optionally outputs individual module
                                                                            // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(4.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          swerveChassis // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autoChooser", autoChooser);

    // Configure the trigger bindings
    // driveTest()
    configureBindings();
    testCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    swerveChassis
        .setDefaultCommand(swerveChassis.basicDriveCommand(m_driverController, localization::getFieldPose, map, hood));

    Trigger shootButton = m_driverController.b();
    Trigger autoShootButton = m_driverController.x();

    // cursed because triggers only operate on the edge
    new Trigger(() -> autoShootChooser.getSelected() == "auto").and(autoShootButton.negate()).and(() -> DriverStation.isDisabled() == false).whileTrue(autoShoot());
    new Trigger(() -> autoShootChooser.getSelected() == "semiAuto").and(autoShootButton.negate()).and(() -> DriverStation.isDisabled() == false).whileTrue(autoAim());
    new Trigger(() -> autoShootChooser.getSelected() == "manual").whileTrue(fixedAim());
    shootButton.whileTrue(fixedFeed());

    intake.setDefaultCommand(intake.setVoltageCommand(Volts.of(10))); 

    // m_driverController.y().whileTrue(feeder.setFeed
    // erSpeewd(MetersPerSecond.of(1)));
    // m_driverController.a().whileTrue(feeder.setFeederSpeed(MetersPerSecond.of(5)));
  }

  private void testCommands() {
    SmartDashboard.putData("Spindex/OFF", spindexer.setVoltsCommand(Volts.of(0)));
    SmartDashboard.putData("Spindex/LOW-0.22", spindexer.setVoltsCommand(Volts.of(4)));
    SmartDashboard.putData("Spindex/1", spindexer.setVoltsCommand(Volts.of(5)));
    SmartDashboard.putData("Spindex/2", spindexer.setVoltsCommand(Volts.of(6)));
    SmartDashboard.putData("Spindex/MAX-5", spindexer.setVoltsCommand(Volts.of(7)));

    SmartDashboard.putData("Intake/OFF", intake.setVelocityCommand(MetersPerSecond.of(0.1)));
    SmartDashboard.putData("Intake/LOW", intake.setVelocityCommand(MetersPerSecond.of(0.2)));
    SmartDashboard.putData("Intake/MID", intake.setVelocityCommand(MetersPerSecond.of(0.3)));
    SmartDashboard.putData("Intake/EXP", intake.setVelocityCommand(MetersPerSecond.of(1.6)));

    SmartDashboard.putData("Feeder/OFF", feeder.setFeederLinearSpeedCommand(MetersPerSecond.of(0)));
    SmartDashboard.putData("Feeder/LOW", feeder.setFeederLinearSpeedCommand(MetersPerSecond.of(1)));
    SmartDashboard.putData("Feeder/EXP", feeder.setFeederLinearSpeedCommand(MetersPerSecond.of(1.2)));
    SmartDashboard.putData("Feeder/HIGH", feeder.setFeederLinearSpeedCommand(MetersPerSecond.of(1.4)));

    SmartDashboard.putData("Turret/CENTER", turret.setTargetHardwareStateCommand(Rotation2d.fromDegrees(0)));
    SmartDashboard.putData("Turret/45", turret.setTargetHardwareStateCommand(Rotation2d.fromDegrees(45)));
    SmartDashboard.putData("Turret/90", turret.setTargetHardwareStateCommand(Rotation2d.fromDegrees(90)));
    SmartDashboard.putData("Turret/-90", turret.setTargetHardwareStateCommand(Rotation2d.fromDegrees(-90)));

    SmartDashboard.putData("Turret/Low", turret.setVoltsTarget(Volts.of(1)));
    SmartDashboard.putData("Turret/Mid", turret.setVoltsTarget(Volts.of(1.5)));
    SmartDashboard.putData("Turret/High", turret.setVoltsTarget(Volts.of(2)));
    SmartDashboard.putData("Turret/ULTRASPIN", turret.setVoltsTarget(Volts.of(3)));

    SmartDashboard.putData("Hood/MIN-21", hood.setHoodAngleCommand(Degrees.of(21)));
    SmartDashboard.putData("Hood/Slightly up-26", hood.setHoodAngleCommand(Degrees.of(26)));
    SmartDashboard.putData("Hood/MID-33", hood.setHoodAngleCommand(Degrees.of(33)));
    SmartDashboard.putData("Hood/MAX-40", hood.setHoodAngleCommand(Degrees.of(40)));

    SmartDashboard.putData("Hood/VOLTS-LOW", hood.setHoodVoltCommand(Volts.of(0.1)));
    SmartDashboard.putData("Hood/VOLTS-MID", hood.setHoodVoltCommand(Volts.of(0.5)));

    SmartDashboard.putData("Shooter/OFF", shooter.setCommandVelocity(shooter.calcBallAngularV(MetersPerSecond.of(0))));
    SmartDashboard.putData("Shooter/LOW-3",
        shooter.setCommandVelocity(shooter.calcBallAngularV(MetersPerSecond.of(3))));
    SmartDashboard.putData("Shooter/EXP-7",
        shooter.setCommandVelocity(shooter.calcBallAngularV(MetersPerSecond.of(7))));
    SmartDashboard.putData("Shooter/HIGH-10",
        shooter.setCommandVelocity(shooter.calcBallAngularV(MetersPerSecond.of(10))));

    SmartDashboard.putData("Shooter/RadOFF", shooter.setCommandVelocity(RadiansPerSecond.of(0)));
    SmartDashboard.putData("Shooter/RadLow", shooter.setCommandVelocity(RadiansPerSecond.of(250)));
    SmartDashboard.putData("Shooter/RadMid", shooter.setCommandVelocity(RadiansPerSecond.of(350)));
    SmartDashboard.putData("Shooter/RadHigh", shooter.setCommandVelocity(RadiansPerSecond.of(450)));
    SmartDashboard.putData("Shooter/RadHighest", shooter.setCommandVelocity(RadiansPerSecond.of(550)));

    // SmartDashboard.putData("Sys-ID routine", wheelRadiusCharacterization(swerveChassis));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command autoAim() {
    return Commands.runEnd(() -> {
      TargetParameters target = shootingCommand.locateTarget(localization.getFieldPose().getTranslation());
      ShootExport shootExport;

      if (target == null) {
        // TODO log no target, or stop shooting
        feeder.setFeederLinearSpeed(MetersPerSecond.of(0));

        Logger.recordOutput("Calculator/targetPose", new Translation3d());
        Logger.recordOutput("Calculator/turretPose", new Translation3d());
        Logger.recordOutput("Calculator/RelativeVelocity", new Translation3d());
        return;
      }

      Logger.recordOutput("currentYaw",
          turret.getRobotRelativeAngle().plus(localization.getFieldPose().getRotation()).getRadians());
      shootExport = shootingCommand.calculateTrajectory(
          target.targetPosition,
          localization.getTurretFieldPose(),
          localization.getTurretFieldVelocity(),
          target.fixedEntranceAngle);

      Logger.recordOutput("desiredTurretSetpointTurretRel",
          shootExport.turretYaw.minus(localization.getFieldPose().getRotation()));
      turret.setTargetRobotRelativeState(shootExport.turretYaw.minus(localization.getFieldPose().getRotation()),
          localization.getGyroVelocity());
      hood.setHoodAngle(Degrees.of(90).minus(shootExport.hoodPitch));
      shooter.setVelocity(shooter.calcBallAngularV(shootExport.shooterVelocity));

    }, () -> {
      shooter.setVelocity(RadiansPerSecond.of(0));
      hood.setHoodAngle(HoodConstants.minPosition);
      turret.setTargetHardwareState(new Rotation2d(0), RadiansPerSecond.of(0));
    }, hood, turret, shooter);
  }

  public Command autoFeed() {
    return Commands.runEnd(() -> {
      ProjectileTrajectory currentTraj = getCurrentTrajectory();

      TargetParameters target = shootingCommand.locateTarget(localization.getFieldPose().getTranslation());
      if (target == null) {
        // TODO log no target, or stop shooting
        feeder.setFeederLinearSpeed(MetersPerSecond.of(0));

        Logger.recordOutput("Calculator/targetPose", new Translation3d());
        Logger.recordOutput("Calculator/turretPose", new Translation3d());
        Logger.recordOutput("Calculator/RelativeVelocity", new Translation3d());
        return;
      }

      var closest = currentTraj.getClosestPointOfApproach(target.targetPosition);
      boolean isAlignedToShoot = closest.position.getDistance(target.targetPosition) < target.tolerance.in(Meters);
      Logger.recordOutput("Calculator/isAlignedToShoot", isAlignedToShoot);
      Double flightTime = closest.time;
      // im going crazy let me out i hate hobbes and locke
      Logger.recordOutput("Calculator/flightTime", flightTime);
      boolean hubWillBeActiveAndScore = hubTimer.getTimeUntilNextActiveShift().lt(Seconds.of(flightTime));
      Logger.recordOutput("Calculator/hubWillScore", hubWillBeActiveAndScore);

      Logger.recordOutput("time until next shift", hubTimer.getTimeUntilNextActiveShift());

      // one ruler in freedom units, slightly less than a third of a meter stick,
      // approximate width of one of these chairs
      boolean ballWillClearHubRim = currentTraj.getVertex().position.getZ() >= (1.83 + 0.30);
      Logger.recordOutput("Calculator/ballWillClearHubRim", ballWillClearHubRim);
      if (isAlignedToShoot) {
        if ((hubWillBeActiveAndScore && target.isHub && ballWillClearHubRim)) {
          // Good to shoot
          feeder.setFeederLinearSpeed(MetersPerSecond.of(1));
          spindexer.setVolts(Volts.of(5));
        }
        if (target.isHub && !hubWillBeActiveAndScore) {
          feeder.setFeederLinearSpeed(MetersPerSecond.of(0));
          spindexer.setVolts(Volts.of(0));
        }
        if (!target.isHub) {
          feeder.setFeederLinearSpeed(MetersPerSecond.of(1));
          spindexer.setVolts(Volts.of(5));
        }
      } else {
        feeder.setFeederLinearSpeed(MetersPerSecond.of(0));
        spindexer.setVolts(Volts.of(0));
      }
    }, () -> {
      feeder.setFeederLinearSpeed(MetersPerSecond.of(0));
      spindexer.setVolts(Volts.of(0));
    }, feeder, spindexer);
  }

  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    return autoChooser.getSelected();
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>
   * This command should only be used in voltage control mode.
   */

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization(SwerveChassis drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(4);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }, drive),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(9);
                  drive.runChassisSpeeds(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = localization.getGyroAngle();
                  state.gyroDelta = 0.0;
                }, drive),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = localization.getGyroAngle();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * SwerveConstants.driveBaseRadius.in(Meters)) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public Command fixedAim() {
    return Commands.parallel(Commands.run(() -> {
      // Lineup: turret in center line, intake facing the tower, robot is nearly
      // touching the tower,
      shooter.setVelocity(shooter.calcBallAngularV(MetersPerSecond.of(7.3)));
      turret.setTargetHardwareState(Rotation2d.fromDegrees(0), RadiansPerSecond.of(0));
      hood.setHoodAngle(Degrees.of(10));
    }, turret, hood, shooter));
  }

  public Command fixedFeed() {
    return Commands.runEnd(() -> {
      // terrible terrible implementation but I need to have one consistent log of
      // Current Traj
      getCurrentTrajectory();
      feeder.setFeederLinearSpeed(MetersPerSecond.of(1));
      spindexer.setVolts(Volts.of(5));
    }, () -> {
      feeder.setFeederLinearSpeed(MetersPerSecond.of(0));
      spindexer.setVolts(Volts.of(0));
    }, feeder);
  }

  public Command fixedShoot() {
    return Commands.parallel(fixedAim(), fixedFeed());
  }

  public Command autoShoot() {
    return Commands.parallel(autoAim(), autoFeed());
  }

  public ProjectileTrajectory getCurrentTrajectory() {
    ProjectileTrajectory currentTraj = new BallisticSimulator().calculateTrajectory(Constants.FieldConstants.fuel,
        new BallisticSimulator.ShotParameters(
            new Pose3d(localization.getTurretFieldPose(),
                new Rotation3d(0, hood.getAngle().in(Radians) - Degrees.of(90).in(Radians),
                    turret.getRobotRelativeAngle().plus(localization.getFieldPose().getRotation()).getRadians())),
            localization.getTurretFieldVelocity(),
            shooter.getExitVelocity()));
    Logger.recordOutput("Calculator/currentTrajectory", currentTraj.toDebugPoses());
    return currentTraj;
  }
}