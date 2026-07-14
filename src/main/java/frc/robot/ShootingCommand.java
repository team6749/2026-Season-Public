package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ballistics.BallisticSimulator;
import frc.robot.Ballistics.BallisticSimulator.ProjectileTrajectory;
import frc.robot.utils.FieldMap;

public class ShootingCommand {
        Translation2d xyPose = new Translation2d();
        FieldMap map;
        private LinearVelocity verticalWorldVelocity = MetersPerSecond.of(0);
        private LinearVelocity horizontalBallVelocity = MetersPerSecond.of(0);
        private LinearVelocity horizontalWorldVelocity = MetersPerSecond.of(0);
        private LinearVelocity verticalBallVelocity = MetersPerSecond.of(0);
        private Distance xyDisplacement = Meters.of(0);
        private Rotation2d safeTurretYaw = new Rotation2d();

        private LinearVelocity totalShooterVelocity = MetersPerSecond.of(0);
        private Angle hoodAngle = Radians.of(0);

        public ShootingCommand(FieldMap map) {
                this.map = map;
                SmartDashboard.putNumber("ShootSpeedCompensation", 1);
        }
        public class ShootExport {
                LinearVelocity shooterVelocity = MetersPerSecond.of(0);
                Angle hoodPitch = Radians.of(0);
                Rotation2d turretYaw = new Rotation2d();

                public ShootExport(LinearVelocity fieldTargetVelocity, Angle fieldTargetPitch, Rotation2d fieldYaw) {
                        this.shooterVelocity = fieldTargetVelocity;
                        this.hoodPitch = fieldTargetPitch;
                        this.turretYaw = fieldYaw;
                }
        }

        public class TargetParameters {
                Angle fixedEntranceAngle = Radians.of(0);
                Translation3d targetPosition = new Translation3d();
                Distance tolerance = Meters.of(0);
                boolean isHub;

                public TargetParameters(Angle entranceAngle, Translation3d targetPosition, Distance tolerance,
                                boolean isHub) {
                        this.fixedEntranceAngle = entranceAngle;
                        this.targetPosition = targetPosition;
                        this.tolerance = tolerance;
                        this.isHub = isHub;
                }
        }

        public TargetParameters locateTarget(Translation2d turretPose) {
                xyPose = turretPose;
                Logger.recordOutput("xyPose", xyPose);
                if (map.getRegion("hub_visibility").contains(xyPose)) {
                        Translation3d thehub = map.getPoint("hub").getFlippedLocation();
                        var x = new Translation2d(thehub.getX(), thehub.getY()).getDistance(turretPose);

                        var y = new InterpolatingDoubleTreeMap();
                        y.put(2d, 70d);
                        y.put(6d, 60d);

                        return new TargetParameters(Degrees.of(y.get(x)), map.getPoint("hub").getFlippedLocation(),
                                        Inches.of(24),
                                        true);
                }
                if (map.getRegion("top_shuttle_zone").contains(xyPose)
                                || map.getRegion("top_shuttle_far").contains(xyPose)) {
                        return new TargetParameters(Degrees.of(65),
                                        map.getPoint("top_shuttle_target").getFlippedLocation(),
                                        Inches.of(50), false);
                }
                if (map.getRegion("bottom_shuttle_visibility").contains(xyPose)
                                || map.getRegion("bottom_shuttle_far").contains(xyPose)) {
                        return new TargetParameters(Degrees.of(65),
                                        map.getPoint("bottom_shuttle_target").getFlippedLocation(),
                                        Inches.of(50), false);
                }
                return null;
        }

        public ShootExport calculateTrajectory(Translation3d fieldRelativeTarget, Translation3d curTurretPos,
                        Translation3d fieldRelativeVelocity, Angle desiredEntranceAngle) {

                Logger.recordOutput("Calculator/targetPose", fieldRelativeTarget);
                Logger.recordOutput("Calculator/turretPose", curTurretPos);
                Logger.recordOutput("Calculator/RelativeVelocity", fieldRelativeVelocity);

                xyDisplacement = Meters.of(Math.hypot(fieldRelativeTarget.getX() - curTurretPos.getX(),
                                fieldRelativeTarget.getY() - curTurretPos.getY()));

                // Calculate ballistic trajectory for a fixed launch angle
                // We interpret desiredEntranceAngle as the desired Launch Angle for an upward
                // shot
                double zDiff = fieldRelativeTarget.getZ() - curTurretPos.getZ();
                double launchSlope = Math.tan(desiredEntranceAngle.in(Radians));

                // Derivation: z = x * tan(theta) - (g * x^2) / (2 * vx^2)
                // vx = x * sqrt(g / (2 * (x * tan(theta) - z)))
                double denominator = 2 * (xyDisplacement.in(Meters) * launchSlope - zDiff);

                if (denominator <= 0) {
                        // Target is structurally unreachable with this launch angle (too high for the
                        // angle)
                        // Fall request to a safe default or clamp
                        denominator = 0.1;
                }

                double vx = xyDisplacement.in(Meters) * Math.sqrt(Constants.gravity / denominator);
                horizontalWorldVelocity = MetersPerSecond.of(vx);

                double vy = vx * launchSlope;
                verticalWorldVelocity = MetersPerSecond.of(vy);

                // Calculate target angle and velocity components
                double targetYaw = Math.atan2(fieldRelativeTarget.getY() - curTurretPos.getY(),
                                fieldRelativeTarget.getX() - curTurretPos.getX());
                double worldVx = horizontalWorldVelocity.in(MetersPerSecond) * Math.cos(targetYaw);
                double worldVy = horizontalWorldVelocity.in(MetersPerSecond) * Math.sin(targetYaw);

                // Adjust for robot velocity to get shooter velocity vector
                double shooterVx = worldVx - fieldRelativeVelocity.getX();
                double shooterVy = worldVy - fieldRelativeVelocity.getY();

                horizontalBallVelocity = MetersPerSecond.of(Math.hypot(shooterVx, shooterVy));
                verticalBallVelocity = verticalWorldVelocity.minus(MetersPerSecond.of(fieldRelativeVelocity.getZ()));

                hoodAngle = Radians
                                .of(Math.atan2(verticalBallVelocity.in(MetersPerSecond),
                                                horizontalBallVelocity.in(MetersPerSecond)));
                safeTurretYaw = new Rotation2d(Math.atan2(shooterVy, shooterVx));

                totalShooterVelocity = MetersPerSecond
                                .of(Math.sqrt(Math.pow(horizontalBallVelocity.in(MetersPerSecond), 2)
                                                + Math.pow(verticalBallVelocity.in(MetersPerSecond), 2)));

                // TODO replace this with a Euler solver
                totalShooterVelocity = totalShooterVelocity
                                .plus(MetersPerSecond.of(Math.pow(xyDisplacement.in(Meters), 1.26) * 0.06749));

                totalShooterVelocity = totalShooterVelocity.times(SmartDashboard.getNumber("ShootSpeedCompensation", 1));

                Logger.recordOutput("Calculator/calcShooterVelo", totalShooterVelocity);
                Logger.recordOutput("Calculator/calcHoodAngle", hoodAngle);
                Logger.recordOutput("Calculator/calcTurretYaw", safeTurretYaw);

                ProjectileTrajectory trajectory = new BallisticSimulator().calculateTrajectory(
                                Constants.FieldConstants.fuel,
                                new BallisticSimulator.ShotParameters(
                                                new Pose3d(curTurretPos,
                                                                new Rotation3d(0, -hoodAngle.in(Radians),
                                                                                safeTurretYaw.getRadians())),
                                                fieldRelativeVelocity,
                                                totalShooterVelocity));
                Logger.recordOutput("Trajectory", trajectory.toDebugPoses());

                Logger.recordOutput("Closest Point of approach",
                                trajectory.getClosestPointOfApproach(fieldRelativeTarget).position);

                ProjectileTrajectory shooterTraj = new BallisticSimulator().calculateTrajectory(
                                Constants.FieldConstants.fuel,
                                new BallisticSimulator.ShotParameters(
                                                new Pose3d(curTurretPos,
                                                                new Rotation3d(0, -hoodAngle.in(Radians),
                                                                                safeTurretYaw.getRadians())),
                                                new Translation3d(),
                                                totalShooterVelocity));
                Logger.recordOutput("shooterTraj", shooterTraj.toDebugPoses());

                Logger.recordOutput("close",
                                shooterTraj.getClosestPointOfApproach(fieldRelativeTarget).position);

                return new ShootExport(totalShooterVelocity, hoodAngle, safeTurretYaw);
        }

}
