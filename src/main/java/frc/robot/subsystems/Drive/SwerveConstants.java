package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveConstants {
        public static final double deadZone = 0.06;

        public static final SlewRateLimiter driveLimiterX = new SlewRateLimiter(8);
        public static final SlewRateLimiter driveLimiterY = new SlewRateLimiter(8);
        public static final SlewRateLimiter driveLimiterTheta = new SlewRateLimiter(30);

        // Max linear velocity of the module (how fast it can spin)
        public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(2.5);
        public static final LinearVelocity superLinearVelocity = MetersPerSecond.of(5);

        // This should be based on physical properties of the the max wheel speeds in a
        // circle
        public static final AngularVelocity maxAngularVelocity = DegreesPerSecond.of(135);
        public static final double angleReduction = (12.1 / 1.0);
        public static final double driveReduction = 6.20 / 1.0;

        public static final Distance wheelDiameter = Millimeters.of(100);
        public static final Distance wheelRadius = wheelDiameter.div(2);
        public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);

        // Distance between the center of wheels along the width of the robot(Y axis)
        public static final Distance trackWidth = Meters.of(0.65);
        // Distance between the center of wheels along the depth of the robot (X axis)
        public static final Distance trackHeight = Meters.of(0.65);
        // Radius from the center to module. Square circumscribed in circle not circle
        // circumscbribed in ^2
        public static final Distance driveBaseRadius = Meters.of(0.459);

        public static class ModuleDefinition {
                public final Translation2d location;
                public final String label;

                public ModuleDefinition(String label,
                                Translation2d location) {
                        this.label = label;
                        this.location = location;
                }
        }

        public static final ModuleDefinition FLModule = new ModuleDefinition("Front Left",
                        new Translation2d(trackHeight.div(2), trackWidth.div(2)));
        public static final ModuleDefinition FRModule = new ModuleDefinition("Front Right",
                        new Translation2d(trackHeight.div(2), trackWidth.div(2).unaryMinus()));
        public static final ModuleDefinition BLModule = new ModuleDefinition("Back Left",
                        new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2)));
        public static final ModuleDefinition BRModule = new ModuleDefinition("Back Right",
                        new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2).unaryMinus()));
}
