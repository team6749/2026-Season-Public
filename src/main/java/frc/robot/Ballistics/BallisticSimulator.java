package frc.robot.Ballistics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class BallisticSimulator {

    static double dt = 0.02;
    static int maxSteps = 400;
    static double gravity = 9.81; // m/s^2

    public static class ProjectileParameters {
        public double mass; // kg (0.5 lbs)
        public double dragCoefficient; // cow shaped
        public double radius; // meters
        public double crossSectionalArea;
        public double airDensity; // kg/m^3 at sea level

        public ProjectileParameters(double mass, double dragCoefficient, double radius, double airDensity) {
            this.mass = mass;
            this.dragCoefficient = dragCoefficient;
            this.radius = radius;
            this.crossSectionalArea = Math.PI * radius * radius;
            this.airDensity = airDensity;
        }
    }

    public static class ProjectileState {
        public double time;
        public Translation3d position;
        public Translation3d velocity;

        public ProjectileState(double time, Translation3d position, Translation3d velocity) {
            this.time = time;
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public String toString() {
            return String.format("Time: %.2f s, Position: (%.2f, %.2f, %.2f) m, Velocity: (%.2f, %.2f, %.2f) m/s",
                    time, position.getX(), position.getY(), position.getZ(),
                    velocity.getX(), velocity.getY(), velocity.getZ());
        }
    }

    public static class ShotParameters {
        Pose3d launchPose;
        Translation3d launchRobotVelocity;
        LinearVelocity exitSpeed;

        public ShotParameters(Pose3d launchPose, Translation3d launchRobotVelocity, LinearVelocity exitSpeed) {
            this.launchPose = launchPose;
            this.launchRobotVelocity = launchRobotVelocity;
            this.exitSpeed = exitSpeed;
        }

        @Override
        public String toString() {
            return String.format("Launch Pose: %s, Launch Robot Velocity: (%.2f, %.2f, %.2f) m/s, Exit Speed: %.2f m/s",
                    launchPose.toString(),
                    launchRobotVelocity.getX(), launchRobotVelocity.getY(), launchRobotVelocity.getZ(),
                    exitSpeed.in(MetersPerSecond));
        }
    }

    public static class ProjectileTrajectory {
        public ShotParameters shotParameters;
        public ProjectileState[] states;

        public ProjectileTrajectory(ShotParameters shotParameters, ProjectileState[] states) {
            this.shotParameters = shotParameters;
            this.states = states;
        }

        public Pose3d[] toDebugPoses() {
            // return the first pose, and then every 10th pose as a list of Pose3d
            List<Pose3d> poses = new ArrayList<>();
            final int step = 6;
            for (int i = 0; i < states.length; i += step) {
                Translation3d pos = states[i].position;
                poses.add(new Pose3d(pos, new edu.wpi.first.math.geometry.Rotation3d()));
            }
            // Always add last pose if it is missing
            if(states.length > 0 && (states.length - 1) % step != 0) {
                Translation3d pos = states[states.length - 1].position;
                poses.add(new Pose3d(pos, new edu.wpi.first.math.geometry.Rotation3d()));
            }
            return poses.toArray(new Pose3d[0]);
        }

         /// Returns the closest point of approach to the target along the trajectory
         /// This is a O(n) search through the trajectory states
        public ProjectileState getClosestPointOfApproach(Translation3d target) {
            ProjectileState closestState = states[0];
            for (ProjectileState state : states) {
                double dist = state.position.getDistance(target);
                if (dist < closestState.position.getDistance(target)) {
                    closestState = state;
                }
            }
            return closestState;
        }

        public ProjectileState getVertex () {
            ProjectileState closestState = states[0];
            for (ProjectileState state : states) {
                double height = state.position.getZ();
                if (height > closestState.position.getZ()) {
                    closestState = state;
                }
            }
            return closestState;
        }
    }

    // Launch pose is the global position and rotation of the launcher (the point
    // where the ball goes ballistic)
    // Launch velocity is the global velocity of the launcher (example, the robot
    // turret speed relative to the field)
    // Exit velocity is the velocity of the ball relative to the launcher (the speed
    // the launcher adds to the ball)
    public ProjectileTrajectory calculateTrajectory(ProjectileParameters projectile, ShotParameters shotParams) {
        List<ProjectileState> trajectory = new ArrayList<>();

        double time = 0.0;
        Translation3d position = shotParams.launchPose.getTranslation();
        // Calculate initial velocity vector in global frame
        // Assume shooter launches forward in its own frame (positive X)
        Translation3d muzzleVelocity = new Translation3d(shotParams.exitSpeed.in(MetersPerSecond), 0, 0)
                .rotateBy(shotParams.launchPose.getRotation());
        Translation3d velocity = muzzleVelocity.plus(shotParams.launchRobotVelocity);

        trajectory.add(new ProjectileState(time, position, velocity));

        for (int i = 0; i < maxSteps; i++) {
            // Stop if we hit the ground
            if (position.getZ() < 0) {
                if(i <= 2) {
                    System.err.println("WARNING: Projectile hit the ground immediately after launch: " + trajectory.get(0).toString());
                }
                break;
            }

            // Calculate forces
            // Air Resistance: F_d = 0.5 * rho * Cd * A * v^2
            double speed = velocity.getNorm();
            Translation3d dragForce = new Translation3d();

            if (speed > 0) {
                double dragMagnitude = 0.5 * projectile.airDensity * projectile.dragCoefficient * projectile.crossSectionalArea * speed * speed;
                Translation3d dragDirection = velocity.unaryMinus().div(speed);
                dragForce = dragDirection.times(dragMagnitude);
            }

            Translation3d gravityForce = new Translation3d(0, 0, -projectile.mass * gravity);
            Translation3d totalForce = dragForce.plus(gravityForce);
            Translation3d acceleration = totalForce.div(projectile.mass);

            // Euler Integration
            Translation3d newVelocity = velocity.plus(acceleration.times(dt));
            Translation3d newPosition = position.plus(velocity.times(dt)); // Using initial velocity for position step
                                                                           // (Forward Euler)

            // Update state
            time += dt;
            velocity = newVelocity;
            position = newPosition;

            trajectory.add(new ProjectileState(time, position, velocity));
        }

        return new ProjectileTrajectory(shotParams, trajectory.toArray(new ProjectileState[0]));
    }
}
