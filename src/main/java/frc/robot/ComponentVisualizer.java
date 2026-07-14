package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class ComponentVisualizer {

    Pose3d turretPose = new Pose3d();
    Pose3d hoodPose = new Pose3d();
    Pose3d intakePose = new Pose3d();

    ComponentVisualizer() {
        Logger.recordOutput("ComponentVisualizer/ComponentPosesZeroed",
                new Pose3d[] { new Pose3d(), new Pose3d(), new Pose3d() });
        publish();
    }

    public void setTurret(Angle pose) {
        // Zero degrees in turret space is facing backwards
        turretPose = new Pose3d(-0.13, 0.14, 0.36, new Rotation3d(0, 0, pose.plus(Degrees.of(90)).in(Radians)));
        publish();
    }

    public void setHood(Angle pose) {
        hoodPose = turretPose
                .transformBy(
                        new Transform3d(new Translation3d(0, 0.1, 0.075), new Rotation3d(-pose.in(Radians), 0, 0)));
        publish();
    }

    public void setIntake(double percentageExtended) {
        Pose3d extendedPose = new Pose3d(0.7, 0, 0.1, new Rotation3d(0, 0, 0));
        Pose3d retractedPose = new Pose3d(0.4, 0, 0.3, new Rotation3d(0, 0, 0));
        intakePose = new Pose3d(
                retractedPose.getTranslation().interpolate(extendedPose.getTranslation(), percentageExtended),
                retractedPose.getRotation().interpolate(extendedPose.getRotation(), percentageExtended));
        publish();
    }

    private void publish() {
        Logger.recordOutput("ComponentVisualizer/ComponentPoses", new Pose3d[] { intakePose, turretPose, hoodPose });
    }

}
