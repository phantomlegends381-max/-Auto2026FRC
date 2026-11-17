package frc.robot.subsystems;

import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

/** Desktop-only geometric Limelight/AprilTag simulator. */
public final class LimelightSim {
    // Replace with the measured robot-to-camera transform (meters, radians).
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.25, 0.0, 0.55), new Rotation3d(0, Math.toRadians(-15), 0));
    private static final double MAX_RANGE_METERS = 4.5;
    private static final double HALF_H_FOV_DEGREES = 31.0;
    private static final double HALF_V_FOV_DEGREES = 24.0;
    private static final double LATENCY_SECONDS = 0.035;
    private static final double[] EMPTY_POSE = new double[11];

    private final edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final edu.wpi.first.networktables.NetworkTable visionTelemetry = NetworkTableInstance.getDefault().getTable("Vision");
    private final StructPublisher<Pose2d> groundTruthPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Sim/GroundTruthPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> visionPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Vision/EstimatedPose", Pose2d.struct).publish();
    private final AprilTagFieldLayout field;

    public LimelightSim() {
        if (!RobotBase.isSimulation()) throw new IllegalStateException("LimelightSim is desktop simulation only");
        field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    public void update(Pose2d groundTruth) {
        Pose3d robot = new Pose3d(groundTruth);
        Pose3d camera = robot.transformBy(ROBOT_TO_CAMERA);
        AprilTag best = null;
        double bestDistance = Double.POSITIVE_INFINITY;
        double bestTx = 0, bestTy = 0;
        for (AprilTag tag : field.getTags()) {
            Pose3d target = tag.pose;
            Translation3d relative = target.getTranslation().minus(camera.getTranslation());
            double distance = relative.getNorm();
            if (distance > MAX_RANGE_METERS || distance < 0.05) continue;
            Translation3d local = relative.rotateBy(camera.getRotation().unaryMinus());
            if (local.getX() <= 0) continue;
            double tx = Math.toDegrees(Math.atan2(-local.getY(), local.getX()));
            double ty = Math.toDegrees(Math.atan2(local.getZ(), Math.hypot(local.getX(), local.getY())));
            if (Math.abs(tx) > HALF_H_FOV_DEGREES || Math.abs(ty) > HALF_V_FOV_DEGREES) continue;
            if (distance < bestDistance) {
                best = tag; bestDistance = distance; bestTx = tx; bestTy = ty;
            }
        }

        double now = Timer.getFPGATimestamp();
        groundTruthPublisher.set(groundTruth);
        table.getEntry("tl").setDouble(LATENCY_SECONDS * 1000.0);
        if (best == null) {
            table.getEntry("tv").setDouble(0);
            table.getEntry("tx").setDouble(0);
            table.getEntry("ty").setDouble(0);
            table.getEntry("ta").setDouble(0);
            table.getEntry("tid").setDouble(-1);
            table.getEntry("botpose_wpiblue").setDoubleArray(EMPTY_POSE);
            table.getEntry("botpose_orb_wpiblue").setDoubleArray(EMPTY_POSE);
            visionPosePublisher.set(new Pose2d());
            return;
        }

        table.getEntry("tv").setDouble(1);
        table.getEntry("tx").setDouble(bestTx);
        table.getEntry("ty").setDouble(bestTy);
        table.getEntry("ta").setDouble(100.0 / (bestDistance * bestDistance));
        table.getEntry("tid").setDouble(best.ID);
        double[] pose = poseArray(groundTruth, now, bestDistance);
        table.getEntry("botpose_wpiblue").setDoubleArray(pose);
        table.getEntry("botpose_orb_wpiblue").setDoubleArray(pose);
        visionPosePublisher.set(new Pose2d(pose[0], pose[1], edu.wpi.first.math.geometry.Rotation2d.fromDegrees(pose[5])));
        visionTelemetry.getEntry("BestTagId").setDouble(best.ID);
    }

    private static double[] poseArray(Pose2d pose, double now, double distance) {
        ThreadLocalRandom random = ThreadLocalRandom.current();
        return new double[] {
            pose.getX() + random.nextGaussian() * 0.015,
            pose.getY() + random.nextGaussian() * 0.015,
            0, 0, 0, pose.getRotation().getDegrees() + random.nextGaussian() * 0.5,
            LATENCY_SECONDS * 1000.0, 1, 0, distance, 1
        };
    }

}
