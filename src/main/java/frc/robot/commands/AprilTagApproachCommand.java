package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Drives slowly toward the currently visible AprilTag and brakes on loss or timeout. */
public final class AprilTagApproachCommand extends Command {
    private static final String LIMELIGHT_NAME = "limelight";
    private static final double STOP_DISTANCE_METERS = 0.9;
    private static final double MAX_APPROACH_SPEED_MPS = 0.75;
    private static final double MAX_ROTATION_RAD_PER_SEC = 0.7;
    private static final double MAX_TX_FOR_FORWARD_DEGREES = 25.0;
    private static final double MAX_RUNTIME_SECONDS = 8.0;
    private static final double TARGET_LOST_GRACE_SECONDS = 0.25;

    private final CommandSwerveDrivetrain drivetrain;
    private final IntSupplier targetTagIdSupplier;
    private final boolean requireSpecificTag;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final Timer runTimer = new Timer();
    private final Timer lostTargetTimer = new Timer();

    private boolean finished;
    private int targetTagId;

    public AprilTagApproachCommand(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, () -> -1, false);
    }

    public AprilTagApproachCommand(CommandSwerveDrivetrain drivetrain, IntSupplier targetTagIdSupplier) {
        this(drivetrain, targetTagIdSupplier, true);
    }

    private AprilTagApproachCommand(
        CommandSwerveDrivetrain drivetrain,
        IntSupplier targetTagIdSupplier,
        boolean requireSpecificTag
    ) {
        this.drivetrain = drivetrain;
        this.targetTagIdSupplier = targetTagIdSupplier;
        this.requireSpecificTag = requireSpecificTag;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        finished = false;
        runTimer.restart();
        lostTargetTimer.stop();
        lostTargetTimer.reset();
        drivetrain.setControl(brakeRequest);
        targetTagId = targetTagIdSupplier.getAsInt();
        if (requireSpecificTag && targetTagId <= 0) {
            finishWithBrake("Enter a positive target tag ID");
            return;
        }
        if (targetTagId > 0) {
            LimelightHelpers.setPriorityTagID(LIMELIGHT_NAME, targetTagId);
        }
        SmartDashboard.putString("AprilTag Approach", targetTagId > 0
            ? "Searching for tag ID " + targetTagId
            : "Searching for visible tag");
    }

    @Override
    public void execute() {
        if (runTimer.hasElapsed(MAX_RUNTIME_SECONDS)) {
            finishWithBrake("Timed out");
            return;
        }

        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        double txDegrees = Double.NaN;
        double distanceMeters = Double.NaN;
        boolean requestedTagVisible = targetTagId <= 0;
        if (estimate != null && targetTagId > 0 && estimate.rawFiducials != null) {
            for (LimelightHelpers.RawFiducial fiducial : estimate.rawFiducials) {
                if (fiducial.id == targetTagId) {
                    txDegrees = fiducial.txnc;
                    distanceMeters = fiducial.distToRobot;
                    requestedTagVisible = true;
                    break;
                }
            }
        } else if (targetTagId <= 0) {
            txDegrees = LimelightHelpers.getTX(LIMELIGHT_NAME);
            if (estimate != null) distanceMeters = estimate.avgTagDist;
        }

        boolean validTarget = LimelightHelpers.getTV(LIMELIGHT_NAME)
            && estimate != null
            && estimate.isMegaTag2
            && estimate.tagCount > 0
            && requestedTagVisible
            && Double.isFinite(distanceMeters)
            && distanceMeters > 0.0
            && distanceMeters <= 8.0
            && Double.isFinite(txDegrees);

        if (!validTarget) {
            drivetrain.setControl(brakeRequest);
            if (!lostTargetTimer.isRunning()) lostTargetTimer.restart();
            SmartDashboard.putString("AprilTag Approach", "No valid tag; braking");
            if (lostTargetTimer.hasElapsed(TARGET_LOST_GRACE_SECONDS)) {
                finishWithBrake("Tag lost");
            }
            return;
        }

        lostTargetTimer.stop();
        lostTargetTimer.reset();

        SmartDashboard.putNumber("AprilTag Approach Distance m", distanceMeters);
        SmartDashboard.putNumber("AprilTag Approach TX deg", txDegrees);

        if (distanceMeters <= STOP_DISTANCE_METERS) {
            finishWithBrake("Stop distance reached");
            return;
        }

        // Turn in place until the tag is reasonably centered, then approach slowly.
        double forwardMps = 0.0;
        if (Math.abs(txDegrees) <= MAX_TX_FOR_FORWARD_DEGREES) {
            forwardMps = Math.min(
                MAX_APPROACH_SPEED_MPS,
                Math.max(0.15, (distanceMeters - STOP_DISTANCE_METERS) * 0.45)
            );
        }
        double rotationRadPerSec = clamp(-txDegrees * 0.035,
            -MAX_ROTATION_RAD_PER_SEC, MAX_ROTATION_RAD_PER_SEC);

        drivetrain.setControl(driveRequest
            .withVelocityX(forwardMps)
            .withVelocityY(0.0)
            .withRotationalRate(rotationRadPerSec));
        SmartDashboard.putString("AprilTag Approach", "Approaching visible tag");
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        runTimer.stop();
        lostTargetTimer.stop();
        drivetrain.setControl(brakeRequest);
        if (targetTagId > 0) LimelightHelpers.setPriorityTagID(LIMELIGHT_NAME, 0);
        if (interrupted) SmartDashboard.putString("AprilTag Approach", "Stopped by driver");
    }

    private void finishWithBrake(String reason) {
        drivetrain.setControl(brakeRequest);
        SmartDashboard.putString("AprilTag Approach", reason);
        finished = true;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
