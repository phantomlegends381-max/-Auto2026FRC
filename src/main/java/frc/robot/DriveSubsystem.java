package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private static final double MAX_SPEED = 3.0;
  private static final double MAX_ANGULAR_SPEED = Math.PI;
  private static final String LIMELIGHT_NAME = "limelight"; // Match your Limelight's hostname

  private final boolean simulation = Robot.isSimulation();
  private Pose2d simPose = new Pose2d();
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private double lastUpdate = Timer.getFPGATimestamp();

  // Swerve Kinematics & Pose Estimator (Update translations to match your robot's wheelbase)
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(0.3, 0.3),
      new Translation2d(0.3, -0.3),
      new Translation2d(-0.3, 0.3),
      new Translation2d(-0.3, -0.3));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final NetworkTable telemetry = NetworkTableInstance.getDefault().getTable("Drive");
  private final DoubleArrayPublisher posePublisher = telemetry.getDoubleArrayTopic("Pose").publish();
  private final DoubleArrayPublisher speedsPublisher = telemetry.getDoubleArrayTopic("RobotRelativeSpeeds").publish();

  public DriveSubsystem() {
    // Initialize PoseEstimator with starting positions (Replace with real gyro & module positions)
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        getGyroRotation(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1),   // State std devs (x, y, theta) - trust in odometry
        VecBuilder.fill(0.7, 0.7, 9999999) // Vision std devs - trust in Limelight (ignore vision yaw)
    );

    try {
      RobotConfig config;
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception missingSettings) {
        config = new RobotConfig(50.0, 6.0,
            new ModuleConfig(0.0508, MAX_SPEED, 1.2, DCMotor.getNEO(1), 40.0, 1),
            new Translation2d(0.3, 0.3),
            new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3),
            new Translation2d(-0.3, -0.3));
      }

      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPHolonomicDriveController(new PIDConstants(5.0, 0, 0), new PIDConstants(5.0, 0, 0)),
          config,
          () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
          this);
    } catch (Exception exception) {
      DriverStation.reportError("PathPlanner setup failed: " + exception.getMessage(), exception.getStackTrace());
    }
  }

  public void drive(double forward, double strafe, double rotation) {
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
        -MathUtil.applyDeadband(forward, 0.08) * MAX_SPEED,
        -MathUtil.applyDeadband(strafe, 0.08) * MAX_SPEED,
        -MathUtil.applyDeadband(rotation, 0.08) * MAX_ANGULAR_SPEED);
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getPose().getRotation()));
  }

  public Pose2d getPose() {
    return simulation ? simPose : poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d newPose) {
    simPose = newPose;
    poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return speeds;
  }

  public void driveRobotRelative(ChassisSpeeds robotSpeeds) {
    speeds = robotSpeeds;
    // TODO: Convert robotSpeeds to SwerveModuleStates and command your physical motors here
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = MathUtil.clamp(now - lastUpdate, 0, 0.05);
    lastUpdate = now;

    if (simulation) {
      ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, simPose.getRotation());
      simPose = new Pose2d(
          simPose.getX() + field.vxMetersPerSecond * dt,
          simPose.getY() + field.vyMetersPerSecond * dt,
          simPose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
    } else {
      // 1. Update odometry from wheel encoders and gyro
      poseEstimator.update(getGyroRotation(), getModulePositions());

      // 2. Sync AprilTag vision using LimelightHelpers (MegaTag2)
      updateVisionPose();
    }

    Pose2d currentPose = getPose();
    posePublisher.set(new double[] {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians()});
    speedsPublisher.set(new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond});
  }

  private void updateVisionPose() {
    // MegaTag2 requires the robot's current yaw in degrees (Blue alliance origin: 0 deg = facing Red wall)
    LimelightHelpers.SetRobotOrientation(
        LIMELIGHT_NAME,
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

    // Reject bad measurements (no tags seen, or robot spinning faster than 720 deg/sec)
    boolean doRejectUpdate = false;
    if (mt2 == null || mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (Math.abs(Math.toDegrees(speeds.omegaRadiansPerSecond)) > 720) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
  }

  // --- HARDWARE PLACEHOLDERS: Replace these with your actual Gyro and Swerve Module methods ---
  private Rotation2d getGyroRotation() {
    // Example: return gyro.getRotation2d();
    return new Rotation2d();
  }

  private SwerveModulePosition[] getModulePositions() {
    // Example: return new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition() };
    return new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };
  }
}
