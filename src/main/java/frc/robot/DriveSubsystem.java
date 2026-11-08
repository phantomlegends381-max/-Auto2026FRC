package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;

public class DriveSubsystem extends SubsystemBase {
  private static final double MAX_SPEED = 3.0;
  private static final double MAX_ANGULAR_SPEED = Math.PI;
  private final boolean simulation = Robot.isSimulation();
  private Pose2d pose = new Pose2d();
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private double lastUpdate = Timer.getFPGATimestamp();
  private final NetworkTable telemetry = NetworkTableInstance.getDefault().getTable("Drive");
  private final DoubleArrayPublisher posePublisher = telemetry.getDoubleArrayTopic("Pose").publish();
  private final DoubleArrayPublisher speedsPublisher = telemetry.getDoubleArrayTopic("RobotRelativeSpeeds").publish();

  public DriveSubsystem() {
    try {
      RobotConfig config;
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception missingSettings) {
        config = new RobotConfig(50.0, 6.0,
            new ModuleConfig(0.0508, MAX_SPEED, 1.2, DCMotor.getNEO(1), 40.0, 1),
            new edu.wpi.first.math.geometry.Translation2d(0.3, 0.3),
            new edu.wpi.first.math.geometry.Translation2d(0.3, -0.3),
            new edu.wpi.first.math.geometry.Translation2d(-0.3, 0.3),
            new edu.wpi.first.math.geometry.Translation2d(-0.3, -0.3));
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
      DriverStation.reportError("PathPlanner setup failed; add robot settings in the PathPlanner GUI: " + exception.getMessage(), exception.getStackTrace());
    }
  }

  public void drive(double forward, double strafe, double rotation) {
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
        -MathUtil.applyDeadband(forward, 0.08) * MAX_SPEED,
        -MathUtil.applyDeadband(strafe, 0.08) * MAX_SPEED,
        -MathUtil.applyDeadband(rotation, 0.08) * MAX_ANGULAR_SPEED);
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, pose.getRotation()));
  }

  public Pose2d getPose() { return pose; }

  public void resetPose(Pose2d newPose) { pose = newPose; }

  public ChassisSpeeds getRobotRelativeSpeeds() { return speeds; }

  public void driveRobotRelative(ChassisSpeeds robotSpeeds) { speeds = robotSpeeds; }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = MathUtil.clamp(now - lastUpdate, 0, 0.05);
    lastUpdate = now;
    if (simulation) {
      ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, pose.getRotation());
      pose = new Pose2d(pose.getX() + field.vxMetersPerSecond * dt,
          pose.getY() + field.vyMetersPerSecond * dt,
          pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
    }
    posePublisher.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
    speedsPublisher.set(new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond});
  }
}
