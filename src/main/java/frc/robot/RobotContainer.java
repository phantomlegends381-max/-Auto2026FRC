package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AutonomousFSMCommand;
import frc.robot.commands.AprilTagApproachCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = drivetrain.isPathPlannerConfigured()
            ? AutoBuilder.buildAutoChooser()
            : new SendableChooser<>();
        if (!drivetrain.isPathPlannerConfigured()) {
            SmartDashboard.putString("Auto Mode Status", "PathPlanner settings.json missing; autonomous unavailable");
        }
        autoChooser.addOption("Approach Visible AprilTag", new AprilTagApproachCommand(drivetrain));
        SmartDashboard.putNumber("AprilTag Target ID", 0);
        autoChooser.addOption("Approach Specific AprilTag", new AprilTagApproachCommand(
            drivetrain,
            () -> (int) Math.round(SmartDashboard.getNumber("AprilTag Target ID", 0))
        ));
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                if (DriverStation.isAutonomousEnabled()) return brake;
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
            })
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.rightBumper().whileTrue(new AprilTagApproachCommand(drivetrain));
        joystick.leftTrigger().whileTrue(new AprilTagApproachCommand(
            drivetrain,
            () -> (int) Math.round(SmartDashboard.getNumber("AprilTag Target ID", 0))
        ));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return new AutonomousFSMCommand(
            autoChooser::getSelected,
            () -> drivetrain.setControl(brake),
            drivetrain
        ).withName("Autonomous FSM");
    }
}
