package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private final DriveSubsystem drive = new DriveSubsystem();
  private final XboxController controller = new XboxController(0);
  private final SendableChooser<Command> autoChooser;
  private final Field2d field = new Field2d();

  public RobotContainer() {
    drive.setDefaultCommand(new RunCommand(
        () -> drive.drive(controller.getLeftY(), controller.getLeftX(), controller.getRightX()), drive));
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Drive Forward", driveFor(0.7, 0.0));
    autoChooser.addOption("Drive Backward", driveFor(-0.7, 0.0));
    autoChooser.addOption("Drive Left", driveFor(0.0, 0.7));
    autoChooser.addOption("Drive Right", driveFor(0.0, -0.7));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Field", field);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private Command driveFor(double forward, double strafe) {
    return Commands.run(() -> drive.driveRobotRelative(
        new edu.wpi.first.math.kinematics.ChassisSpeeds(forward, strafe, 0.0)), drive)
        .withTimeout(2.0)
        .finallyDo(interrupted -> drive.driveRobotRelative(
            new edu.wpi.first.math.kinematics.ChassisSpeeds()));
  }

  public void updateField() {
    field.setRobotPose(drive.getPose());
  }
}
