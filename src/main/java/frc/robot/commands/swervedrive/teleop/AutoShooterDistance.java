package frc.robot.commands.swervedrive.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utility;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class AutoShooterDistance extends Command{
     public static boolean disable = false;

  //Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drivebase.getShooterRPS();
    RobotContainer.drivebase.getHoodAngle();
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    disable = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return disable;
  }
}
