// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utility;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TeleopFaceAprilTag extends Command {
  public static boolean disable = false;

  //Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SwerveSubsystem.overrideRotation = Utility.aprilTagInView("limelight");
    // if (!SwerveSubsystem.overrideRotation) { 
    //   SwerveSubsystem.newRotation = 0;
    // return; } // Check for AprilTag in view
    SwerveSubsystem.newRotation = 0;
    RobotContainer.drivebase.aimAtTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SwerveSubsystem.overrideRotation = false;
    disable = false;
    SwerveSubsystem.newRotation = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return disable;
  }
}