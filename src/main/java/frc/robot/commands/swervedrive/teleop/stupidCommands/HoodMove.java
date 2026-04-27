// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.teleop.stupidCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodMove extends Command {
  private final Hood m_hood; 
  private final double m_setPoint;

  public static boolean hoodStop = false;

  private boolean m_autoAim;

  /** Creates a new ArmMove. */
  public HoodMove(Hood hood, double setPoint, boolean autoAim) {
   m_hood = hood;
   m_setPoint = setPoint;
   m_autoAim = autoAim;
    addRequirements(hood);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hood.enable();
    m_hood.setGoal(m_setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!m_hasMoved) {
    //   // if arm is being set to vertical down, then don't set it until the elevator is all the way down
    //   if (m_setPoint < Constants.Setpoints.armElevatorMoveAngle &&
    //       m_elevator.getPos() > Constants.Setpoints.elevatorZero + Constants.MotorConfig.elevatorTolerance)
    //     return;

    //   System.out.println("setting arm angle");
      
    //   m_arm.enable();
    //   m_arm.setGoal(m_setPoint);
    // }
    if (m_autoAim) {
      m_hood.setGoal(RobotContainer.drivebase.getHoodAngle());
    }

    if (hoodStop) {
      m_hood.disable();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_hood.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hood.atGoal();
  }
}