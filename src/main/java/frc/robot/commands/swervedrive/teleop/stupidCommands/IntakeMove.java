// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.teleop.stupidCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utility;
//import frc.robot.subsystems.CoralDetector;
import frc.robot.subsystems.swervedrive.BasicSparkMotor;

public class IntakeMove extends Command {
  private final BasicSparkMotor m_intakeMotor;
  //private final CoralDetector m_coralDetector;

  private double m_stopTime;

  private boolean m_reverse;

  public IntakeMove(BasicSparkMotor intakeMotor, boolean reverse) {
    m_intakeMotor = intakeMotor;
    m_reverse = reverse;
    m_stopTime = 0;

    addRequirements(intakeMotor);
  }

  @Override
  public void initialize() {
    m_stopTime = 0;

    if (m_reverse) {
      m_intakeMotor.onBackwards();
    } else {
      m_intakeMotor.onForward(); // drives intake motor
    }
  }

  @Override
  public void execute() {
    if (m_stopTime != 0) return;
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeMotor.disable();
  }

  @Override
  public boolean isFinished() {
    return (m_stopTime != 0) && (Utility.getTime() >= m_stopTime);
  }
}