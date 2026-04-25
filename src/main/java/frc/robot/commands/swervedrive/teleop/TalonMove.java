// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utility;
//import frc.robot.subsystems.swervedrive.StupidMotor;
import frc.robot.subsystems.swervedrive.TalonMotor;

public class TalonMove extends Command {
  private final TalonMotor m_stupidMotor;

  private boolean m_startDetected;

  private double m_stopTime;

  private boolean m_stop;

  private double m_power;

  public TalonMove(TalonMotor stupidMotor, double power, boolean stop) {
    m_stupidMotor = stupidMotor;
    m_power = power; //num of rotations per second
    m_stopTime = 0;
    m_stop = stop;

    addRequirements(m_stupidMotor);
  }

  @Override
  public void initialize() {
    m_stopTime = 0;
  }

  @Override
  public void execute() {
    if (m_stopTime != 0) return;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_stop) {
      m_stupidMotor.disable();
    }
  }
 
  @Override
  public boolean isFinished() {
    return (m_stopTime != 0) && (Utility.getTime() >= m_stopTime);
  }
}