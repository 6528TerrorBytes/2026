// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.AbsoluteEncoder;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.mechanisms.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonMotor extends SubsystemBase {
  public final TalonFX m_motor;

  public boolean m_disable = true;

  /** Creates a new SparkMove. */
  public TalonMotor(int motorId, TalonFXConfiguration constants) {
    m_motor = new TalonFX(motorId, "canivore");

    // this mayhaps cause problems later
    //m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motor.getConfigurator().apply(constants);
    SmartDashboard.putNumber("Shooter Speed", m_motor.getVelocity().getValueAsDouble());
  }

  @Override
  public void periodic() {
    // Sets reference of motor higher/lower based on the constant offset (gravity)
    // Example: https://github.com/STMARobotics/frc-7028-2023/blob/5578980e795de4744cb163f76aa883e8ba2c35d5/src/main/java/frc/robot/subsystems/WristSubsystem.java
    // May not be necessary because the new position control is now based on position rather rather than velocity.
    // if (m_motor.get() != 0) { }
    //System.out.println(m_motor.getVelocity().getValueAsDouble());
    if (m_disable) {
      m_motor.setControl(new VelocityVoltage(m_motor.getVelocity().getValueAsDouble() * 0.9));
    }
  }

  public void disable() {
    m_disable = true;
    //Constants.MotorConfig.talonConfig.Slot0.kV = 0;
    //m_motor.getConfigurator().apply(Constants.MotorConfig.talonConfig);
    //m_motor.set(0);
  }

  public void setPower(double rps) {
    m_disable = false;
    m_motor.setControl(new VelocityVoltage(rps));
  }

  public void forwards() {
    m_disable = false;
    m_motor.setControl(new VelocityVoltage(100)); //was 135
    //m_motor.set(1);
  }

  public void backwards() {
    m_disable = false;
    m_motor.setControl(new VelocityVoltage(-100));
    //m_motor.set(-1);
  }
}
