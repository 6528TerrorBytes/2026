// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.lang.reflect.Array;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class IntakeArm extends WPIArm {
  public IntakeArm() {
    super(
      Constants.MotorIDs.intakeArmID, Constants.MotorConfig.armConfig, Constants.MotorConfig.armTolerance,
      Constants.PIDPresets.regArmPreset[1], Constants.PIDPresets.regArmPreset[2], // Max velocity, Max acceleration
      Constants.PIDPresets.regArmPreset[3], Constants.PIDPresets.regArmPreset[4], Constants.PIDPresets.regArmPreset[5], // PID p 0.15   i 0
      Constants.PIDPresets.regArmPreset[6], Constants.PIDPresets.regArmPreset[7], Constants.PIDPresets.regArmPreset[8] // S, G, V (Arm Feedforward) //0.23
    );
    //this lets us change the PID values mid match (Constants at line 299 - 311 & RobotContainer at line 352)
    //enable();
    setGoal(Constants.Setpoints.armAngleBack);
  }

  @Override
  public double getAngleFromHorizontal(double encoderPos) {
    return encoderPos - Constants.Setpoints.armAngleHorizontal;
  }
}