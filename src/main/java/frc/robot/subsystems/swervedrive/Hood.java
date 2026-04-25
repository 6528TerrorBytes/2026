// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import frc.robot.Constants;

public class Hood extends WPIArm {
  public Hood() {
    super(
      Constants.MotorIDs.hoodID, Constants.MotorConfig.hoodConfig, Constants.MotorConfig.hoodTolerance,
      0, 0, // Max velocity, Max acceleration
      0.11, 0, 0, // PID
      0, 0.45, 0 // S, G, V (Arm Feedforward)
    );

    //enable();
    setGoal(Constants.Setpoints.hoodAngleBack);
  }

  public double testPoint = 0;

  @Override
  public double getAngleFromHorizontal(double encoderPos) {
    return encoderPos - Constants.Setpoints.hoodAngleHorizontal;
  }

  public void changeTestPoint(double change) {
    testPoint += change;
  }

  public void test() {
    setGoal(testPoint);
  }
}