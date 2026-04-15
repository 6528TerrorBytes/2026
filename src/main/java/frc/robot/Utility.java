// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.subsystems.swervedrive.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

// The data you can get from the Limelight: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
 
public final class Utility {

  private static final double m_limeLightHeight = 37.846; //cm

  private static final double m_aprilTagHeight = 112.395; //cm

  private static final double m_limelightAngle = 15; //degrees

  public static boolean teamColorIsRed() {
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red; 
  }

  public static double clampNum(double num, double min, double max) {
    if (num > max)      { return max; }
    else if (num < min) { return min; }
    else                { return num; }
  }
  
  // System clock time in seconds
  public static double getTime() {
    return Timer.getFPGATimestamp();
  }

  public static double getMatchTime() {
    return Timer.getMatchTime();
  }

  public static double calcSpeedFaceTag(double tx) {
    double rotationSpeed;
    if (Math.abs(tx) > 40) {
      rotationSpeed = tx / 60;
    } else if (Math.abs(tx) > 20) {
      rotationSpeed = tx / 40;
    } else {
      rotationSpeed = tx / 20;
    }

    // if (Math.abs(tx) <= Constants.ShooterConstants.crossPoint) {
    //   // Between +- where the functions cross, use linear
    //   rotationSpeed = -tx / Constants.ShooterConstants.linearDivider;
    // } else { 
    //   // Use cubic since it's outside the inner range
    //   rotationSpeed = -Math.pow(tx, 3) / Constants.ShooterConstants.cubicDivider;
    // }

    //rotationSpeed *= Constants.ShooterConstants.speedScale;
    return clampNum(rotationSpeed, -1, 1) * 0.625;
  }

  public static double getDistance(double TY) {
    double angleToTarget = m_limelightAngle + TY;
    double heightDisplacement = m_aprilTagHeight - m_limeLightHeight;

    return heightDisplacement / Math.tan(Math.toRadians(angleToTarget));
  }

  // public static double getYaw() {
  //   var result = camera.getLatestResult();

  //   boolean hasTargets = result.hasTargets();

  //   if (!hasTargets) {
  //     return 0;
  //   }

  //   //List<PhotonTrackedTarget> targets = result.getTargets();

  //   PhotonTrackedTarget target = result.getBestTarget();

  //   return target.getYaw();
  // }
}
