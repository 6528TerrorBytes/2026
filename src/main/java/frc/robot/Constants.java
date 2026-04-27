// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.sql.Array;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import edu.wpi.first.math.Matrix;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static final int CANdleID = 30;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  //private TalonFX m_motor = new TalonFX(17);

  public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

  	public static final class VisionConstants {
		public static final double MAX_POSE_JUMP_METERS = 1.0;
		public static final double POSE_AMBIGUITY_THRESHOLD = 0.2;
		public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;
		public static final double HIGH_LATENCY_THRESHOLD_MS = 100.0;

		public static final class CameraStdDevs {
			public static final double[] SINGLE_TAG = { 4.0, 4.0, 6.0 };
			public static final double[] MULTI_TAG = { 0.5, 0.5, 4.0 };
		}
	}

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class MotorIDs {
    public static final int stupidMotorID = 0;
    public static final int intakeTeethID = 21;
    public static final int zushiID = 23;
    public static final int intakeArmID = 22;
    public static final int launcher1ID = 25;
    public static final int launcher2ID = 26;
    public static final int hoodID = 24;
  }

  public static final class Setpoints {
    public static final double armAngleHorizontal = 0; //Change this to whatever it is when horizontal, 135
    public static final double armAngleIntake = armAngleHorizontal - 147; //Change to arm angle when given no power
    public static final double armAngleBack = armAngleHorizontal - 4; //
    public static final double armAngleVertical = armAngleHorizontal - 40;
    
    public static final double armAngleStartUp = armAngleHorizontal + 115; //Angle that arm moves to upon being enabled
    public static final double armOscillateUp = armAngleHorizontal + 70;
    public static final double armOscillateDown = armAngleHorizontal + 20;

    public static final double hoodAngleHorizontal = 0; //Hard Stop Position
    public static final double hoodAngle0Spot = hoodAngleHorizontal - 7.5;  //Close Angle
    public static final double hoodAngle90Spot = hoodAngleHorizontal - 15;  //Mid Angle
    public static final double hoodAngle180Spot = hoodAngleHorizontal - 20; //Far Angle
    public static final double hoodAngle270Spot = hoodAngleHorizontal - 30; //Feed Angle
  }

  public static final class AprilTags {
    public static final double[] hubBlueTags = { 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28 };
    public static final double[] hubRedTags =  { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

    public static final double[] otherBlueTags = { 29, 30, 31, 32 };
    public static final double[] otherRedTags =  { 13, 14, 15, 16 };

    // X is horizontal distance, Y is distance out from coral aprilTag -- half of bot width (bumpers included)
    public static final Translation2d coralOffsetRight = new Translation2d(0.147, 0.545); // 0.148
    public static final Translation2d coralOffsetLeft = new Translation2d(-0.147, 0.545);
    public static final Translation2d coralOffsetRightLow = new Translation2d(0.147, 0.62);
    public static final Translation2d coralOffsetLeftLow = new Translation2d(-0.147, 0.62);
    
    public static final Translation2d coralOffsetCentered = new Translation2d(-0.06, 0.46);

    public static final double coralXTagOffset = -0.08;

    public static final Translation2d coralCollectOffset = new Translation2d(0.06, 0.515); // 0.035

    public static final PathConstraints aprilTagDriveConstraints = new PathConstraints(0.85, 0.85, 2 * Math.PI, 4 * Math.PI);

    public static Map<Double, Double> aprilTagFaceAngles = new HashMap<Double, Double>();

    static {
      // blue side apriltags 
      aprilTagFaceAngles.put(Double.valueOf(17), Double.valueOf(60));
      aprilTagFaceAngles.put(Double.valueOf(18), Double.valueOf(0));
      aprilTagFaceAngles.put(Double.valueOf(19), Double.valueOf(-60));
      aprilTagFaceAngles.put(Double.valueOf(20), Double.valueOf(-120));
      aprilTagFaceAngles.put(Double.valueOf(21), Double.valueOf(180));
      aprilTagFaceAngles.put(Double.valueOf(22), Double.valueOf(120));

      // red side apriltags
      aprilTagFaceAngles.put(Double.valueOf(11), Double.valueOf(60));
      aprilTagFaceAngles.put(Double.valueOf(10), Double.valueOf(0));
      aprilTagFaceAngles.put(Double.valueOf(9), Double.valueOf(-60));
      aprilTagFaceAngles.put(Double.valueOf(8), Double.valueOf(-120));
      aprilTagFaceAngles.put(Double.valueOf(7), Double.valueOf(180));
      aprilTagFaceAngles.put(Double.valueOf(6), Double.valueOf(120));

      // coral intake apriltags
      aprilTagFaceAngles.put(Double.valueOf(13), Double.valueOf(125));
      aprilTagFaceAngles.put(Double.valueOf(12), Double.valueOf(-125));
      aprilTagFaceAngles.put(Double.valueOf(2), Double.valueOf(55));
      aprilTagFaceAngles.put(Double.valueOf(1), Double.valueOf(-55));
    }
  }

  public static final class MotorConfig {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final double elevatorTolerance = 0.1;

    static {
      elevatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(35)
        .inverted(false);
      elevatorConfig.alternateEncoder
        .positionConversionFactor(1) // 5.95 conversion inches
        .velocityConversionFactor(1) // 5.95 conversion inches
        .inverted(true);
    }
    
    // intake arm
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final double armTolerance = 1; // Might need to be greater to account for error

    static {
      armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(true);
      armConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(false);
    }

    public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();
    public static final double hoodTolerance = 1;

    static {
      hoodConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false);
      hoodConfig.absoluteEncoder
        .positionConversionFactor(360) //was 360 / 0.722222
        .velocityConversionFactor(360 / 60)
        .inverted(false);
    }

    public static final SparkMaxConfig tailArmConfig = new SparkMaxConfig();
    public static final double tailArmTolerance = 2;

    static {
      tailArmConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false);
      tailArmConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(true);
    }

    public static final SparkMaxConfig stupidConfig = new SparkMaxConfig();

    static {
      stupidConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(35)
        .inverted(false);
      stupidConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(true);
      stupidConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.025, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);
        // .positionWrappingMinInput(0)
        // .positionWrappingMaxInput(360);
      stupidConfig.closedLoop.maxMotion
        .cruiseVelocity(360 * 300) // units per minute i think? of the actual motor, not the encoder?
        .maxAcceleration(360 * 500)
        .allowedProfileError(10);
    }

    public static final SparkMaxConfig innerClimbConfig = new SparkMaxConfig();

    static {
      innerClimbConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(35)
        .inverted(true); // this is important or else it'll go the wrong direction away from any setpoints
      innerClimbConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360 / 60)
        .inverted(false);
      innerClimbConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.025, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);
        // .positionWrappingMinInput(0)
        // .positionWrappingMaxInput(360);
      innerClimbConfig.closedLoop.maxMotion
        .cruiseVelocity(360 * 300) // units per minute i think? of the actual motor, not the encoder?
        .maxAcceleration(360 * 500)
        .allowedProfileError(10);
    }

    public static final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    static {
      talonConfig.Slot0.kP = 0.47; // 0.47
      talonConfig.Slot0.kI = 0; // 0
      talonConfig.Slot0.kD = 0; // 0
      talonConfig.Slot0.kS = 0.11; // 0.11
      talonConfig.Slot0.kV = 0.11739; // Feedforward (Volts per RPS) 0.11536
    }
  }

  public static final class PIDPresets {
    public static double[] regArmPreset = new double[] {
    1, //tolerance
    1000, //max vel
    1000, //max accel
    0.11, //p 0.17
    0, //i
    0, //d
    0, //s
    0.23, //g
    0  //v
  };

  public static double[] slowArmPreset = new double[] {
    1, //tolerance
    300, //max vel
    300, //max accel
    0.12, //p
    0, //i
    0, //d
    0, //s
    0.23, //g
    0  //v
  };
  //change these numbers later, probably way off
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class Vision {
    public static final String kCameraName = "YOUR CAMERA NAME";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

     // The standard deviations of our vision estimated poses, which affect correction rate
     // (Fake values. Experiment and determine estimation noise on an actual robot.)
     public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
     public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
