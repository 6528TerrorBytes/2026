// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.type.PlaceholderForType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import au.grapplerobotics.MitoCANdria;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.teleop.TeleopFaceAprilTag;
import frc.robot.commands.swervedrive.teleop.AutoShooterDistance;
import frc.robot.commands.swervedrive.teleop.TalonMove;
import frc.robot.commands.swervedrive.teleop.stupidCommands.ArmMove;
import frc.robot.commands.swervedrive.teleop.stupidCommands.HoodMove;
import frc.robot.commands.swervedrive.teleop.stupidCommands.IntakeMove;
import frc.robot.commands.swervedrive.teleop.stupidCommands.ZushiMove;
import frc.robot.subsystems.swervedrive.BasicSparkMotor;
import frc.robot.subsystems.swervedrive.IntakeArm;
//import frc.robot.subsystems.swervedrive.CANdleSystem;
//import frc.robot.subsystems.swervedrive.StupidMotor;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.TalonMotor;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.robot.subsystems.swervedrive.Hood;
import frc.utils.JoystickAnalogButton;
import frc.utils.JoystickMultiAnalogButton;
import frc.utils.JoystickMultiButton;
import java.io.File;

import javax.swing.plaf.SliderUI;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream; 



import java.util.function.Supplier;

//Mr guy, you have to re-zero the hood and tune the angles today, also the shooting during auton doesn't work, good luck

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  //something pretty cool dude!
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // Is currently Xbox Controller
  final         Joystick leftJoystick = new Joystick(0);
  final         Joystick rightJoystick = new Joystick(1);
  final         Joystick otherController = new Joystick(2);
  
  //final MitoCANdria mito = new MitoCANdria(11);

  //private final CANdleSystem m_candleSubsystem = new CANdleSystem();
  private final BasicSparkMotor m_intakeRoller = new BasicSparkMotor(Constants.MotorIDs.intakeTeethID);
  private final BasicSparkMotor m_zushiMotor = new BasicSparkMotor(Constants.MotorIDs.zushiID);
  private final TalonMotor m_launcherMotor1 = new TalonMotor(Constants.MotorIDs.launcher1ID, Constants.MotorConfig.talonConfig);
  private final TalonMotor m_launcherMotor2 = new TalonMotor(Constants.MotorIDs.launcher2ID, Constants.MotorConfig.talonConfig);
  private final IntakeArm m_intakeArmMotor = new IntakeArm();
  private final Hood m_hood = new Hood();

  public double driveLimit = 1;

  //Might not be used anymore, who knows
  public double driveRotation = 0;

  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/falcon"));
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> rightJoystick.getY() * -driveLimit,
                                                                () -> rightJoystick.getX() * -driveLimit)
                                                            .withControllerRotationAxis(
                                                              () -> (leftJoystick.getZ() 
                                                             + SwerveSubsystem.newRotation
                                                              ) * -0.8
                                                            )
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(rightJoystick::getX,
                                                                                             rightJoystick::getY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -leftJoystick.getY(),
                                                                        () -> -leftJoystick.getX())
                                                                    .withControllerRotationAxis(() -> leftJoystick.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  rightJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  rightJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    //Intake Auto
    NamedCommands.registerCommand("Intake Forward", m_intakeRoller.onIntakeCmd());
    NamedCommands.registerCommand("Intake Backwards", m_intakeRoller.onOutakeCmd());

    //Zushi Auto
    NamedCommands.registerCommand("Zushi Forward", new InstantCommand(() ->  m_zushiMotor.onBackwards()));
    NamedCommands.registerCommand("Zushi Backwards", new InstantCommand(() ->  m_zushiMotor.onForward()));
    NamedCommands.registerCommand("Zushi Disable", new InstantCommand(() ->  m_zushiMotor.disable()));

    //Shooter Auto
    NamedCommands.registerCommand("Shooter Forward", 
    new ParallelCommandGroup(
      new HoodMove(m_hood, Constants.Setpoints.hoodAngle180Spot, false),
      new TalonMove(m_launcherMotor1, -42, true),
      new TalonMove(m_launcherMotor2, -42, true),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ZushiMove(m_zushiMotor, true, true)
      )));
    NamedCommands.registerCommand("Shooter Disable", shooterStopCmd());

    //Hood Auto

    //Intake Arm Auto
    NamedCommands.registerCommand("Intake Arm Up", armAngleUpCmd());
    NamedCommands.registerCommand("Intake Arm Vertical", armAngleVerticalCmd());
    NamedCommands.registerCommand("Intake Arm Down", armAngleDownCmd());

    //I initialized it ingerdingus

    // Configure the trigger bindings

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //ALWAYS AVOID USING EVENTTRIGGERS!!!!!     THEY ARE EVIL!!!! USE NAMED COMMANDS INSTEAD!!!! THEY GOOD!!!!
    //ALSO EVENT MARKERS IN PATHPLANNER ARE EVIL TOO!!!! DO NOT USE!!!


    // new EventTrigger("IntakeDown").whileTrue(new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleIntake, false));
    // new EventTrigger("IntakeUp").whileTrue(new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleBack, false));
    // new EventTrigger("IntakeVertical").whileTrue(new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleVertical, false));

    // new EventTrigger("IntakeRun").whileTrue(new IntakeMove(m_intakeRoller, true, false));

    // new EventTrigger("ZushiRun").whileTrue(new ZushiMove(m_zushiMotor, true, false));
    // new EventTrigger("ReverseZushiRun").whileTrue(new ZushiMove(m_zushiMotor, false, true));

    // new EventTrigger("FaceAprilTag").whileTrue(new TeleopFaceAprilTag());

    // new EventTrigger("ZeroGyro").onTrue(new InstantCommand(() -> drivebase.zeroGyroWithAlliance()));

    // new EventTrigger("Launch").whileTrue(new ParallelCommandGroup(
    //   new TalonMove(m_launcherMotor1, -100, false),
    //   new TalonMove(m_launcherMotor2, -100, false),
    //   new SequentialCommandGroup(
    //     new WaitCommand(0.5),
    //     new ZushiMove(m_zushiMotor, true, false)
    //   )
    // ));

    // new EventTrigger("ShooterRun").whileTrue(new ParallelCommandGroup(
    //   new TalonMove(m_launcherMotor1, -100, false),
    //   new TalonMove(m_launcherMotor2, -100, false)
    // ));

    // new EventTrigger("ReverseShooterRun").whileTrue(new ParallelCommandGroup(
    //   new TalonMove(m_launcherMotor1, 100, false),
    //   new TalonMove(m_launcherMotor2, 100, false)
    // ));

    // new EventTrigger("StopShoot").whileTrue(new ParallelCommandGroup(
    //   new TalonMove(m_launcherMotor1, -100, true),
    //   new TalonMove(m_launcherMotor2, -100, true),
    //   new ZushiMove(m_zushiMotor, true, true)
    // ));

    // new EventTrigger("CloseShoot").whileTrue(new ParallelCommandGroup(
    //   new TalonMove(m_launcherMotor1, -100, false),
    //   new TalonMove(m_launcherMotor2, -100, false),
    //   new HoodMove(m_hood, Constants.Setpoints.hoodAngle180Spot)
    // ));
    

    // new EventTrigger("FarShoot").whileTrue(new ParallelCommandGroup(
    //   new TalonMove(m_launcherMotor1, -100, false),
    //   new TalonMove(m_launcherMotor2, -100, false),
    //   new HoodMove(m_hood, Constants.Setpoints.hoodAngleFarShoot),
    //   new SequentialCommandGroup(
    //     new WaitCommand(0.5),
    //     new ZushiMove(m_zushiMotor, true, false)
    //   )
    // ));
    autoChooser = AutoBuilder.buildAutoChooser();

    //Woah, it allows us to select autos, woah!
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //new JoystickButton(leftJoystick, 5).whileTrue(new InstantCommand(() -> System.out.println(Utility.getYaw())));

    //Zero Robot, doesn't work anymore?
    new JoystickButton(rightJoystick, 4).whileTrue(new InstantCommand(() -> drivebase.zeroGyroWithAlliance()));
    
    //Drive Speed Change
    new JoystickButton(rightJoystick, 2).whileTrue(new InstantCommand(() -> driveLimit = 0.5));
    new JoystickButton(rightJoystick, 2).onFalse(new InstantCommand(() -> driveLimit = 1));

    //B button; Intake in
    new JoystickButton(otherController, 2).whileTrue(m_intakeRoller.onIntakeCmd());

    //Intake/Feed out
    new JoystickButton(otherController, 6).whileTrue(new ParallelCommandGroup(
      new IntakeMove(m_intakeRoller, false),
      new ZushiMove(m_zushiMotor, false, true)
    ));

    //new JoystickButton(otherController, 2).whileTrue(new ZushiMove(m_zushiMotor, true));
    
    //Automatically faces the center hub
    new JoystickButton(leftJoystick, 1).whileTrue(new TeleopFaceAprilTag());

    //Left Trigger; Scopes in the robot, does auto hood adjustment and shooter speed adjustment based off distance
    new JoystickAnalogButton(otherController, 2, 0.3, 1.3).whileTrue(new ParallelCommandGroup(
      new AutoShooterDistance(),
      new HoodMove(m_hood, RobotContainer.drivebase.getHoodAngle(), true)
    ));

    //Shooting Preset adjustments via D-pad
    new POVButton(otherController, 0).whileTrue(new ParallelCommandGroup(
      new HoodMove(m_hood, -7.5, false),
      m_launcherMotor1.closeCmd(),
      m_launcherMotor2.closeCmd(),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ZushiMove(m_zushiMotor, true, true)
      )
    ));
    new POVButton(otherController, 90).whileTrue(new ParallelCommandGroup(
      new HoodMove(m_hood, -15, false),
      m_launcherMotor1.midCmd(),
      m_launcherMotor2.midCmd(),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ZushiMove(m_zushiMotor, true, true)
      )
    ));
    new POVButton(otherController, 180).whileTrue(new ParallelCommandGroup(
      new HoodMove(m_hood, Constants.Setpoints.hoodAngle180Spot, false),
      m_launcherMotor1.farCmd(),
      m_launcherMotor2.farCmd(),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ZushiMove(m_zushiMotor, true, true)
      )
    ));
    new POVButton(otherController, 270).whileTrue(new ParallelCommandGroup(
      new HoodMove(m_hood, Constants.Setpoints.hoodAngle270Spot, false),
      m_launcherMotor1.feedCmd(),
      m_launcherMotor2.feedCmd(),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ZushiMove(m_zushiMotor, true, true)
      )
    ));

    //Right Trigger shoot
    new JoystickAnalogButton(otherController, 3, 0.3, 1.3).whileTrue(new ParallelCommandGroup(
      m_launcherMotor1.shootCmd(),
      m_launcherMotor2.shootCmd(),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ZushiMove(m_zushiMotor, true, true)
      )
    ));

    //new JoystickButton(otherController, 3).whileTrue(new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleTest));

    //X, A, Y buttons; Arm Angle Positions, Vertical, All the way down, and All the way up.
    new JoystickButton(otherController, 3).whileTrue(new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleVertical, false));
    new JoystickButton(otherController, 1).whileTrue(new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleIntake, false));
    new JoystickButton(otherController, 4).whileTrue(new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleBack, false));


    //Left Bumper, Hood angle all the way back
    new JoystickButton(otherController, 5).whileTrue(new HoodMove(m_hood, Constants.Setpoints.hoodAngleHorizontal, false));

    new JoystickButton(otherController, 9).whileTrue(new InstantCommand(() -> System.out.println("Hood Angle: " + RobotContainer.drivebase.getHoodAngle())));
    new JoystickButton(otherController, 9).whileTrue(new InstantCommand(() -> System.out.println("AutoSpeed: " + RobotContainer.drivebase.getShooterRPS())));
    new JoystickButton(otherController, 9).whileTrue(new InstantCommand(() -> System.out.println("Distance: " + RobotContainer.drivebase.getDistance())));
    new JoystickButton(otherController, 9).whileTrue(new InstantCommand(() -> System.out.println("Attempted Hood Angle: " + m_hood.m_encoder.getPosition())));


    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      //Was start, is now 5
      new JoystickButton(otherController, 0).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      new JoystickButton(otherController, 3).whileTrue(drivebase.sysIdDriveMotorCommand());
      new JoystickButton(otherController, 2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));


      // leftJoystick.button(1);

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      //This is temporary, fix later :)
      //List of previous button orders, x y start back leftBumper rightBumper
      // otherController.button(3).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // otherController.button(4).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      // otherController.button(5).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // otherController.button(6).whileTrue(drivebase.centerModulesCommand());
      // otherController.button(7).onTrue(Commands.none());
      // otherController.button(8).onTrue(Commands.none());
    } else
    {
      // List of button orders again, a x start back leftBumper rightBumper
      // otherController.button(9).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // otherController.button(3).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // otherController.button(5).whileTrue(Commands.none());
      // otherController.button(6).whileTrue(Commands.none());
      // otherController.button(7).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // otherController.button(8).onTrue(Commands.none());
      // !Six Seven;
      // !Alex was here;
      // Brayden+Tool=Bad;
      // Not Alex || Knot Alex?
      //
      // if (liam == 8th Grader) {
      //    robot.explode();
      // }
    }

    
  }


  public Command armAngleUpCmd(){
    return new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleBack, false);
  }

  public Command armAngleVerticalCmd(){
    return new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleVertical, false);
  }

  public Command armAngleDownCmd(){
    return new ArmMove(m_intakeArmMotor, Constants.Setpoints.armAngleIntake, false);
  }

  public Command shooterStopCmd() {
     return new ParallelCommandGroup(
      new InstantCommand(() -> m_launcherMotor1.disable()),
      new InstantCommand(() -> m_launcherMotor2.disable()),
      new InstantCommand(() ->  m_zushiMotor.disable())
    );
  }

  private final SendableChooser<Command> autoChooser;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");

    return autoChooser.getSelected();

    // return new RunCommand(() -> {
    //   System.out.println("yaw " + drivebase.getAngle());
    // });
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
