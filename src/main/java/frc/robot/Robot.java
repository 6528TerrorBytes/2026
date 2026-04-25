// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.RGBWColor;
import com.pathplanner.lib.events.EventScheduler;

import java.awt.Color;



import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix6.signals.*;
import org.w3c.dom.css.*;
/*import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;  */

//import au.grapplerobotics.CanBridge;

//import com.ctre.phoenix6.controls


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  final TalonFX m_frontLeftDrive = new TalonFX(1);
  final TalonFX m_frontRightDrive = new TalonFX(3);
  final TalonFX m_backLeftDrive = new TalonFX(5);
  final TalonFX m_backRightDrive = new TalonFX(7);

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private static CANdle m_candle = new CANdle(30, "canivore");

  public static SolidColor blueSolid = new SolidColor(0, 113).withColor(new RGBWColor(0, 0, 255));
  public static SolidColor redSolid = new SolidColor(0, 113).withColor(new RGBWColor(255, 0, 0));
  public static StrobeAnimation blueStrobe = new StrobeAnimation(0, 113).withColor(new RGBWColor(0, 0, 255));
  public static StrobeAnimation redStrobe = new StrobeAnimation(0, 113).withColor(new RGBWColor(255, 0, 0));
  public static LarsonAnimation knightMode = new LarsonAnimation(55, 113).withColor(new RGBWColor(255, 0, 0)).withSize(6);
  public static SolidColor stopCandle = new SolidColor(0, 8).withColor(new RGBWColor(0, 0, 0));
  //box end 54, ends at 113

  public Robot()
  {
    //CanBridge.runTCP();
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();

    if (Utility.teamColorIsRed()) {
      m_candle.setControl(redSolid);
    } else {
      m_candle.setControl(blueSolid);
    }
    m_candle.setControl(knightMode);
    m_candle.setControl(stopCandle);
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
      //m_autonomousCommand.end(true);
    } 
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    if (
    (DriverStation.getMatchTime() <= 133 && DriverStation.getMatchTime() > 130) || 
    (DriverStation.getMatchTime() <= 108 && DriverStation.getMatchTime() > 105) || 
    (DriverStation.getMatchTime() <= 83 && DriverStation.getMatchTime() > 80) || 
    (DriverStation.getMatchTime() <= 58 && DriverStation.getMatchTime() > 55) || 
    (DriverStation.getMatchTime() <= 33 && DriverStation.getMatchTime() > 30) || 
    DriverStation.getMatchTime() <= 8) {
      
      if (Utility.teamColorIsRed()) {
        m_candle.setControl(redStrobe);
      } else {
      m_candle.setControl(blueStrobe);
      }

    } else {

      if (Utility.teamColorIsRed()) {
        m_candle.setControl(redSolid);
      } else {
      m_candle.setControl(blueSolid);
      }

    }
    m_candle.setControl(stopCandle);
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
