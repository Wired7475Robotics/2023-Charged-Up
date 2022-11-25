// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.FileSystems;
import java.nio.file.Path;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  public static DriveTrain drivetrain;
  public static Shooter shooter;
  public static Intake intake;
  public static Pneumatics PnuLifts;
  public static Lifts lifts;
  public static Encoder leftEncoder;
  public static Encoder rightEncoder;
  public static ShuffleboardTab autoTab;
  NetworkTableEntry leftEncoderData;
  NetworkTableEntry rightEncoderData;
  public static Timer timer;
  private String m_autoSelected;
  public static Controll oi;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private SequentialCommandGroup autonomusCommands;
  private SequentialCommandGroup autonomusIntake;
  private ParallelCommandGroup autonomusShooter;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    oi = new Controll();
    drivetrain = new DriveTrain();
    drivetrain.setDefaultCommand( new ArcadeDrive());
    shooter = new Shooter();
    shooter.setDefaultCommand( new Shoot());
    intake = new Intake();
    intake.setDefaultCommand( new RunIntake());
    PnuLifts = new Pneumatics();
    PnuLifts.setDefaultCommand( new PneumaticLifts());
    lifts = new Lifts();
    lifts.setDefaultCommand( new runLifts());

    leftEncoder = new Encoder(2, 3, false , EncodingType.k4X);
    leftEncoder.setDistancePerPulse(18.8 / 2048.0);
    leftEncoder.setMaxPeriod(0.1);
    leftEncoder.setMinRate(5);
    leftEncoder.setSamplesToAverage(4);
    leftEncoder.setReverseDirection(false);
   
    rightEncoder = new Encoder(0, 1,true, EncodingType.k4X);
    rightEncoder.setDistancePerPulse(18.8 / 2048.0);
    rightEncoder.setMaxPeriod(0.1);
    rightEncoder.setMinRate(5);
    rightEncoder.setSamplesToAverage(4);
    rightEncoder.setReverseDirection(true);

    autoTab = Shuffleboard.getTab("PID");
    leftEncoderData = autoTab.add("Left Encoder", leftEncoder.getDistance()).getEntry();
    rightEncoderData = autoTab.add("Right Encoder", rightEncoder.getDistance()).getEntry();
    timer = new Timer();
    timer.start();

    autonomusIntake = new SequentialCommandGroup(new AutoDelay(5), new AutoFeed(5));
    autonomusShooter = new ParallelCommandGroup(autonomusIntake, new AutoShooter(10));

    autonomusCommands = new SequentialCommandGroup(new AutoDrive(48),  new AutoFeed(5)/*autonomusShooter*/);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    leftEncoderData.setDouble(leftEncoder.getDistance());
    rightEncoderData.setDouble(rightEncoder.getDistance());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    CommandScheduler.getInstance().schedule(autonomusCommands);
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
