// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.FileSystems;
import java.nio.file.Path;

import edu.wpi.first.networktables.GenericEntry;
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
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kCubePipeline = "Cube";
  private static final String kConePipeline ="Cone";
  public static DriveTrain drivetrain;
  public static Encoder leftEncoder;
  public static Encoder rightEncoder;
  public static ShuffleboardTab autoTab;;
  public static Timer timer;
  public static AHRS navX;
  private String m_autoSelected;
  public static Controll oi;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public final SendableChooser<String> v_chooser = new SendableChooser<>();
  private SequentialCommandGroup autonomusCommands;
  private SequentialCommandGroup visionCommands;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    v_chooser.setDefaultOption("Cube", kCubePipeline);
    v_chooser.addOption("Cone", kConePipeline);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData("Vision Pipeline", v_chooser);
    drivetrain = new DriveTrain();
    drivetrain.setDefaultCommand( new ArcadeDrive());

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

    timer = new Timer();
    timer.start();
    navX = new AHRS();
    navX.calibrate();
    navX.reset();
    SmartDashboard.putNumber("Angle", navX.getAngle());



    autonomusCommands = new SequentialCommandGroup(
    new AutoDrive(24),
    new AutoTurn(45),
    new AutoDrive(24),
    new AutoTurn(-45),
    new AutoDrive(25),
    new AutoTurn(180)
    );
    //visionCommands =  new SequentialCommandGroup(new );

    oi = new Controll();
    
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
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("Raw Angle", navX.getAngle());
    SmartDashboard.putNumber("Absolute Angle", absAngle());
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
    navX.reset();
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
  public void teleopInit() {
  /** This function is called periodically during operator control. */
  }
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    CommandScheduler.getInstance().cancel(autonomusCommands);
  }
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

  public static double absAngle() {
  double outAngle = navX.getAngle() % 360;

  if(outAngle < 0)
    outAngle += 360;

  return outAngle;
  }

  public static double absAngle(double angle) {
    double outAngle = angle % 360;

    if(outAngle < 0)
      outAngle += 360;
  
    return outAngle;
    }

}
