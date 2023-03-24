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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Auto3";
  private static final String kCustomAuto = "Auto1";
  private static final String kCubePipeline = "Cube";
  private static final String kConePipeline ="Cone";
  public static DriveTrain drivetrain;
  public static Arm arm;
  public static Lift lift;
  public static Claw claw;
  public static RelativeEncoder leftEncoder;
  public static RelativeEncoder rightEncoder;
  public static SparkMaxLimitSwitch ExtenderLimit;
  public static ShuffleboardTab autoTab;
  public static Timer timer;
  public static AHRS navX;
  private String m_autoSelected;
  public static Controll oi;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public final SendableChooser<String> v_chooser = new SendableChooser<>();
  private SequentialCommandGroup autonomus1Commands;
  private SequentialCommandGroup autonomus2Commands;
  private SequentialCommandGroup visionCommands;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Auto3", kDefaultAuto);
    m_chooser.addOption("Auto1", kCustomAuto);
    v_chooser.setDefaultOption("Cube", kCubePipeline);
    v_chooser.addOption("Cone", kConePipeline);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData("Vision Pipeline", v_chooser);

    
    drivetrain = new DriveTrain();
    drivetrain.setDefaultCommand( new ArcadeDrive());
    arm = new Arm();
    lift = new Lift();
    lift.setDefaultCommand(new ArmController());
    claw = new Claw();
    claw.setDefaultCommand(new ClawControll());
    ExtenderLimit = lift.Extender.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    leftEncoder = drivetrain.leftDrive1.getEncoder();

   
    rightEncoder = drivetrain.leftDrive1.getEncoder();

    timer = new Timer();
    timer.start();
    navX = new AHRS();
    navX.calibrate();
    navX.reset();
    SmartDashboard.putNumber("Angle", navX.getAngle());



    autonomus1Commands = new SequentialCommandGroup(
      new AutoLift(40.5),
      new AutoArm(85),
      Commands.waitSeconds(0.5),
      new AutoClaw(0.75),
      new AutoArm(-30),
      new AutoDrive(-2),
      new AutoClaw(0)
    );
    autonomus2Commands = new SequentialCommandGroup(
      new AutoLift(40.5),
      new AutoArm(85),
      Commands.waitSeconds(0.5),
      new AutoClaw(0.75),
      new AutoArm(-30),
      new AutoDrive(-3),
      new AutoClaw(0)
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
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
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
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        CommandScheduler.getInstance().schedule(autonomus2Commands);
      case kDefaultAuto:
        CommandScheduler.getInstance().schedule(autonomus1Commands);
      default:
        // Put default auto code here

    }

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

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

    System.out.println("disabled");
    CommandScheduler.getInstance().cancelAll();
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
