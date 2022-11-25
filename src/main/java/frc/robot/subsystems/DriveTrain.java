package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase{
    private static WPI_VictorSPX leftDrive1;
    private static WPI_VictorSPX leftDrive2;
    private static WPI_TalonSRX rightDrive1;
    private static WPI_TalonSRX rightDrive2;
    private static double xSpeed;
    private static double zRotation;
    private static MotorControllerGroup m_leftDrive;
    private static MotorControllerGroup m_rightDrive;
    private static DifferentialDrive drivetrain;
    private static double MED_SPEED_COEFF = 0.6;
    private static double LOW_SPEED_COEFF = 0.4;
    private static double HIGH_SPEED_COEFF = 0.7;
    private static double TIMEOUT = 15;
    public DriveTrain() {
        leftDrive1 = new WPI_VictorSPX(9);
        leftDrive2 = new WPI_VictorSPX(13);
        rightDrive1 = new WPI_TalonSRX(14);
        rightDrive2 = new WPI_TalonSRX(15);

        m_leftDrive = new MotorControllerGroup(leftDrive1, leftDrive2);
        m_rightDrive = new MotorControllerGroup(rightDrive1, rightDrive2);
        drivetrain = new DifferentialDrive(m_leftDrive,m_rightDrive);
    }

    public void teleDrive() {
        if (Controll.getDriveBumper(Controll.RIGHT)){
            xSpeed = Controll.getDriveRightStick(Controll.X);
            zRotation = Controll.getDriveLeftStick(Controll.Y);
        } else if (Controll.getDriveBumper(Controll.LEFT)){
            xSpeed = Controll.getDriveRightStick(Controll.X) * MED_SPEED_COEFF;
            zRotation = Controll.getDriveLeftStick(Controll.Y) * MED_SPEED_COEFF;
        } else if (Controll.getDriveTrigger(Controll.RIGHT)){
            xSpeed = Controll.getDriveRightStick(Controll.X) *MED_SPEED_COEFF;
            zRotation = Controll.getDriveLeftStick(Controll.Y) * LOW_SPEED_COEFF;
        }  else {
            xSpeed = Controll.getDriveRightStick(Controll.X) * HIGH_SPEED_COEFF;
            zRotation = Controll.getDriveLeftStick(Controll.Y) * HIGH_SPEED_COEFF;
        }


        drivetrain.arcadeDrive(xSpeed, -zRotation);
        drivetrain.feed();
    }

    /**An autonomus drive function
     * 
     * @param target
     * The target distance, in inches
     * @param drivePid
     * The pid object to drive the motors
     * @return
     */
    public boolean autoDrive(double target, PIDController drivePid) {

        //The linear speed, derived from the pid loop output
        double linear_speed = drivePid.calculate(Robot.leftEncoder.getDistance(), target);
        double angular_error;
        //An error factor to ensure the robot drives straight
        if(Math.abs(Robot.rightEncoder.getDistance()) < 0.1)
           angular_error = 1;
        else
           angular_error = Robot.leftEncoder.getDistance() / Robot.rightEncoder.getDistance();

        //Final motor speed
        double speed_factor = linear_speed * angular_error / 5;

        //Drive the motors
        m_leftDrive.set(speed_factor);
        m_rightDrive.set(-speed_factor);
        System.out.println(drivePid.getSetpoint() + ":" + Robot.leftEncoder.getDistance() + ":" + linear_speed);
        return drivePid.atSetpoint();
    }
/*
    public void autoTurn(double target) {        
        PIDController anglepid = new PIDController(0.01, 0.0, 0.0);
        anglepid.enableContinuousInput(-180, 180);
    }
    */
}
