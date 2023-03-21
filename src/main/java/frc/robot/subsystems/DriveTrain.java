package frc.robot.subsystems;

import java.net.CacheRequest;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase{
    private static final double DRIVECONV = 14;
    public CANSparkMax leftDrive1;
    public CANSparkMax leftDrive2;
    public CANSparkMax rightDrive1;
    public CANSparkMax rightDrive2;
    private static double xSpeed;
    private static double zRotation;
    private static MotorControllerGroup m_leftDrive;
    private static MotorControllerGroup m_rightDrive;
    public static DifferentialDrive drivetrain;
    private static double MED_SPEED_COEFF = 0.6;
    private static double LOW_SPEED_COEFF = 0.4;
    private static double HIGH_SPEED_COEFF = 0.8;
    private static double TIMEOUT = 15;
    private static PhotonCamera camera;
    private static PhotonCamera camera2;
    public final double[] AREA = {0,0};
    public final int CUBE = 0;
    public final int CONE = 1;
    private final double MAX_LIN_SPEED = 0.25;
    private final double MAX_ROT_SPEEED = 0.25;
    public Timer timer;
    public DriveTrain() {
        timer = new Timer();
        leftDrive1 = new CANSparkMax(1, MotorType.kBrushless);
        leftDrive2 = new CANSparkMax(9, MotorType.kBrushless);
        rightDrive1 = new CANSparkMax(10, MotorType.kBrushless);
        rightDrive2 = new CANSparkMax(19, MotorType.kBrushless);
        leftDrive1.setOpenLoopRampRate(0.75);
        leftDrive2.setOpenLoopRampRate(0.75);
        rightDrive1.setOpenLoopRampRate(0.75);
        rightDrive2.setOpenLoopRampRate(0.75);
        m_leftDrive = new MotorControllerGroup(leftDrive1, leftDrive2);
        m_rightDrive = new MotorControllerGroup(rightDrive1, rightDrive2);
        m_leftDrive.setInverted(true);
        drivetrain = new DifferentialDrive(m_leftDrive,m_rightDrive);

        camera = new PhotonCamera("Cam1");
        camera2 = new PhotonCamera("Cam2");
        camera2.setDriverMode(true);
    }

    public void teleDrive() {
        
        if (Controll.getDriveBumper(Controll.RIGHT)){
            xSpeed = Controll.getDriveLeftStick(Controll.Y);
            zRotation = Controll.getDriveRightStick(Controll.X);
        } else if (Controll.getDriveBumper(Controll.LEFT)){
            xSpeed = Controll.getDriveLeftStick(Controll.Y) * MED_SPEED_COEFF;
            zRotation = Controll.getDriveRightStick(Controll.X) * MED_SPEED_COEFF;
        } else if (Controll.getDriveTrigger(Controll.RIGHT)){
            xSpeed = Controll.getDriveLeftStick(Controll.Y) *LOW_SPEED_COEFF;
            zRotation = Controll.getDriveRightStick(Controll.X) * LOW_SPEED_COEFF;
        }  else {
            xSpeed = Controll.getDriveLeftStick(Controll.Y) * HIGH_SPEED_COEFF;
            zRotation = Controll.getDriveRightStick(Controll.X) * HIGH_SPEED_COEFF;
        }




        drivetrain.arcadeDrive(-xSpeed*1, -zRotation*-1);
        drivetrain.feed();
    }

    public void targetCone() {
        if (camera.getPipelineIndex() != 1){
            camera.setPipelineIndex(1);
        }
        double rotation;
        var result = camera.getLatestResult();
        if (result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            rotation = target.getYaw()*0.0175;
        } else {
            rotation = 0;
        }
        drivetrain.arcadeDrive(0, rotation);
    }

    public void Vision(){
        return;
        
    }

    public PhotonTrackedTarget findCube() {

        if (camera.getPipelineIndex() != 0){
            camera.setPipelineIndex(0);
        }
        var result = camera.getLatestResult();
        if (result.hasTargets()){
            return result.getBestTarget();
        }
        
        return null;
    }
    public boolean targetCube(PIDController linPID, PIDController anglePID, double maxlinError, double maxAngError) {
        PhotonTrackedTarget target = findCube();

        if(target == null)
            return true;

        double linSpeed = linPID.calculate(target.getArea()) / maxlinError;
        double angSpeed = anglePID.calculate(target.getYaw()) / maxAngError;

        m_leftDrive.set((linSpeed + angSpeed) * 0.5 );
        m_rightDrive.set((linSpeed - angSpeed) * 0.5 );


        return linPID.atSetpoint() && anglePID.atSetpoint();
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
        if (timer.get() > 1){
            return true;
        }
        //The linear speed, derived from the pid loop output
        double linear_speed = drivePid.calculate(Robot.leftEncoder.getPosition() , target*DRIVECONV);
        double angular_error;
        //An error factor to ensure the robot drives straigmht
        if(Math.abs(Robot.rightEncoder.getPosition() * DRIVECONV) < 0.5 && Math.abs(Robot.leftEncoder.getPosition() * DRIVECONV) < 0.5)
           angular_error = 1;
        else
           angular_error = Robot.leftEncoder.getPosition() / Robot.rightEncoder.getPosition() ;

        linear_speed = linear_speed > MAX_LIN_SPEED ? MAX_LIN_SPEED : linear_speed ;

        //Drive the motor
        m_leftDrive.set(-(linear_speed * (1*angular_error)/10));
        m_rightDrive.set(-(linear_speed * angular_error)/10);
        System.out.println(drivePid.atSetpoint());
        return drivePid.atSetpoint();

    }

    public boolean autoTurn(double target, PIDController anglepid) {        
        double output = anglepid.calculate(Robot.absAngle(), target);

        if (output > MAX_ROT_SPEEED) {
            output = MAX_ROT_SPEEED;
        } else if(output < -MAX_ROT_SPEEED) {
            output = -MAX_ROT_SPEEED;
        }

        m_leftDrive.set(-output);
        m_rightDrive.set(output);

        return anglepid.atSetpoint();
    }

}
