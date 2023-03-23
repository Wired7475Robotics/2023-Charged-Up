package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoDrive  extends CommandBase{
    
    private double target = 0;
    private boolean isDone = false;
    private boolean targetInit = false;
    private double kP = 0.5;
    private double kI = 0;
    private double kD = 0.05;
    PIDController pid = new PIDController(kP, kI, kD);
    

    public AutoDrive() {
        addRequirements(Robot.drivetrain);
    }

    public AutoDrive(double target_) {
        addRequirements(Robot.drivetrain);
        target = -target_;
    }

    @Override
    public void initialize() {

    }

    private void setAllIdleMode(IdleMode mode) {
        Robot.drivetrain.leftDrive1.setIdleMode(mode);
        Robot.drivetrain.leftDrive2.setIdleMode(mode);
        Robot.drivetrain.rightDrive1.setIdleMode(mode);
        Robot.drivetrain.rightDrive2.setIdleMode(mode);
    }

    @Override
    public void execute() {
        if(!targetInit) {
            pid.setSetpoint(target);
            pid.setTolerance(10);
            Robot.leftEncoder.setPosition(0);
            Robot.rightEncoder.setPosition(0);
            Robot.drivetrain.timer.stop();
            Robot.drivetrain.timer.reset();
            Robot.drivetrain.timer.start();
            setAllIdleMode(IdleMode.kBrake);
            targetInit = true;
        }
        
        isDone = Robot.drivetrain.autoDrive(target, pid);
        System.out.println("AutoDrive isDone? " + isDone);
        if (isDone){
            Robot.drivetrain.drivetrain.arcadeDrive(0,0); 
            setAllIdleMode(IdleMode.kCoast);
        }
    }

   @Override
   public void end(boolean interrupted) {
       
       Robot.leftEncoder.setPosition(0);
       Robot.rightEncoder.setPosition(0);
       
   }

    @Override
    public boolean isFinished() {

        return isDone;
          
    }    
}
