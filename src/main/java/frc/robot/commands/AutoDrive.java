package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoDrive  extends CommandBase{
    
    private double target = 0;
    private boolean isDone = false;
    private double kP = 0.5;
    private double kI = 0;
    private double kD = 0;
    PIDController pid = new PIDController(kP, kI, kD);
    

    public AutoDrive() {
        addRequirements(Robot.drivetrain);
    }

    public AutoDrive(double target_) {
        addRequirements(Robot.drivetrain);
        target = target_;
        System.out.println("Initalization Complete!");
    }

    @Override
    public void initialize() {
        Robot.leftEncoder.reset();
        Robot.rightEncoder.reset();
        pid.setTolerance(0.5, 0.5);
        System.out.println("Command Initalized!");
    }

    @Override
    public void execute() {
        isDone = Robot.drivetrain.autoDrive(target, pid);
        System.out.println("Command Running!");
    }

   @Override
   public void end(boolean interrupted) {
       Robot.leftEncoder.reset();
       Robot.rightEncoder.reset();

       if(interrupted)
            System.out.println("Interrupt!");

        System.out.println("Command Over!");
   }

    @Override
    public boolean isFinished() {
        return isDone;   
    }    
}
