package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoTurn  extends CommandBase{
    
    private double target = 0;
    private double finalTarget = 0;
    private boolean isDone = false;
    private boolean targetInit = false;
    private double kP = 0.35;
    private double kI = 0.0;
    private double kD = 0.045;
    PIDController pid = new PIDController(kP, kI, kD);


    public AutoTurn() {
        addRequirements(Robot.drivetrain);
    }

    public AutoTurn(double target_) {
        target = target_;
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void initialize() {
        pid.setTolerance(0.1);
        pid.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        if(!targetInit){
            finalTarget = Robot.absAngle(Robot.absAngle() + target);
            pid.setSetpoint(finalTarget);
            targetInit = true;
        }

        isDone = Robot.drivetrain.autoTurn(finalTarget, pid);
    }

   @Override
   public void end(boolean interrupted) {
   }

    @Override
    public boolean isFinished() {
        return isDone;   
    }    
}
