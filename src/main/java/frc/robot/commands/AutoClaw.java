package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoClaw extends CommandBase {
    double target;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    boolean targetInit = false;


    PIDController armPID = new PIDController(kP,kI,kD);

   public AutoClaw(){
        addRequirements(Robot.claw);

   } 

   public AutoClaw(double target_){
        addRequirements(Robot.lift);
        target = target_;
   }

   @Override
   public void initialize() {
        armPID.setTolerance(1);

   }

   @Override
   public void execute() {
       if(targetInit == false){
            armPID.setSetpoint(target);
            targetInit = true;
       }

       Robot.claw.claw.set(armPID.calculate(Robot.claw.claw.getEncoder().getPosition()) <= 0.5? 0.5 : armPID.calculate(Robot.claw.claw.getEncoder().getPosition()) >= -0.5? -0.5 : armPID.calculate(Robot.claw.claw.getEncoder().getPosition()));
       
    }

   @Override
   public boolean isFinished() {
        return armPID.atSetpoint();
   }

   @Override
   public void end(boolean interrupted) {
     Robot.claw.claw.set(0);
   }
    
}
