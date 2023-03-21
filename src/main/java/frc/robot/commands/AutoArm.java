package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoArm extends CommandBase {
    double target;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    boolean targetInit = false;

    private final double armConvFactor = -(1/4) * Math.PI * 2;

    PIDController armPID = new PIDController(kP,kI,kD);

   public AutoArm(){
        addRequirements(Robot.lift);

   } 

   public AutoArm(double target_){
        addRequirements(Robot.lift);
        target = target_ * armConvFactor;
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

       Robot.lift.Extender.set(armPID.calculate(Robot.lift.Extender.getEncoder().getPosition()) <= 0.5? 0.5 : armPID.calculate(Robot.lift.Extender.getEncoder().getPosition()) >= -0.5? -0.5 : armPID.calculate(Robot.lift.Extender.getEncoder().getPosition()));
    
    }

   @Override
   public boolean isFinished() {
        return armPID.atSetpoint();
   }
    
}
