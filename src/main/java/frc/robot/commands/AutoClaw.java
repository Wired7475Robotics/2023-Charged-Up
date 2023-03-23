package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class AutoClaw extends CommandBase {
    double targetSpeed;
    double kP = 1;
    double kI = 0;
    double kD = 0;
    boolean isDone = false;
    boolean targetInit = false;


    PIDController armPID = new PIDController(kP,kI,kD);


   public AutoClaw(double target_){
        addRequirements(Robot.claw);
        targetSpeed = -target_;
   }

   @Override
   public void initialize() {
        armPID.setTolerance(0.1);

   }

   @Override
   public void execute() {
          Robot.claw.claw.set(targetSpeed);
          isDone = true;

    }

   @Override
   public boolean isFinished() {
        return isDone;
   }

   @Override
   public void end(boolean interrupted) {
   }

}
