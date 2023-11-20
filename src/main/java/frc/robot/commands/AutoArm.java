package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoArm extends CommandBase {
    double target = 0;
    double kP = 0.5;
    double kI = 0;
    double kD = 0;
    boolean targetInit = false;

    double position;
    double pidPosCmd;
    double clampedPidPosCmd;

    private final double armConvFactor = (1.0/4.0) * Math.PI * 2;

    PIDController armPID = new PIDController(kP,kI,kD);
    public AutoArm(){
          addRequirements(Robot.arm);
    }

   public AutoArm(double target_){
        addRequirements(Robot.arm);
        target = target_ / armConvFactor;
        System.out.println(target + ","+ target_ + ","+ armConvFactor);
   }

   @Override
   public void initialize() {
        armPID.setTolerance(0.5);
   }


   @Override
   public void execute() {
       if(targetInit == false){
            armPID.setSetpoint(target);
            targetInit = true;
            Robot.arm.Extender.getEncoder().setPosition(0);
       }

       position = Robot.arm.Extender.getEncoder().getPosition();
       pidPosCmd = armPID.calculate(position);
       clampedPidPosCmd = Math.max(-0.5, Math.min(0.5, pidPosCmd));
       Robot.arm.Extender.set(clampedPidPosCmd);

       System.out.println(position + "," + armPID.getSetpoint() + ","+ clampedPidPosCmd + "," +  pidPosCmd);

    }

   @Override
   public boolean isFinished() {
        return armPID.atSetpoint();
   }

   @Override
   public void end(boolean interrupted) {
     Robot.arm.Extender.set(0);
     targetInit = false;
   }

}
