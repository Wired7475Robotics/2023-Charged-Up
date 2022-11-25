package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoShooter  extends CommandBase{

    private double runTime;
    private double startTime;
    public AutoShooter() {
        addRequirements(Robot.shooter);
    }

    public AutoShooter(double timeOn) {
        addRequirements(Robot.shooter);
        runTime = timeOn;
        startTime = Robot.timer.get();
        System.out.println("Initalization Complete!");
    }

    @Override
    public void execute() {
        Robot.shooter.autoShoot(true);
        System.out.println("Command Running!");
    }

   @Override
   public void end(boolean interrupted) {
       Robot.shooter.autoShoot(false);

       if(interrupted)
            System.out.println("Interrupt!");

        System.out.println("Command Over!");
   }

    @Override
    public boolean isFinished() {
        return startTime + runTime < Robot.timer.get();   
    }    
}
