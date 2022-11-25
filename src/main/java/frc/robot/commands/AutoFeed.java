package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoFeed  extends CommandBase{

    private double runTime;
    private double startTime;
    public AutoFeed() {
        addRequirements(Robot.intake);
    }

    public AutoFeed(double timeOn) {
        addRequirements(Robot.intake);
        runTime = timeOn;
        startTime = Robot.timer.get();
        System.out.println("Initalization Complete!");
    }

    @Override
    public void execute() {
        Robot.intake.autoIntake(true);
        System.out.println("Command Running!");
    }

   @Override
   public void end(boolean interrupted) {
       Robot.intake.autoIntake(false);

       if(interrupted)
            System.out.println("Interrupt!");

        System.out.println("Command Over!");
   }

    @Override
    public boolean isFinished() {
        return startTime + runTime < Robot.timer.get();   
    }    
}
