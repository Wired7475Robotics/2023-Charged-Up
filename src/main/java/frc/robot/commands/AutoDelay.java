package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoDelay  extends CommandBase{

    private double runTime;
    private double startTime;
    
    public AutoDelay() {
    }

    public AutoDelay(double timeOn) {
        addRequirements(Robot.intake);
        runTime = timeOn;
        startTime = Robot.timer.get();
        System.out.println("Initalization Complete!");
    }

    @Override
    public void execute() {
        System.out.println("Command Running!");
    }

    @Override
    public boolean isFinished() {
        return startTime + runTime < Robot.timer.get();   
    }    
}
