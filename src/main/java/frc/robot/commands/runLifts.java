package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class runLifts extends CommandBase {
    public runLifts(){
        addRequirements(Robot.lifts);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Robot.lifts.teleLifts();
    }

    @Override
    public boolean isFinished() {
        return false;        
    }
}
