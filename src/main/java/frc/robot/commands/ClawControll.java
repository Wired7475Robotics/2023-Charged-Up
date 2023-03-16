package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ClawControll extends CommandBase {
    public ClawControll(){
        addRequirements(Robot.claw);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        Robot.claw.teleClaw();
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}   
