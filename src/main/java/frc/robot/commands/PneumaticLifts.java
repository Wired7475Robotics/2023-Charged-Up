package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class PneumaticLifts extends CommandBase{
    
    public PneumaticLifts() {
        addRequirements(Robot.PnuLifts);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Robot.PnuLifts.teleLift();
    }

    @Override
    public boolean isFinished() {
        return false;        
    }
}
