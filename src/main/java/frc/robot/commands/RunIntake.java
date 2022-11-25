package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunIntake extends CommandBase{

    public RunIntake() {
        addRequirements(Robot.intake);
            
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Robot.intake.teleIntake();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
