package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class LiftEncoderReset extends CommandBase {
    
    public LiftEncoderReset(){
        addRequirements(Robot.lift);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Robot.lift.Elevator1.getEncoder().setPosition(0);
        Robot.lift.Elevator2.getEncoder().setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
