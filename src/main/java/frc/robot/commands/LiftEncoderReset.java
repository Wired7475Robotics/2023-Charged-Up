package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class LiftEncoderReset extends CommandBase {
    
    public LiftEncoderReset(){
        addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Robot.arm.Elevator1.getEncoder().setPosition(0);
        Robot.arm.Elevator2.getEncoder().setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
