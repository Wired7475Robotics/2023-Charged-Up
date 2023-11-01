package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class GyroReset extends CommandBase{
    public GyroReset(){
        addRequirements(Robot.drivetrain);
    }
    @Override
    public void execute() {
        
    }
}
