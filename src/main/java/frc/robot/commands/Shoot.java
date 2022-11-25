package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Shoot extends CommandBase {
    public Shoot (){
        addRequirements(Robot.shooter);
    }

    @Override
    public void initialize() {
        
    }
    @Override 
    public void execute() {
        Robot.shooter.teleShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
