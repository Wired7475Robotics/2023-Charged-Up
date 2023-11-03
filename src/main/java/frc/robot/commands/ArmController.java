package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lift;

public class ArmController extends CommandBase{
    public ArmController(){
        addRequirements(Robot.arm);
    }
    
    @Override 
    public void initialize(){

    }

    @Override
    public void execute(){
        Robot.arm.TeleArm();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
