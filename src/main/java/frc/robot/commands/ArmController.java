package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class ArmController extends CommandBase{
    public ArmController(){
        addRequirements(Robot.lift);
    }
    
    @Override 
    public void initialize(){

    }

    @Override
    public void execute(){
        Robot.lift.TeleArm();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
