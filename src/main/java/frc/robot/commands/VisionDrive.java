package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controll;
import frc.robot.Robot;

public class VisionDrive extends CommandBase{
    private double MAX_FOV = 35;
    private double TAR_AREA = 40;
    private double TAR_YAW = 0;

    boolean isDone = false;

    PIDController anglePID = new PIDController(0.35, 0, 0.5);

    double maxDistError = 0;
    double maxAngError = 0;

    public VisionDrive(){
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void initialize(){

        anglePID.setSetpoint(TAR_YAW);
        anglePID.setTolerance(2);
        maxAngError = MAX_FOV - Math.abs(Robot.drivetrain.findCube().getYaw());
    }

    @Override
    public void execute(){
        isDone = Robot.drivetrain.targetCube(anglePID, MAX_FOV, maxAngError);
        System.out.println(isDone);
    }
    @Override
    public boolean isFinished(){
        return isDone;
    }
    
}
