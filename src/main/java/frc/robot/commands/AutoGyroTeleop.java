package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoGyroTeleop extends CommandBase{
    double kp = 0.25;
    double ki = 0;
    double kd = 0.05;

    double pidPosCmd;
    double clampedPidPosCmd;

    PIDController pid = new PIDController(kp,ki,kd);
    public AutoGyroTeleop(){
        addRequirements(Robot.drivetrain);
    }
    @Override
    public void initialize() {
        pid.setTolerance(1);
        pid.setSetpoint(0);
        //Robot.navX.reset();
    }
    @Override
    public void execute() {
        
        pidPosCmd = pid.calculate(Robot.navX.getRoll() + 1.5);
        clampedPidPosCmd = Math.max(-0.6, Math.min(0.6, pidPosCmd /5));
        Robot.drivetrain.drivetrain.arcadeDrive(clampedPidPosCmd,0);
        System.out.println(pidPosCmd +","+ clampedPidPosCmd);

    }

}
