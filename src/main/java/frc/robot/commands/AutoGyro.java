package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutoGyro extends CommandBase{
    double kp = 0.1;
    double ki = 0;
    double kd = 0;

    double pidPosCmd;
    double clampedPidPosCmd;

    PIDController pid = new PIDController(kp,ki,kd);
    public AutoGyro(){
        addRequirements(Robot.drivetrain);
    }
    @Override
    public void initialize() {
        pid.setTolerance(0.2);
        pid.setSetpoint(0);
        Robot.navX.reset();
    }
    @Override
    public void execute() {
        
        pidPosCmd = pid.calculate(Robot.navX.getRoll());
        clampedPidPosCmd = Math.max(-0.1, Math.min(0.1, pidPosCmd * 10));
        Robot.drivetrain.drivetrain.arcadeDrive(-clampedPidPosCmd,0);
        System.out.println(pidPosCmd +","+ clampedPidPosCmd);
    }
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.drivetrain.arcadeDrive(0, 0);
    }

}
