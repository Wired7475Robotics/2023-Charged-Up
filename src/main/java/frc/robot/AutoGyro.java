package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
        pid.setTolerance(0.5);
        pid.setSetpoint(0);
        Robot.navX.reset();
    }
    @Override
    public void execute() {
        
        pidPosCmd = pid.calculate(Robot.navX.getPitch());
        clampedPidPosCmd = Math.max(-0.5, Math.min(0.5, pidPosCmd));
        Robot.drivetrain.drivetrain.arcadeDrive(clampedPidPosCmd,0);
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
