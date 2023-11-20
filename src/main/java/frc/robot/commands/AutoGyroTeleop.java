package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

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
        pid.setTolerance(9);
        pid.setSetpoint(0);
        //Robot.navX.reset();
        Robot.drivetrain.leftDrive1.setIdleMode(IdleMode.kBrake);
        Robot.drivetrain.leftDrive2.setIdleMode(IdleMode.kBrake);
        Robot.drivetrain.rightDrive1.setIdleMode(IdleMode.kBrake);
        Robot.drivetrain.rightDrive2.setIdleMode(IdleMode.kBrake);
        
    }
    @Override
    public void execute() {
        
        pidPosCmd = pid.calculate(Robot.navX.getRoll() + 1.5);
        clampedPidPosCmd = Math.max(-0.35, Math.min(0.35, pidPosCmd /5));
        Robot.drivetrain.drivetrain.arcadeDrive(clampedPidPosCmd,0);
        System.out.println(pidPosCmd +","+ clampedPidPosCmd);

    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.drivetrain.drivetrain.arcadeDrive(0, 0);
        Robot.drivetrain.leftDrive1.setIdleMode(IdleMode.kCoast);
        Robot.drivetrain.leftDrive2.setIdleMode(IdleMode.kCoast);
        Robot.drivetrain.rightDrive1.setIdleMode(IdleMode.kCoast);
        Robot.drivetrain.rightDrive2.setIdleMode(IdleMode.kCoast);
    }

}
