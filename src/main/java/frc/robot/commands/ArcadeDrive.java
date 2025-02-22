package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ArcadeDrive extends CommandBase {
    
    public ArcadeDrive() {
        addRequirements(Robot.drivetrain);
    }
      // Called just before this Command runs the first time
      @Override
      public void initialize() {
      }
    
      // Called repeatedly when this Command is scheduled to run
      @Override
      public void execute() {
        Robot.drivetrain.teleDrive();
        //Robot.drivetrain.targetCube();
        //Robot.drivetrain.targetCone();
      }
    
      // Make this return true when this Command no longer needs to run execute()
      @Override
      public boolean isFinished() {
        return false;
      }
    
}
