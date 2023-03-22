package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Pause extends CommandBase{

    double pauseTime;
    Timer timer;

    public Pause(double seconds){
        pauseTime = seconds;
    }

    @Override
    public void initialize() {
        timer.reset();
    }
    @Override
    public void execute() {
        timer.start();
    }
    @Override
    public boolean isFinished() {
        return timer.get() == pauseTime;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }
    
}
