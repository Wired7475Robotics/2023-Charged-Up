package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;
import frc.robot.commands.wiredAPI.Motor;

public class Shooter extends SubsystemBase {
    int idleTimer;
    boolean shooterIdleIsActive;
    int idleCooldown;
    private static Motor shooter;
    public Shooter() {
        
        shooter = new Motor("ShooterMotor");
        
    }



    public void teleShooter(){
        
        if(Controll.getOpA())
            shooter.run(1);
        else if(Controll.getOpTrigger(Controll.RIGHT))
            shooter.run(0.1);
        else
            shooter.run(0);

        
        
    }

    public void autoShoot(boolean state) {
        if(state) {
            shooter.run(1.0);
        } else {
            shooter.run(0.0);
        }
    }



}
