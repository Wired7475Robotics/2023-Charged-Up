package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;

public class Pneumatics extends SubsystemBase{
    //declare compressor values
    Compressor compressor;
    boolean enabled;
    boolean pSwitch;
    double current;
    Solenoid Lifts;
    public Pneumatics() {
        compressor = new Compressor(1, PneumaticsModuleType.REVPH);
        enabled = compressor.enabled();
        pSwitch = compressor.getPressureSwitchValue();
        current = compressor.getPressure();
        Lifts = new Solenoid(PneumaticsModuleType.REVPH , 14 );
    }

    public void teleLift() {
        Lifts.set(Controll.getOpTrigger(Controll.LEFT));
    }
}
