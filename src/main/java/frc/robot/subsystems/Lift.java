package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;

public class Lift extends SubsystemBase{
    public CANSparkMax Extender;
    public CANSparkMax Elevator1;
    public CANSparkMax Elevator2;
    public DigitalInput topLimitSwitch;

    public Lift(){
        Extender = new CANSparkMax(6, MotorType.kBrushless);
        Extender.setIdleMode(IdleMode.kBrake);
        Extender.setOpenLoopRampRate(2);
       
    }
    
    public void TeleArm(){
       // System.out.println("arm code reached");
        if(Controll.getOpBumper(Controll.LEFT)){
            Extender.set(0.50);
        } else if(Controll.getOpBumper(Controll.RIGHT)){
            Extender.set(-0.50);
        } else {
            Extender.set(0);
        }
    }
}
