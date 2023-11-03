package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;

public class Arm extends SubsystemBase{
    public CANSparkMax Extender;
    public DigitalInput topLimitSwitch;

    public Arm(){
        Extender = new CANSparkMax(6, MotorType.kBrushless);
        Extender.setIdleMode(IdleMode.kBrake);
        Extender.setOpenLoopRampRate(0.25);
       
    }
    
    public void TeleArm(){
        //System.out.println("arm code reached");
        if(Controll.getOpBumper(Controll.LEFT)){
            Extender.set(0.75);
        } else if(Controll.getOpBumper(Controll.RIGHT)){
            Extender.set(-0.75);
        } else {
            Extender.set(0);
        }
    }
}
