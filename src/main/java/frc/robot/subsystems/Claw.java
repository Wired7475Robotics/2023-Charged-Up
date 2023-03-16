package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;

public class Claw extends SubsystemBase{
    public CANSparkMax claw;
    public Claw(){
        claw = new CANSparkMax(13, MotorType.kBrushless);
        claw.setIdleMode(IdleMode.kBrake);
    }

    public void teleClaw(){
        claw.set(Controll.getOpLeftStick(Controll.X)*0.75);
    }
    
    
}
