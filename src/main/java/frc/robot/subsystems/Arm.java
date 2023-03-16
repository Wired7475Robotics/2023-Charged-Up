package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;

public class Arm extends SubsystemBase{
    public CANSparkMax Elevator1;
    public CANSparkMax Elevator2;
    public DigitalInput topLimitSwitch;

    public Arm(){
        Elevator1 = new CANSparkMax(3, MotorType.kBrushless);
        Elevator2 = new CANSparkMax(16, MotorType.kBrushless);
        topLimitSwitch = new DigitalInput(0);

        Elevator1.setIdleMode(IdleMode.kBrake);
    
        Elevator2.setIdleMode(IdleMode.kBrake);
        Elevator1.setOpenLoopRampRate(2);
        Elevator2.setOpenLoopRampRate(2);        
    }
}
