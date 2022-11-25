package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;
import frc.robot.commands.wiredAPI.Motor;

public class Lifts extends SubsystemBase {
    
    public static Motor leftPulleyMotor1;
    public static Motor leftPulleyMotor2;
    public static Motor rightPulleyMotor1;
    public static Motor rightPulleyMotor2;



    public Lifts(){
        leftPulleyMotor1 = new Motor("LeftPulleyMotor1");
        leftPulleyMotor2 = new Motor("LeftPulleyMotor2");
        rightPulleyMotor1 = new Motor("RightPulleyMotor1");
        rightPulleyMotor2 = new Motor("RightPulleyMotor2");

    }

    public void teleLifts() {
        leftPulleyMotor2.run(Controll.getOpBumper(Controll.RIGHT) ? Controll.getOpRightStick(Controll.Y) : Controll.getOpLeftStick(Controll.Y));
        rightPulleyMotor1.run(Controll.getOpBumper(Controll.RIGHT) ? Controll.getOpLeftStick(Controll.Y) : Controll.getOpLeftStick(Controll.Y));
        rightPulleyMotor2.run(Controll.getOpBumper(Controll.RIGHT) ? Controll.getOpLeftStick(Controll.Y) : Controll.getOpLeftStick(Controll.Y));
        leftPulleyMotor1.run(Controll.getOpBumper(Controll.RIGHT) ? -Controll.getOpRightStick(Controll.Y) : -Controll.getOpLeftStick(Controll.Y));
    }

}
