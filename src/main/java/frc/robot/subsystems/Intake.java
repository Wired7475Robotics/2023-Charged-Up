package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controll;
import frc.robot.commands.wiredAPI.Motor;

public class Intake extends SubsystemBase {
    private static Motor ballLoad;
    private static Motor intake;
    public Intake() {
        ballLoad = new Motor("BallLoader");
        intake = new Motor("IntakeMotor");
    }

    public void teleIntake() {
        if (Controll.getOpB()) {
            intake.run(-0.5);
        } else {
            intake.run(0);
        }
        if (Controll.getOpX()) {
            ballLoad.run(1);
        } else if (Controll.getOpY()){
            ballLoad.run(-1);
        } else {
            ballLoad.run(0);
        }
    }

    public void autoIntake(boolean onState) {
        if(onState) {
            ballLoad.run(1.0);
            intake.run(-0.5);
        } else {
            ballLoad.run(0.0);
            intake.run(0.0);
        }
    }
}
