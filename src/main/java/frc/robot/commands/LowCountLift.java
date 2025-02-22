package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controll;
import frc.robot.Robot;

public class LowCountLift extends CommandBase{
    public static final int LOW = 0;
    public static final int MED = 1;
    public static final int HIGH = 2;

    private int mode = 0;

    private double lowCount = 5;
    private double medCount = 20;
    private double highCount = 45;

    private double lowArmCount = 0;
    private double medArmCount = 24;
    private double highArmCount = 20;

    private final double liftConvFactor = -(1/3.4) * Math.PI * 1.5;

    private final double armConvFactor = -(1/4) * Math.PI * 2;


    PIDController liftPid = new PIDController(1, 0, 0);
    PIDController armPid = new PIDController(0.5, 0, 0);

    public LowCountLift(){
        addRequirements(Robot.lift);
    }


    
    @Override 
    public void initialize(){


        //liftPid.setSetpoint(2);
        liftPid.setSetpoint(lowCount / liftConvFactor);
    }
    @Override
    public void execute(){
        /*
        if (Controll.getOpA()){
            mode = LOW;
        } else if ( Controll.getOpB()){
            mode = MED;
        } else if (Controll.getOpY()){
            mode = HIGH;
        } else {
            mode = 3;
        }

        switch(mode){
            case LOW: {
                liftPid.setSetpoint(lowCount / liftConvFactor);
                armPid.setSetpoint(lowArmCount / armConvFactor);
            };
            case MED: {
                liftPid.setSetpoint(medCount / liftConvFactor);
                armPid.setSetpoint(medArmCount / armConvFactor);
            };
            case HIGH: {
                liftPid.setSetpoint(highCount / liftConvFactor);
                armPid.setSetpoint(highArmCount / armConvFactor);
            };
            default: {
                liftPid.setSetpoint(Robot.arm.Elevator1.getEncoder().getPosition());
            };

        }
        */
        
        liftPid.setTolerance(3);
        armPid.setTolerance(3);


        double input = (-Robot.lift.Elevator1.getEncoder().getPosition() + Robot.lift.Elevator2.getEncoder().getPosition()) / 2;
        System.out.println("Input :" + input + " output :" + liftPid.calculate(input)/10 + " setpoint: " + liftPid.getSetpoint() + " limitSwitch: " + Robot.lift.topLimitSwitch.get());
        if (liftPid.calculate(input)/10 > 0.10 && Robot.lift.topLimitSwitch.get()){

            Robot.lift.Elevator1.set(-0.10);
            Robot.lift.Elevator2.set(0.10);
        } else if ((liftPid.calculate(input)/10 < 0.20) && liftPid.calculate(input)/10 > -0.20 && Robot.lift.topLimitSwitch.get() ){
            Robot.lift.Elevator1.set(-liftPid.calculate(input)/10);

            Robot.lift.Elevator2.set(liftPid.calculate(input)/10);
        } else if (liftPid.calculate(input)/10 < -0.20 && Robot.lift.topLimitSwitch.get()){
            Robot.lift.Elevator1.set(0.20);

            Robot.lift.Elevator2.set(-0.20);
        } else {
            Robot.lift.Elevator1.set(0);
            Robot.lift.Elevator2.set(0);
        }
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
