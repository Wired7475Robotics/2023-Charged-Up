package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controll;
import frc.robot.Robot;

public class AutoLift extends CommandBase{
    public static final int LOW = 0;
    public static final int MED = 1;
    public static final int HIGH = 2;



    private boolean targetInit = false;

    private final double liftConvFactor = -(1/3.4) * Math.PI * 1.5;



    PIDController liftPid = new PIDController(1, 0, 0);
    PIDController armPid = new PIDController(0.5, 0, 0);
    double target;

    public AutoLift(){
        addRequirements(Robot.lift);
    }
    public AutoLift(double target_){
        addRequirements(Robot.lift);
        
        target = target_ / liftConvFactor;
    }


    
    @Override 
    public void initialize(){

    }
    @Override
    public void execute(){

        if(!targetInit) {
            Robot.lift.Elevator1.getEncoder().setPosition(0);
            Robot.lift.Elevator2.getEncoder().setPosition(0);
            liftPid.setSetpoint(target);
            targetInit = true;
        }

        liftPid.setTolerance(3);
        armPid.setTolerance(3);


        double input = (-Robot.lift.Elevator1.getEncoder().getPosition() + Robot.lift.Elevator2.getEncoder().getPosition()) / 2;
        System.out.println("Input :" + input + " output :" + liftPid.calculate(input)/10 + " setpoint: " + liftPid.getSetpoint() + " limitSwitch: " + !Robot.lift.topLimitSwitch.get());
        if (liftPid.calculate(input)/10 > 0.30 && Robot.lift.topLimitSwitch.get()){

            Robot.lift.Elevator1.set(-0.30);
            Robot.lift.Elevator2.set(0.30);
        } else if ((liftPid.calculate(input)/10 < 0.30) && liftPid.calculate(input)/10 > -0.30 && Robot.lift.topLimitSwitch.get() ){
            Robot.lift.Elevator1.set(-liftPid.calculate(input)/10);

            Robot.lift.Elevator2.set(liftPid.calculate(input)/10);
        } else if (liftPid.calculate(input)/10 < -0.30 && Robot.lift.topLimitSwitch.get()){
            Robot.lift.Elevator1.set(0.30);
            Robot.lift.Elevator2.set(-0.30);
        } else {
            Robot.lift.Elevator1.set(0);
            Robot.lift.Elevator2.set(0);
        }
    }
    @Override
    public boolean isFinished(){

        return liftPid.atSetpoint();
    }
    @Override
    public void end(boolean interrupted) {
        Robot.lift.Elevator1.set(0);
        Robot.lift.Elevator2.set(0);
    }
}
