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
        addRequirements(Robot.arm);
    }
    public AutoLift(double target_){
        addRequirements(Robot.arm);
        
        target = target_ / liftConvFactor;
    }


    
    @Override 
    public void initialize(){

    }
    @Override
    public void execute(){

        if(!targetInit) {
            Robot.arm.Elevator1.getEncoder().setPosition(0);
            Robot.arm.Elevator2.getEncoder().setPosition(0);
            liftPid.setSetpoint(target);
            targetInit = true;
        }

        liftPid.setTolerance(3);
        armPid.setTolerance(3);


        double input = (-Robot.arm.Elevator1.getEncoder().getPosition() + Robot.arm.Elevator2.getEncoder().getPosition()) / 2;
        System.out.println("Input :" + input + " output :" + liftPid.calculate(input)/10 + " setpoint: " + liftPid.getSetpoint() + " limitSwitch: " + !Robot.arm.topLimitSwitch.get());
        if (liftPid.calculate(input)/10 > 0.20 && Robot.arm.topLimitSwitch.get()){

            Robot.arm.Elevator1.set(-0.20);
            Robot.arm.Elevator2.set(0.20);
        } else if ((liftPid.calculate(input)/10 < 0.20) && liftPid.calculate(input)/10 > -0.20 && Robot.arm.topLimitSwitch.get() ){
            Robot.arm.Elevator1.set(-liftPid.calculate(input)/10);

            Robot.arm.Elevator2.set(liftPid.calculate(input)/10);
        } else if (liftPid.calculate(input)/10 < -0.20 && Robot.arm.topLimitSwitch.get()){
            Robot.arm.Elevator1.set(0.20);
            Robot.arm.Elevator2.set(-0.20);
        } else {
            Robot.arm.Elevator1.set(0);
            Robot.arm.Elevator2.set(0);
        }
    }
    @Override
    public boolean isFinished(){

        return liftPid.atSetpoint();
    }
    @Override
    public void end(boolean interrupted) {
        Robot.arm.Elevator1.set(0);
        Robot.arm.Elevator2.set(0);
    }
}
