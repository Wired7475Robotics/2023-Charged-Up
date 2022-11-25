package frc.robot.commands.wiredAPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;             
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;
import java.util.concurrent.TimeUnit;


/**
 * <h1>The class For running and loading motors.</h1>
 * 
 * declare variable by initiating like this:
 * {@code motor Motor = new motor(motorName, filePath);}
 * 
 * use {@code motor.run(speed) ;} to move motor
 * use {@code motor.stop()} to stop motor
 * <h2>Notes</h2>
 * <ul>
 *  <li>The motor name is the name of the motor in the config file.</li>
 *  <li>The file path is the relative path to the folder CONTAINING config file.</li>
 *  <li>The speed is a double value. ex.{@code motor.run(0.5);} will run the motor at %50 power.</li>
 *  <li>!!DO NOT SET THE SPEED ABOVE 1.0 (or below - 1.0) UNLESS YOU ARE ABSOLUTLY SURE OF WHAT YOU ARE DOING!!</li>
 * </ul>
 */
public class Motor {
    // Declare motor variables
    private String motorType;
    private boolean loaded;
    TalonSRX talonMotor;
    VictorSPX victorMotor;
    TalonFX falconMotor;
    CANSparkMax sparkMaxMotor;
    Spark sparkMotor;
    VictorSP victorSPMotor;
    private static String filePath = "";

    /**
     * <h1>The constructor for motor class.</h1>
     * 
     * @param motorName The name of the motor
     * The path to the config folder
     */
    public Motor (String motorName) {
        if (filePath.equals("")) {
            System.out.println("Error: filePath not set");
        }else{
            loadMotor(motorName);
        }
    }
    /**
     * <h1>Method to set the filePath variable used when loading motors</h1>
     * @param path The relative path to the folder CONTAINING config file.
     */
    public static void setMotorConfigPath(String path) {
        filePath = path;
    }
    /**
    * Returns a Talon SRX motor object with the given properties
    * @param motorProp The motor properties file to read
    * @return Returns the new motor object
    * @
    */
    private TalonSRX loadTalon(Properties motorProp) {
        loaded = true;
        return new TalonSRX(Integer.parseInt(motorProp.getProperty("motorPort")));

    }
   /**
    * Returns a Falcon FX motor object with the given properties
    * @param motorProp The motor properties file to read
    * @return Returns the new motor object
    */
    private TalonFX loadFalcon(Properties motorProp) {
        loaded = true;
        return new TalonFX(Integer.parseInt(motorProp.getProperty("motorPort")));
    }
    /**
    * Returns a Victor (Falcon FX) motor object with the given properties
    * @param motorProp The motor properties file to read
    * @return Returns the new motor object
    */
    private VictorSPX loadVictor(Properties motorProp) {
        loaded = true;
        return new VictorSPX(Integer.parseInt(motorProp.getProperty("motorPort")));
    }
    private CANSparkMax loadSparkMax(Properties motorProp) {
        loaded = true;
        CANSparkMax CPM;
        if (Boolean.valueOf( motorProp.getProperty("brushed"))){
            CPM = new CANSparkMax( Integer.parseInt(motorProp.getProperty("motorPort")),MotorType.kBrushed);
        } else {
            CPM = new CANSparkMax( Integer.parseInt(motorProp.getProperty("motorPort")),MotorType.kBrushless);
        }
        CPM.restoreFactoryDefaults();
        return CPM;
    }
    private Spark loadSpark(Properties motorProp){
        loaded = true;
        return new Spark(Integer.parseInt(motorProp.getProperty("motorPort")));
    }
    private VictorSP loadVictorSP(Properties motorProp){
        loaded = true;
        return new VictorSP(Integer.parseInt(motorProp.getProperty("motorPort")));
    }
    
    /**
     * Loads and sets the correct CAN controller type
     * @param motorName The name of the motor
     */
    private void loadMotor(String motorName) {
        Properties motorProp = new Properties();
        String motorFile = getMotorFilename(motorName);
        FileReader motorFiles;
        try {
            motorFiles = new FileReader(motorFile);
            motorProp.load(motorFiles);
        } catch (IOException e) {
            e.printStackTrace();
        }

        motorType = motorProp.getProperty("motorType");
        if(motorType.equals("TalonSRX")){
            talonMotor = loadTalon(motorProp);
        } else if(motorType.equals("VictorSPX")){
            victorMotor = loadVictor(motorProp);
        } else if(motorType.equals("Falcon")){
            falconMotor = loadFalcon(motorProp);
        } else if(motorType.equals("TalonFX")){
            falconMotor = loadFalcon(motorProp);
        } else if(motorType.equals("CANSparkMax")) {
            sparkMaxMotor = loadSparkMax(motorProp);
        } else if(motorType.equals("Spark")){
            sparkMotor = loadSpark(motorProp);
        } else if(motorType.equals("VictorSP")){ 
            loadVictorSP(motorProp);
        }else {
            System.out.println("Motor type not found");
        }
    }
    /**
     * Returns the filename of the motor properties file
     */
    private String getMotorFilename(String MotorName) throws NullPointerException{
        Properties motorProp = new Properties();
        int MCFileNum = new File(filePath).listFiles().length;
        File[] MCfile = new File(filePath).listFiles();
        String filename = "";
        
        for(int i = 0; i < MCFileNum; i++){
            FileInputStream motorFiles;
            try {
                motorFiles = new FileInputStream(new File (MCfile[i].getPath()));
                motorProp.load(motorFiles);
            } catch (IOException e) {
                e.printStackTrace();
            }

            if(motorProp.getProperty("motorName").equals(MotorName)){
                filename = MCfile[i].getPath();
                break;
            }
        }
        return filename;
    }
    /**
     * Runs motor at the given speed
     * @param speed The speed to run the motor at
     * <h2>NOTES:</h2>
     * <ul>
     * <li>The speed is a double value. ex.{@code motor.run(0.5);} will run the motor at %50 power.</li>
     * <li>!!DO NOT SET THE SPEED ABOVE 1.0 (or below - 1.0) UNLESS YOU ARE ABSOLUTLY SURE OF WHAT YOU ARE DOING!!</li>
     </ul>
     */
    public void run(double speed){
        if (loaded == true){
            if(motorType.equals("TalonSRX")){
                talonMotor.set(ControlMode.PercentOutput, speed);
            } else if(motorType.equals("VictorSPX")){
                victorMotor.set(ControlMode.PercentOutput, speed);
            } else if(motorType.equals("Falcon")){
                falconMotor.set(ControlMode.PercentOutput, speed);
            } else if(motorType.equals("CANSparkMax")){
                sparkMaxMotor.set(speed);
            } else if(motorType.equals("Spark")){
                sparkMotor.set(speed);
            } else if(motorType.equals("VictorSP")){
                victorSPMotor.set(speed);
            } else if (motorType.equals("TalonFX")){
                falconMotor.set(ControlMode.PercentOutput, speed);
            }

        } else {
            System.out.println("Error: Motor not loaded");
        }
    }
    /**
    *Runs all motors given at the same speed
    *can be given an infinite number of motors
    *Use like this:
    *{@code Motor.runsame(0.5, motor2, motor3)}
    */
    public static void runSame(double speed, Motor ... motors)
    {
        int length = motors.length;
        for(int i = 0; i < length; i++)
        {
            motors[i].run(speed);
        }
    }

    public VictorSP getVictorSP(){
            return victorSPMotor;
    }

    public Spark getSpark(){
            return sparkMotor;
    }

    public CANSparkMax getSparkMax(){
            return sparkMaxMotor;
    }

    public TalonSRX getTalon(){
            return talonMotor;
    }

    public VictorSPX getVictor(){
            return victorMotor;
    }


    /**
    *Runs motor(s) at oposite speed to an/group of other motor(s)
    *Can be given either an infinte number of motors, or two lists of motors:
    *{@code motor.runOpposite(0.5, motor1, motor2)}
    *or {@code motorList1[].runOposite(0.5, motorList2[])}
    */
    public void runOpposite(double speed, Motor ... motors)
    {
        this.run(speed);
        int length = motors.length;
        for(int i = 0; i < length; i++)
        {
            motors[i].run(-speed);
        }
    }

    public static void runOpposite(double speed, Motor matchMotors[], Motor oppositeMotors[])
    {
        int length = matchMotors.length;
        for(int i = 0; i < length; i++)
        {
            matchMotors[i].run(speed);
        }
    
        length = matchMotors.length;
        for(int i = 0; i < length; i++)
        {
            oppositeMotors[i].run(speed);
        }
    }

    /**
    * Runs motor for the given length of time at the set speed
    *@param time the time (in seconds) that the motor will be run for
    *@param speed The speed to run the motor at.
    */

    public void runForTime(double speed,int time){
            run(speed);
            try {
                TimeUnit.SECONDS.sleep(time);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            run(0);
    }


    /**
     * Stops motor 
     */
    public void stopMotor(){
        run(0);
    }
    /**
     * Stops all motors
     */
    public void stopAllMotors(){
        try{
            File[] MCfile = new File(filePath).listFiles();
            int MCFileNum = MCfile.length;
            int i = 0;
            for (;i <= MCFileNum; i++);{
                FileInputStream motorFiles = new FileInputStream(new File(MCfile[i].getPath()));
                Properties motorProp = new Properties();
                motorProp.load(motorFiles);
                String motorType = motorProp.getProperty("motorType");
                String motorPort = motorProp.getProperty("motorPort");
                if(motorType.equals("TalonSRX")){
                    TalonSRX motor = new TalonSRX(Integer.valueOf(motorPort));
                    motor.set(ControlMode.PercentOutput, 0);
                } else if(motorType.equals("VictorSPX")){
                    VictorSPX motor = new VictorSPX(Integer.valueOf(motorPort));
                    motor.set(ControlMode.PercentOutput, 0);
                } else if(motorType.equals("Falcon")){
                    TalonFX motor = new TalonFX(Integer.valueOf(motorPort));
                    motor.set(ControlMode.PercentOutput, 0);
                } else if(motorType.equals("TalonFX")){
                    TalonFX motor = new TalonFX(Integer.valueOf(motorPort));
                    motor.set(ControlMode.PercentOutput, 0);
                }
                else{
                    System.out.println("Motor type not found");
                }
                }
            } catch(IOException e){
            e.printStackTrace();
    }
    }
}

