package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class Intake {
    public static TalonFX leftLinkageMotor = new TalonFX(Constants.leftLinkageMotorID);
    public static TalonFX rightLinkageMotor = new TalonFX(Constants.rightLinkageMotorID);
    static TalonFX[] motorList = new TalonFX[] {leftLinkageMotor, rightLinkageMotor};

    public static TalonFX intakeRollerMotor = new TalonFX(Constants.intakeRollerMotorID);

    public static DigitalInput positioningSensorL = new DigitalInput(Constants.intakePositionSensorLPort);
    public static DigitalInput positioningSensorR = new DigitalInput(Constants.intakePositionSensorRPort);
    static DigitalInput[] sensorList = new DigitalInput[] {positioningSensorL, positioningSensorR};
    
    // method to apply current limit for the linkage motors
    public static final double maxCurrent = 1.5;// set the current limit to 2 and the stator limit to double that
    public static void configureCurrentLimit(){
        for (TalonFX item: motorList){// do the same thing for the 2 motors
            TalonFXConfigurator configurator = item.getConfigurator();
            CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

            limitConfigs.SupplyCurrentLimit = maxCurrent;
            limitConfigs.SupplyCurrentLimitEnable = true;
            //limitConfigs.StatorCurrentLimit = maxCurrent*2;
            limitConfigs.StatorCurrentLimitEnable = false;

            configurator.apply(limitConfigs);
        }
    }

    // method to reset the encoders for the powered bar pushers
    public static Boolean needReset = true, // variable that needs to be true to START resetting the encoders
        resetL = false, resetR = false;// variables for if the left and right sides have been reset
    public static Double power = 0.1;
    public static Boolean resetLinkageEncoders(){
        if (needReset){
            resetL = false;
            resetR = false;
        }
        int[] nums = new int[] {0, 1};
        for (int i : nums){ // do the same thing for the 2 motors & sensors
            TalonFX curMotor = motorList[i];
            DigitalInput curSensor = sensorList[i];

            // skip the motor if it is already reset
            if (((i == 0) && (resetL == true)) || ((i == 1) && (resetR == true))){
                continue;
            }

            // set the motor to a power if it isn't already
            
            if (needReset){
                if (curSensor.get() == true){ // positive power if the sensor isn't triggered
                    curMotor.set(-power);
                }else{
                    curMotor.set(power);
                }
            }

            // check if the sensor value has changed
            if (curSensor.get() && curMotor.get() > 0){
                // reset encoder and set speed to 0
                curMotor.set(0);
                curMotor.setPosition(0.0);
                if (i == 0) resetL = true;
                else resetR = true;
            }else if ((!curSensor.get()) && curMotor.get() < 0){// if the sensor was triggered approaching from the extended direction, make it approach from the other direction
                curMotor.set(power);
            }
        }
        needReset = false;
        return (resetL && resetR);// return true if and only if both motors are reset
    }

    // method for moving the intake - extending, retracting, going to neutral
    public static final Double maxSpeed = 0.1, // exactly what it looks like; max speed for moving the intake around
        deadZone = 0.5; // how far away in motor rotations it is from the correct position when it stops ~= 360*deadzone/(gear ratio n:1) degrees, currently about 2.25 each direction
    public static void moveIntakeTo(String position){

        Double posNum; // convert the position from a string to a numerical value
        switch(position){
            case "in":case "In":
            case "inside":case "Inside":
            case "retract":case "Retract":
            case "retracted":case "Retracted":
                posNum = 7.0;
                break;
            case "out":case "Out":
            case "outside":case "Outside":
            case "extend":case "Extend":
            case "extended":case "Extended":
                posNum = 28.0;
                break;
            case "neutral":case "Neutral":
            default:
                posNum = 14.0;
                break;
        }

        int[] nums = new int[] {0, 1};
        for (int i : nums){ // do the same thing for the 2 motors
            TalonFX curMotor = motorList[i];
            Double error = posNum-curMotor.getRotorPosition().getValueAsDouble();
            
            if (Math.abs(error) < deadZone){ // if in the deadzone, stop
                curMotor.set(0.0);
            }else{ // if not in the deadzone, power is proportional to error
                curMotor.set(Math.min(maxSpeed, Math.max(-maxSpeed, error/10)));
            }
        }

    }
}
