package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class Intake {
    public static TalonFX leftLinkageMotor = new TalonFX(Constants.leftLinkageMotorID);
    public static TalonFX rightLinkageMotor = new TalonFX(Constants.rightLinkageMotorID);
    public static TalonFX[] motorList = new TalonFX[] {leftLinkageMotor, rightLinkageMotor};

    public static TalonFX intakeRollerMotor = new TalonFX(Constants.intakeRollerMotorID);

    public static DigitalInput positioningSensorL = new DigitalInput(8);//Constants.intakePositionSensorLPort);
    public static DigitalInput positioningSensorR = new DigitalInput(9);//Constants.intakePositionSensorRPort);
    static DigitalInput[] sensorList = new DigitalInput[] {positioningSensorL, positioningSensorR};

    public static TalonFXConfigurator rightLinkageConfigurator = rightLinkageMotor.getConfigurator();
    public static TalonFXConfigurator leftLinkageConfigurator = leftLinkageMotor.getConfigurator();
    public static TalonFXConfigurator[] configuratorList = new TalonFXConfigurator[] {leftLinkageConfigurator, rightLinkageConfigurator};
    public static TalonFXConfigurator intakeRollerConfigurator = intakeRollerMotor.getConfigurator();
    public static CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

    
    // method to apply current limit for the linkage motors
    public static final double maxCurrent = 3.0;// set the current limit to 2 and the stator limit to double that
    public static void configureCurrentLimit(double[] current){
        int i = 0;
        for (TalonFX item: motorList){// do the same thing for the 2 motors
            limitConfigs.SupplyCurrentLimit = current[i];
            limitConfigs.SupplyCurrentLimitEnable = true;
            //limitConfigs.StatorCurrentLimit = maxCurrent*2;
            limitConfigs.StatorCurrentLimitEnable = false;

            configuratorList[i].apply(limitConfigs);
            ++i;
        }
    }

    public static Boolean usingCurrentLimit = true;

    public static void changeRollerLimit(boolean useLimit){
        if (usingCurrentLimit == useLimit) return;
        usingCurrentLimit = useLimit;

        limitConfigs.SupplyCurrentLimit = 10;
        limitConfigs.SupplyCurrentLimitEnable = useLimit;
        limitConfigs.StatorCurrentLimitEnable = false;

        intakeRollerConfigurator.apply(limitConfigs);
    }

    // method to reset the encoders for the powered bar pushers
    public static Boolean needReset = true; // variable that needs to be true to START resetting the encoders
    public static Boolean[] resetLR = {false, false};// variables for if the left and right sides have been reset
    public static Double farPower = 0.2, smallPower = 0.1;
    public static Double[] linkagePow = {0.0, 0.0};
    public static int[] highCurrentNum = {0, 0};
    public static int highCurrentThreshold = 20;
    public static Double[] resetLinkageEncoders(){
        if (needReset){
            resetLR[0] = false;
            resetLR[1] = false;
            configureCurrentLimit(new double[] {1.5, 1.5});
            highCurrentNum[0] = 0;
            highCurrentNum[1] = 0;
        }
        int[] nums = new int[] {0, 1};
        for (int i : nums){ // do the same thing for the 2 motors & sensors
            // skip the motor if it is already reset
            if (resetLR[i] == true) {
                continue;
            }

            //TalonFX curMotor = motorList[i];
            //DigitalInput curSensor = sensorList[i];

            if (Math.abs(motorList[i].getVelocity().getValueAsDouble()) <5){//(Math.abs(motorList[i].getSupplyCurrent().getValueAsDouble()) > 0.7*(new double[] {3, 3})[i]){
                highCurrentNum[i] = Math.min(highCurrentThreshold, highCurrentNum[i]+1);
            }
                
            if (highCurrentNum[i] >= highCurrentThreshold) {
                linkagePow[i] = 0.0;
                motorList[i].setPosition((23.73+11.35)*1.125);
                resetLR[i] = true;

                limitConfigs.SupplyCurrentLimit = maxCurrent;
                limitConfigs.SupplyCurrentLimitEnable = true;
                limitConfigs.StatorCurrentLimitEnable = false;

                configuratorList[i].apply(limitConfigs);
            }else
                linkagePow[i] = farPower;
        }
        needReset = false;
        return linkagePow;
    }
    
    // *************************
    // method for moving the intake - extending, retracting, going to neutral
    // only returns the calculated power - doesn't actually set motors
    // *************************
    public static final Double maxSpeed = 0.3, // exactly what it looks like; max speed for moving the intake around
        deadZone = 0.5; // how far away in motor rotations it is from the correct position when it stops ~= 360*deadzone/(gear ratio n:1) degrees, currently about 4.5 each direction
    public static final Double retract = 2.0*1.125;
    public static final Double extend = 25.0*1.125;
    public static final Double neutral = 18.5*1.125;
    public static Double[] moveIntakeTo(Double posNum){
        Double[] newPow = {0.0, 0.0};
        int[] nums = new int[] {0, 1};
        for (int i : nums){ // do the same thing for the 2 motors
            Double error = posNum-motorList[i].getRotorPosition().getValueAsDouble();
            
            if (Math.abs(error) < deadZone){ // if in the deadzone, stop
                motorList[i].set(0.0);
            }else{ // if not in the deadzone, power is proportional to error
                if (error < 0){
                    newPow[i] = Math.max(-maxSpeed, error/20+0.05);
                }else{
                    newPow[i] = Math.min(maxSpeed, error/20-0.05);
                }
            }
        }
        return newPow;
    }
}
