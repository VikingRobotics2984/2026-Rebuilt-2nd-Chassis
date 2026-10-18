package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Transport {
    public static TalonSRX horizontalTransportMotor = new TalonSRX(Constants.horizontalTransportMotorID);
    public static TalonSRX upperVerticalTransportMotor = new TalonSRX(Constants.upperVerticalTransportMotorID);
    public static TalonFX lowerVerticalTransportMotor = new TalonFX(Constants.lowerVerticalTransportMotorID);

    public static Double horizontalPower = 0.0,
        lowerVerticalPower = 0.0,
        upperVerticalPower = 0.0;

    public static final int horizontalMaxCurrent = 10;// current max for horizontal motor
    public static final int upperVerticalMaxCurrent = 11;// current max for upper vertical motor
    public static final int lowerVerticalMaxCurrent = 18;// current max for lower vertical motor
    // configure the current limits for the motor
    public static void configureCurrentLimit(){
        // do it for lower vertical motor
        TalonFXConfigurator configurator = lowerVerticalTransportMotor.getConfigurator();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.SupplyCurrentLimit = lowerVerticalMaxCurrent;
        limitConfigs.SupplyCurrentLimitEnable = true;
        configurator.apply(limitConfigs);
        // different motor, different syntax for horizontal
        horizontalTransportMotor.configContinuousCurrentLimit(horizontalMaxCurrent);
        horizontalTransportMotor.enableCurrentLimit(true);
        // do the same thing for upper vertical motor
        upperVerticalTransportMotor.configContinuousCurrentLimit(upperVerticalMaxCurrent);
        upperVerticalTransportMotor.enableCurrentLimit(true);
    }

    public static final Double[] powerArray = {0.3, 0.4, 0.5, 0.6, 0.75};
    //public static final Double[] lowerVerticalPowers = {0.9, 0.75, 0.65, 0.55, 0.35};
    //public static final Double[] upperVerticalPowers = {1.0, 0.8, 0.7, 0.6, 0.4};
    
    public static void transportPower(){
        int kitchenStovePos = Math.max(0, Driver_Controller.kitchenStove()-7);
        
        if (Driver_Controller.buttonReverseShooter() || Driver_Controller.buttonReverseTransport()){
            horizontalPower = -(powerArray[kitchenStovePos]+0.2);
            lowerVerticalPower = -(powerArray[kitchenStovePos]+0.1);
            upperVerticalPower = -powerArray[kitchenStovePos];
        }else if ((Driver_Controller.buttonTransport() || DriverStation.isAutonomousEnabled()) && Shooter.close && 
            ((Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees()-Driver_Controller.SwerveCommandEncoderValue) < 5)
            || (!Driver_Controller.SwerveCommandControl))){
                
            horizontalPower = powerArray[kitchenStovePos]+0.2;
            lowerVerticalPower = powerArray[kitchenStovePos]+0.1;
            upperVerticalPower = powerArray[kitchenStovePos]+0.2;
            if (Driver_Controller.buttonPreloadTransport()){
                horizontalPower /= 2;
                lowerVerticalPower /= 2;
                upperVerticalPower /= 2;
            }
        }else {
            if (System.nanoTime()%(2500*1000*1000) < (750*1000*1000) && Driver_Controller.switchIntakeRoller()){
                horizontalPower = powerArray[kitchenStovePos]+0.2;
            } else {
                horizontalPower = 0.0;
            }
            lowerVerticalPower = 0.0;
            upperVerticalPower = 0.0;
        }
    }
}