package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Shooter {
    public static TalonFX rightShooter = new TalonFX(Constants.rightShooterMotorID);
    public static TalonFX leftShooter = new TalonFX(Constants.leftShooterMotorID);
    public static Double curPower = 0.0;
    public static Double[] prevSpeed = {0.0, 0.0, 0.0, 0.0, 0.0};
    public static int speedIndex = 0;

    public static Boolean close = false;
    public static Double avgSpeed = 0.0;

    public static final Double[]
        interpolationDists = {1.0, 1.5, 2.0, 2.5, 3.0},
        interpolationSpeeds = {40.0, 45.0, 50.0, 57.5, 67.0};

    public static Double speedController(Double desiredSpeed){
        int nextSpeedIndex = (speedIndex+1)%5;

        Double velocity = Math.abs(rightShooter.getVelocity().getValueAsDouble()); // rotations per second
        prevSpeed[speedIndex] = velocity;
        Double accel = (velocity-prevSpeed[nextSpeedIndex]);// rotations per second per second, based on .1 second ago

        Double p, i, d;
        p = desiredSpeed*0.0125/1.25; // 80 rps is max power?
        if ((desiredSpeed-velocity) < 0){
            p += (desiredSpeed-velocity)*0.01*0; // needs to slow down
        }else{
            p += (desiredSpeed-velocity)*0.02*0; // needs to go faster
        }
        i = (desiredSpeed-velocity)*0.00005/3;
        curPower += i;
        d = accel*(-0.015);

        if (velocity < desiredSpeed*0.9 || velocity > desiredSpeed*1.1){ // check if we are within 10% of correct speed
            //LED.setPattern('R');
        }else{
            //LED.setPattern('B');
        }

        avgSpeed += velocity/2;
        speedIndex = nextSpeedIndex;
        return Math.min(1.0, Math.max(0.0, p+curPower+d));
    }
}
