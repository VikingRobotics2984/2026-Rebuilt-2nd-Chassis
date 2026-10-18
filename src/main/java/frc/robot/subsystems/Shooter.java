package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Autonomous;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter {
    public static TalonFX rightShooter = new TalonFX(Constants.rightShooterMotorID);
    public static TalonFX leftShooter = new TalonFX(Constants.leftShooterMotorID);
    public static Double curPower = 0.0;
    public static Double[] prevSpeed = {0.0, 0.0, 0.0, 0.0, 0.0};
    public static int speedIndex = 0;

    public static Boolean close = false;
    public static Double avgSpeed = 0.0;

    public static final Double[]
        interpolationDists = {-99999.9, 2.374,  2.894,  3.440,  3.711,  4.391,  4.903,  5.385,    99999.9},
        interpolationSpeeds = {50.1,    50.1,   54.1,   56.2,   59.7,   64.1,   66.6,   71.2,     71.2};
    public static Double dist = 0.0;
    public static Double interpolatePower(){
        Double posX = RobotContainer.drivetrain.getState().Pose.getX();
        Double posY = RobotContainer.drivetrain.getState().Pose.getY();
        Double hubY = Autonomous.centerY;
        Double hubX;
        if (Robot.alliance == 'B'){
            hubX = Autonomous.hubX;
        }else{
            hubX = Autonomous.centerX*2-Autonomous.hubX;//if red alliance, flip the hub x over the center
        }
        dist = Math.sqrt(
            Math.pow(posX-hubX, 2)+
            Math.pow(posY-hubY, 2)
        );

        int lBound = 0, uBound = 1;
        while (uBound < (interpolationDists.length-1) && interpolationDists[uBound] < dist){
            uBound++;
            lBound++;
        }

        Double lSpeed = interpolationSpeeds[lBound], uSpeed = interpolationSpeeds[uBound];
        Double lDist = interpolationDists[lBound], uDist = interpolationDists[uBound];

        Double speed = ((lSpeed*(-dist+uDist))
            +(uSpeed*(-lDist+dist)))/(uDist-lDist);
        return speed;
    }

    public static Double speedController(Double desiredSpeed){
        int nextSpeedIndex = (speedIndex+1)%5;

        Double velocity = Math.abs(rightShooter.getVelocity().getValueAsDouble()); // rotations per second
        prevSpeed[speedIndex] = velocity;
        Double accel = (velocity-prevSpeed[nextSpeedIndex]);// rotations per second per second, based on .1 second ago

        Double p, i, d;
        p = (desiredSpeed*0.01)/1.1; // 100 rps is max power?
        p += (desiredSpeed-velocity)*0.004*2;
        i = (desiredSpeed-velocity)*0.00001/3;
        curPower += i;
        d = accel*(-0.005);

        if (velocity < (desiredSpeed-10) || velocity > (desiredSpeed+10)){ // check if we are within 10% of correct speed
            close = false;
            LED.setPattern('R');
        }else{
            close = true;
            LED.setPattern('B');
        }

        avgSpeed += velocity/2;
        speedIndex = nextSpeedIndex;
        return Math.min(1.0, Math.max(0.0, (p+curPower+d)*12/Math.abs(RobotController.getBatteryVoltage())));
    }
}
