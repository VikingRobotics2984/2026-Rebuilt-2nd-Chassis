package frc.robot.subsystems;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Driver_Controller;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
public class AutoDrive{
    public static Double sliderAdjust = 1.0;
    public static Boolean isDriving = false, goBehindReef = true;
    public static Double accelFac = 0.0, autoDriveMaxSpeed = 2.0;
    public static char alliance = 'R';
    public static Boolean driveToXYA(Double x, Double y, Double angle, Double speed, Double slowDownDist){
        Driver_Controller.SwerveControlSet(true);
        Double odoAngle = ((RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees()+ 360*1000 + 180)%360);
        //((RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble() + 360*1000 + 180)%360) - 180;
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();

        // use pythagorean theorum to calculate distance and if it is close enough
        
        Double dist = Math.sqrt(Math.pow(odox-x, 2)+Math.pow(odoy-y, 2));
        Double driveAngle = Math.atan2(y - odoy, x - odox);
        // limit max "speed" to autoDriveMaxSpeed
        if (speed > autoDriveMaxSpeed) speed = autoDriveMaxSpeed;
        if (speed < -autoDriveMaxSpeed) speed = -autoDriveMaxSpeed;

        // accelerate power when starting and lower power when approaching the destination, add a bit to dist so it goes faster at the end
        Double accelMult = Math.min(1.0, Math.min((dist+0.1)/slowDownDist, accelFac));
        //adjust speed based on the slider
        speed *= sliderAdjust;
        speed *= accelMult;
        speed += 0.15; // add a little bit of speed to avoid dead spot
        speed *= ((alliance == 'R')?1:-1); // opposite directions if red vs blue
        Double AllowedError = 0.05;
        if (slowDownDist < 0.001)AllowedError = 0.0;
        if (dist > AllowedError){
            Driver_Controller.SwerveCommandXValue = speed*Math.cos(driveAngle);
            Driver_Controller.SwerveCommandYValue = speed*Math.sin(driveAngle);
        }else{
            Driver_Controller.SwerveCommandXValue = 0.0;
            Driver_Controller.SwerveCommandYValue = 0.0;
        }
        Driver_Controller.SwerveCommandEncoderValue = odoAngle*0 + angle;
        return !(dist > 0.05);
    }
    public static Boolean driveToXYA(Double x, Double y, Double angle, Double speed){
        return driveToXYA(x, y, angle, speed, 2.0);
    }

    public static Boolean validSpline = false;

    static ArrayList<Double> Xpos = new ArrayList<>();
    static ArrayList<Double> Ypos = new ArrayList<>();
    static int totalPoints, progress;
    static Double maxDist = 0.0;
    public static void setSpline(Double destX, Double destY, Double endX, Double endY, Double speed, Integer step){
        Double startY = RobotContainer.drivetrain.getState().Pose.getY();
        Double startX = RobotContainer.drivetrain.getState().Pose.getX();
        Double yVelo = RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond,
            xVelo = RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond;
        Double angle = Math.atan2(yVelo, xVelo)+Math.toRadians((RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees()+ 360*1000 + 180)%360);
        Double dist = Math.sqrt(Math.pow(yVelo, 2) + Math.pow(xVelo, 2));
        Double srcX = -dist*Math.cos(angle)/speed;
        Double srcY = -dist*Math.sin(angle)/speed;
        destX -= startX;
        destY -= startY;
        
        while (Xpos.size() > 0) Xpos.remove(0);
        while (Ypos.size() > 0) Ypos.remove(0);
        Xpos.ensureCapacity(step+1);
        Ypos.ensureCapacity(step+1);
        // the math for this is in the "spline math" google sheet
        totalPoints = step+1;
        progress = 0;
        Double[] startScaling = new Double[step+1];
        Double[] endScaling = new Double[step+1];
        Double[] mainScaling = new Double[step+1];
        for (int i = 0; i <= step; ++i){
            startScaling[i] = 1.0-(i*1.0/step);
            endScaling[i] = i*1.0/step;
            mainScaling[i] = Math.min(i*1.0/step, 1.0-(i*1.0/step));
        }
        Double[] destXpoints = new Double[step+1];  Arrays.fill(destXpoints, 0.0);
        Double[] destYpoints = new Double[step+1];  Arrays.fill(destYpoints, 0.0);
        Double[] srcXpoints = new Double[step+1];   Arrays.fill(srcXpoints, 0.0);
        Double[] srcYpoints = new Double[step+1];   Arrays.fill(srcYpoints, 0.0);
        Double[] endXpoints = new Double[step+1];   Arrays.fill(endXpoints, 0.0);
        Double[] endYpoints = new Double[step+1];   Arrays.fill(endYpoints, 0.0);
        
        
        for (int i = 1; i < step; ++i){
            destXpoints[i] = destXpoints[i-1]+(2*(mainScaling[i]+mainScaling[i-1])*destX/step);
            destYpoints[i] = destYpoints[i-1]+(2*(mainScaling[i]+mainScaling[i-1])*destY/step);
            srcXpoints[i] = (srcX/step*startScaling[i])+(srcXpoints[i-1]*startScaling[i+1]/startScaling[i]);
            srcYpoints[i] = (srcY/step*startScaling[i])+(srcYpoints[i-1]*startScaling[i+1]/startScaling[i]);
        }
        for (int i = step-1; i >= 1; --i){
            endXpoints[i] = (endX/step*endScaling[i])+(endXpoints[i+1]*endScaling[i-1]/endScaling[i]);
            endYpoints[i] = (endY/step*endScaling[i])+(endYpoints[i+1]*endScaling[i-1]/endScaling[i]);
        }
        maxDist = 0.0;
        destXpoints[step] = destX;
        destYpoints[step] = destY;
        for (int i = 0; i < step; ++i){
            maxDist = Math.max(maxDist, Math.sqrt(Math.pow(destXpoints[i]-destXpoints[i+1], 2)+Math.pow(destYpoints[i]-destYpoints[i+1], 2)));
            //System.out.println(Math.sqrt(Math.pow(destXpoints[i]-destXpoints[i+1], 2)+Math.pow(destYpoints[i]-destYpoints[i+1], 2)));
            
        }
        for (int i = 0; i < step; ++i){
            Double srcScalingPow = startScaling[i];//*startScaling[i];
            Double endScalingPow = endScaling[i];//*endScaling[i];
            Xpos.add(i, startX+destXpoints[i]+(srcXpoints[i]*srcScalingPow+endXpoints[i]*endScalingPow)*(maxDist*50/speed));
            Ypos.add(i, startY+destYpoints[i]+(srcYpoints[i]*srcScalingPow+endYpoints[i]*endScalingPow)*(maxDist*50/speed));
        }
        Xpos.add(step, destX+startX);
        Ypos.add(step, destY+startY);
        for (int i = 0; i <= step; ++i){
            //System.out.printf("x = %.3f, y = %.3f\n", Xpos.get(i), Ypos.get(i));
        }
        maxDist /= speed;
        validSpline = true;
    }

    public static Double correctionMult = 1.0;
    public static Boolean driveSpline(Double specifiedAngle){
        if (!validSpline) return true;
        Double speed = 1.0;
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
        while (progress < totalPoints - 2){
            if (Math.sqrt(Math.pow(odox-Xpos.get(progress+1), 2)+Math.pow(odoy-Ypos.get(progress+1), 2)) < Math.sqrt(Math.pow(Xpos.get(progress)-odox, 2)+Math.pow(Ypos.get(progress)-odoy, 2))){
                ++progress;
            }else break;
        }
        Double x = Xpos.get(progress+1), y = Ypos.get(progress+1);
        Double pointsDist = Math.sqrt(Math.pow(Xpos.get(progress)-Xpos.get(progress+1), 2)+Math.pow(Ypos.get(progress)-Ypos.get(progress+1), 2));
        //calculate how far off from the ideal locationit is
        Double m1 = (Ypos.get(progress)-Ypos.get(progress+1))/(Xpos.get(progress)-Xpos.get(progress+1));
        Double b1 = Ypos.get(progress)-m1*Xpos.get(progress);
        Double m2 = -1/m1;
        Double b2 = y-m2*x;
        Double idealX = ((b1-b2)/(m2-m1));
        Double dist = Math.sqrt(Math.pow(x-idealX, 2)+Math.pow((idealX-x)*m2, 2));
        
        Double driveAngle = Math.atan2(Ypos.get(progress+1)-Ypos.get(progress)+(correctionMult*Math.sin(Math.atan(m2))*dist/(totalPoints-1)), Xpos.get(progress+1)-Xpos.get(progress)+(correctionMult*Math.cos(Math.atan(m2))*dist/(totalPoints-1)));
        //adjust speed based on the slider
        speed *= sliderAdjust;
        speed *= pointsDist/maxDist;

        //System.out.println(pointsDist/maxDist);
        speed = Math.max(speed, 0.2);
        Double maxError = speed*0.2;
        
        speed *= ((alliance == 'R')?1:-1); // opposite directions if red vs blue
        dist = Math.sqrt(Math.pow(x-odox, 2)+Math.pow((odoy-x), 2));
        if (progress < totalPoints - 2 || ((Math.sqrt(Math.pow(odox-Xpos.get(totalPoints-1), 2)+Math.pow(odoy-Ypos.get(totalPoints-1), 2)) > maxError) && dist < pointsDist) ){
            Driver_Controller.SwerveCommandXValue = -speed*Math.cos(driveAngle);
            Driver_Controller.SwerveCommandYValue = -speed*Math.sin(driveAngle);
            if (specifiedAngle < -99998){
                specifiedAngle = Math.toDegrees(driveAngle);
                
            }
            Driver_Controller.SwerveCommandEncoderValue = specifiedAngle;
            Driver_Controller.SwerveControlSet(true);
            return false;
        }else{
            validSpline = false;
            return true;
        }
    }
    public static Boolean driveSpline(Boolean keepAngle){
        if (keepAngle){
            return driveSpline(RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
        }
        else return driveSpline(-999999.0);
    }
}