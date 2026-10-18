package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Driver_Controller;

public class Autonomous {
    public static int autoState = 0, autoRampState = 0; // progress index shared between autos
    public static Double speed = 1.0, // driving speed might be changed with smartDashboard
        nextShootDist; // estimated distance from hub when shooting next based on smartDashboard selection
    public static final Double endVeloMult = 0.5; // modify the ending velocities of splines by a factor, also the math requires multiply by speed
    public static char alliance; // current alliance duh

    public static void setupAutos(){ // must run before other autos - reset autostate
        driveAngle = null;
        alliance = DriverStation.getAlliance().toString().charAt(9);
        autoState = 0;
        autoRampState = 0;
    }

    // following variables are really constants showing the numbers of the field - all in meters
    public static final Double
        centerY = 4.034663,
        hubX = 4.62534,
        centerX = (11.915394+hubX)/2,
        bumpY = 2.5041352,
        bumpWidth = 1.128;
    public static final char
        blueAlliance = 'B',
        redAlliance = 'R';

    public static Double orientBump(){
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Boolean flippedAngle = false;
        if (odoX < centerX) {
            if (odoX < hubX){
                flippedAngle = true;
            }
        }else if (odoX < (centerX*2-hubX)) {
            flippedAngle = true;
        }
        Double targAngle = (RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees() < 0)?(-45.0):45.0;
        if (flippedAngle){
            targAngle*= 3;
        }
        return targAngle;
    }

    public static Double destX1, destX2, destY, targetAngle;
    public static boolean autoRamp(){
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        switch(autoRampState){
            case 0: // set positions
                destY = ((odoY < centerY)? // assign the y destination based on which side of the field (in terms of y) are we on
                    bumpY:
                    (centerY*2-bumpY)
                );//flip y value across the field

                if (alliance == 'B'){ // same thing but for the first x position
                    if (odoX < hubX) { // check if we are entering or exiting alliance zone; first destination isn't the middle of bump
                        destX1 = hubX-(bumpWidth/2+0.5);
                        destX2 = hubX+(bumpWidth/2+0.5);
                    }else{
                        destX1 = hubX+(bumpWidth/2+0.5);
                        destX2 = hubX-(bumpWidth/2+0.5);
                    }
                }else{
                    if (odoX < (centerX*2-hubX)) { // same but flip
                        destX1 = centerX*2-hubX-(bumpWidth/2+0.5);
                        destX2 = centerX*2-hubX+(bumpWidth/2+0.5);
                    }else{
                        destX1 = centerX*2-hubX+(bumpWidth/2+0.5);
                        destX2 = centerX*2-hubX-(bumpWidth/2+0.5);
                    }
                }
                targetAngle = orientBump();
                autoRampState++;
            case 1:
                // wait until we are turned correctly
                if (Math.abs(RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees()-targetAngle) > 5){
                    Driver_Controller.SwerveCommandEncoderValue = targetAngle;
                    return false;
                } 

                if (Math.abs(odoX-destX1) < 0.2){
                    autoRampState = 3; // if we are in the correct place to immediately go over the bump, skip state 1
                    AutoDrive.setSpline(destX2, destY, (destX2-destX1)*speed*endVeloMult/2, 0.0, speed, 50);
                }else{
                    autoRampState = 2; // go toward the bump
                    AutoDrive.setSpline(destX1, destY, (destX2-destX1)*speed*endVeloMult/2, 0.0, speed, 50);
                }
                break;
            case 2:
                if (AutoDrive.driveSpline(targetAngle)){
                    AutoDrive.setSpline(destX2, destY, (destX2-destX1)*speed*endVeloMult/2, 0.0, speed, 50);
                    autoRampState++;
                }
                break;
            case 3:
                if (AutoDrive.driveSpline(targetAngle))return true;
                break;
        }
        Driver_Controller.SwerveCommandEncoderValue = targetAngle;
        return false;
    }

    public static Double driveAngle = null;
    public static Boolean fullPassAuto(){
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        switch(autoState){
            case 0: // go to the side of the field
                autoRampState = 0;
                ++autoState;
            case 1:
                if (autoRamp()){
                    Double xPos, yPos, xVelo;
                    if (alliance == 'B'){
                        xPos = centerX-1;
                        xVelo = 2.0*endVeloMult;
                        if (odoY < centerY){
                            yPos = centerY-2.75;
                            driveAngle = 75.0;
                        }else{
                            yPos = centerY+2.75;
                            driveAngle = -75.0;
                        }
                    }else{
                        xPos = centerX+1;
                        xVelo = -2.0*endVeloMult;
                        if (odoY < centerY){
                            yPos = centerY-2.75;
                            driveAngle = 105.0;
                        }else{
                            yPos = centerY+2.75;
                            driveAngle = -105.0;
                        }
                    }
                    AutoDrive.setSpline(xPos, yPos, xVelo*Math.max(1.5, speed), 0.0, speed, 50);
                    autoState++;
                }
                break;
            case 2:
                Robot.rollerPow = -1.0;
                if (AutoDrive.driveSpline(driveAngle)){
                    Double xPos, yPos = centerY;
                    if (alliance == 'B'){
                        xPos = centerX-1;
                    }else{
                        xPos = centerX+1;
                    }
                    if (odoY < centerY){
                        yPos += 1;
                    }else{
                        yPos -= 1;
                    }
                    autoRampState = 0;
                    AutoDrive.setSpline(xPos, yPos, 0.0, (centerY-odoY)*endVeloMult, Math.max(1.5, speed), 50);
                    autoState++;
                }
                break;
            case 3:
                Robot.rollerPow = -1.0;
                if (AutoDrive.driveSpline(driveAngle)){
                    autoRampState = 0;
                    autoState++;
                    driveAngle = null;
                }
                break;
            case 4:
                Robot.rollerPow = -1.0;
                Robot.spinShooters = true;
                return autoRamp();
        }
        return false;
    }

    public static Boolean halfPassAuto(){
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        switch(autoState){
            case 0: // go to the side of the field
                autoRampState = 0;
                ++autoState;
            case 1:
                if (autoRamp()){
                    Double xPos, yPos, xVelo;
                    if (alliance == 'B'){
                        xPos = centerX-1;
                        xVelo = 2.0*endVeloMult;
                        if (odoY < centerY){
                            yPos = centerY-2.75;
                            driveAngle = 75.0;
                        }else{
                            yPos = centerY+2.75;
                            driveAngle = -75.0;
                        }
                    }else{
                        xPos = centerX+1;
                        xVelo = -2.0*endVeloMult;
                        if (odoY < centerY){
                            yPos = centerY-2.75;
                            driveAngle = 105.0;
                        }else{
                            yPos = centerY+2.75;
                            driveAngle = -105.0;
                        }
                    }
                    AutoDrive.setSpline(xPos, yPos, xVelo*Math.max(1.5, speed), 0.0, speed, 50);
                    autoState++;
                }
                break;
            case 2:
                Robot.rollerPow = -1.0;
                if (AutoDrive.driveSpline(driveAngle)){
                    Double xPos, yPos = centerY;
                    if (alliance == 'B'){
                        xPos = centerX-0.3;
                    }else{
                        xPos = centerX+0.3;
                    }
                    if (odoY < centerY){
                        yPos -= 0.4;
                    }else{
                        yPos += 0.4;
                    }
                    autoRampState = 0;
                    AutoDrive.setSpline(xPos, yPos, (xPos-centerX)*endVeloMult, 0.0, Math.max(1.5, speed), 50);
                    autoState++;
                }
                break;
            case 3:
                Robot.rollerPow = -1.0;
                if (AutoDrive.driveSpline(driveAngle)){
                    autoRampState = 0;
                    autoState++;
                }
                break;
            case 4:
                Robot.rollerPow = -1.0;
                Robot.spinShooters = true;
                return autoRamp();
        }
        return false;
    }

    public static Boolean closeHalfPass(){
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        switch(autoState){
            case 0: // go to the side of the field
                autoRampState = 0;
                ++autoState;
            case 1:
                if (autoRamp()){
                    Double xPos, yPos, xVelo;
                    if (alliance == 'B') xPos = hubX+1.5;
                    else xPos = centerX*2-hubX-1.5;
                    xVelo = 2.0*endVeloMult*speed;
                    if (odoY < centerY){
                        yPos = centerY-0.3;
                        driveAngle = 90.0;
                    }else{
                        yPos = centerY+0.3;
                        driveAngle = -90.0;
                    }
                    AutoDrive.setSpline(xPos, yPos, 0.0, 0.0, Math.max(1.5, speed), 50);
                    autoState++;
                }
                break;
            case 2:
                Robot.rollerPow = -1.0;
                if (AutoDrive.driveSpline(driveAngle)){
                    autoRampState = 0;
                    autoState++;
                }
                break;
            case 3:
                Robot.rollerPow = -1.0;
                Robot.spinShooters = true;
                return autoRamp();
        }
        return false;
    }

    public static Boolean closeFullPass(){
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        switch(autoState){
            case 0: // go to the side of the field
                autoRampState = 0;
                ++autoState;
            case 1:
                if (autoRamp()){
                    Double xPos, yPos, xVelo;
                    if (alliance == 'B') xPos = hubX+1.5;
                    else xPos = centerX*2-hubX-1.5;
                    xVelo = 2.0*endVeloMult*speed;
                    if (odoY < centerY){
                        yPos = centerY+0.3;
                        driveAngle = 90.0;
                    }else{
                        yPos = centerY-0.3;
                        driveAngle = -90.0;
                    }
                    AutoDrive.setSpline(xPos, yPos, 0.0, 0.0, Math.max(1.5, speed), 50);
                    autoState++;
                }
                break;
            case 2:
                Robot.rollerPow = -1.0;
                if (AutoDrive.driveSpline(driveAngle)){
                    autoRampState = 0;
                    autoState++;
                }
                break;
            case 3:
                Robot.rollerPow = -1.0;
                Robot.spinShooters = true;
                return autoRamp();
        }
        return false;
    }

    public static Boolean toBump(){
        return true;
    }

    public static final Double[] shootingPosX = {1.5, 2.5, 2.0}, // x position for shooting according to smartDashboard choice
        shootingPosY = {1.5, 2.5, centerY};
    public static void setScoringSpline(){
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        Double Ypos, Xpos; // the variables representing the target point
        if (odoY < centerY){ // detecting which side of the field we are on
            Ypos = shootingPosY[Robot.scorePos];
        }else{
            Ypos = centerY*2-shootingPosY[Robot.scorePos];
        }
        if (alliance == 'R'){ // X pos is flipped if we are red vs blue
            Xpos = centerX*2-shootingPosX[Robot.scorePos];
        }else{
            Xpos = shootingPosX[Robot.scorePos];
        }
        Double angle = Math.atan2(Ypos-odoY, Xpos-odoX);// give the spline a little extra boost in the right direction to make it faster
        AutoDrive.setSpline(Xpos, Ypos, Math.cos(angle)*speed*endVeloMult, Math.sin(angle)*speed*endVeloMult, speed, 50);
    }
}
