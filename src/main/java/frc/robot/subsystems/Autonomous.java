package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// this is mostly organization for our different autos
public class Autonomous {
    public static char alliance;
    public static Boolean ready, idleShooter = true;
    public static Integer revdIntakeCnt = 0, prev_autostate = -1, shootPosition = 0;
    public static Double prevSpeed = 0.0;
    public static void shootAuto(Boolean transport){ // this function automatically shoots
        if (revdIntakeCnt <= -1){
            if (revdIntakeCnt == 0) prevSpeed = Transport.prevIntakePower;
            Transport.setIntake(-0.4);
            if (revdIntakeCnt == -1) Transport.setIntake(prevSpeed);
        }
        ++revdIntakeCnt;
        Turret.calcDist();
        Double shooterPower = Turret.resetEncoder();
        if (Math.abs(shooterPower) < 0.0001){
            shooterPower = Turret.spinTurret();
        }
        if (Driver_Controller.pauseTurret()) shooterPower = 0.0;
        Turret.turretSpin.set(shooterPower);
        if ((idleShooter == false) && (transport == false)){
            ready = false;
            Turret.shooter1.set(0.0);
            Turret.shooter2.set(0.0);
            Transport.setTransport(0.0);
            Transport.setSpindexer(0.0);
            Transport.agitate(false);
            return;
        }
        // spin up shooter and reset/aim turret and stuff
        
        if (transport == false) Turret.desiredSpeed = 20.0;
        Double[] power = Turret.speedController();
        Turret.shooter1.set(power[0]);
        Turret.shooter2.set(-power[1]);
        Turret.servo1.set(Turret.cowlAngle);
        Turret.servo2.set(Turret.cowlAngle-0.129);
        Turret.servoInverted.set(1-Turret.cowlAngle+0.162);
        // checks everything if ready to shoot
        ready |= (Turret.close
            && (Driver_Controller.pauseTurret() || (Math.abs(Turret.modTurretOff) < 5)));
        if (ready && transport){
            Transport.setSpindexer(Transport.powerArray[Driver_Controller.kitchenStove()]);
            Transport.setTransport(0.85);
            Transport.agitatorMotor.set(TalonSRXControlMode.PercentOutput, 0.8);
        }else{
            Transport.setTransport(0.0);
            Transport.setSpindexer(0.0);
            Transport.agitate(false);
        }
    }
    public static void shootAuto(){
        shootAuto(true);
    }

    public static Boolean shouldSkipMove = true;
    public static Double[] destPoints = {0.0, 0.0, 0.0}; // x1, x2, y
    public static void enterNeutralPoints(){
        shouldSkipMove = true;
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        Double odox = RobotContainer.drivetrain.getState().Pose.getX();
        if (odoy > 4.034663){
            destPoints[2] = 7.4247756;//-0.15;
            if (odoy < 4.034663*1.5) shouldSkipMove = false;
        }else{
            destPoints[2] = (4.034663*2-7.4247756);//+0.15;
            if (odoy > 4.034663*0.5) shouldSkipMove = false;
        }
        if (odox > (11.915394+4.62534)/2){
            destPoints[0] = 11.915394+1.0;
            if (odox > (11.915394+1.0)){
                shouldSkipMove = false;
            }
            destPoints[1] = 11.915394-2;
        }else{
            destPoints[0] = 4.62534-1.0;
            if (odox < (4.62534-1.0)){
                shouldSkipMove = false;
            }
            destPoints[1] = 4.62534+2;
        }
    }

    public static void enterAlliancePoints(){
        Double odoy = RobotContainer.drivetrain.getState().Pose.getY();
        if (odoy > 4.034663){
            destPoints[2] = 7.4247756;//-0.075;
        }else{
            destPoints[2] = (4.034663*2-7.4247756);//+0.075;
        }
        
        if (alliance == 'R'){
            destPoints[0] = 11.915394;
            destPoints[1] = 11.915394+1.5;
        }else{
            destPoints[0] = 4.62534;
            destPoints[1] = 4.62534-1.5;
        }
    }

    public static Double speed = 4.0, endVeloMult = 0.3*speed, shootTimeSec = 5.0;
    public static final Double slowTrenchSpeed = 2.0;
    public static Integer autoState = 0;
    public static void shuttleAuto(){
        ++cnt;
        shootAuto((autoState > 2) || autoState == 0);
        Double driveAngle = ((alliance == 'R')?80.0:100.0)*((destPoints[2] > 4.034663)?1.0:-1.0);
        switch(autoState){
            case 0:
                RobotContainer.stopMovement();
                if (cnt >= 50*shootTimeSec){
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    ++autoState;
                }
                break;
            case 1:
                enterNeutralPoints();
                AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                ++autoState;
            case 2:
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                    Transport.setIntake(0.6);
                }
                break;
            case 3:
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    AutoDrive.setSpline((14.552041+1.988947)/2, 4.034663-(destPoints[2]-4.034663)*0.6, 0.0, (destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 4:
                if (AutoDrive.driveSpline(-driveAngle)){
                    Double xAdd = (alliance == 'R')?1.0:-1.0;
                    AutoDrive.setSpline((14.552041+1.988947)/2+xAdd, 4.034663+(destPoints[2]-4.034663)*0.6, 0.0, -(destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                if (AutoDrive.driveSpline(driveAngle)){
                    Double xAdd = (alliance == 'R')?2.0:-2.0;
                    AutoDrive.setSpline((14.552041+1.988947)/2+xAdd, 4.034663-(destPoints[2]-4.034663)*0.6, 0.0, (destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 6:
                if (AutoDrive.driveSpline(-driveAngle)){
                    ++autoState;
                }
                break;
            case 7:
                Transport.setIntake(0.0);
                RobotContainer.stopMovement();
                break;
        }
    }

    public static void outpostAuto(){
        Double outpostA = ((alliance == 'R')?0.0:180);
        ++cnt;
        switch(autoState){
            case 0:
                RobotContainer.stopMovement();
                if (cnt >= 50*shootTimeSec){ // 5 seconds
                    Double outpostX = ((alliance == 'R')?11.915394+4.62534-0.5:0.5),
                    outpostY = ((alliance == 'R')?(4.034663*2-0.5):0.5);
                    AutoDrive.setSpline(outpostX, outpostY, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                shootAuto(false);
                Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
                Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
                if (AutoDrive.driveSpline(outpostA) ||
                    ((alliance == 'R')?odoX>(11.915394+4.62534-0.5):odoX<0.5) ||
                    ((alliance == 'R')?odoY>(4.034663*2-0.5):odoY<0.5)){
                        ++autoState;
                        cnt = 0;
                }
                break;
            case 2:
                shootAuto(false);
                Driver_Controller.SwerveControlSet(false);
                if (cnt >= 50*2){ // 2 seconds wait for fuel from outpost
                    Double ypos, xpos;
                    ++autoState;
                    if (shootPosition == 0){
                        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                        enterAlliancePoints();
                        xpos = destPoints[1];
                        ypos = destPoints[2];
                    }else if (shootPosition == 1){
                        xpos = destPoints[1];
                        ypos = (4.034663*2.0+destPoints[2])/3.0;
                    }else{
                        xpos = destPoints[1]*2.0-destPoints[0];
                        ypos = 4.034663;
                    }
                    AutoDrive.setSpline(xpos, ypos, 0.0, 0.0, speed, 50);
                }
                break;
            case 3:
                shootAuto(false);
                Double angle = (shootPosition==2?(Driver_Controller.pigeonOffset+((alliance=='R')?179.9:0.0)):RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
                if (AutoDrive.driveSpline(angle)) ++autoState;
                break;
            case 4:
                RobotContainer.stopMovement();
                shootAuto(true);
                break;
        }
    }

    public static void intakeAuto(){
        ++cnt;
        Double driveAngle = ((destPoints[2] > 4.034663)?-1.0:1.0)*((alliance == 'R')?80.0:100.0);
        switch(autoState){
            case 0:
                RobotContainer.stopMovement();
                if (cnt >= 50*shootTimeSec){
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                shootAuto(false);
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?-1.0:1.0);
                    AutoDrive.setSpline((14.552041+1.988947)/2, yPosition, 0.0, (destPoints[2]-4.034663)/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline(driveAngle)){
                    enterAlliancePoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*Math.min(0.3*slowTrenchSpeed, endVeloMult), 0.0, Math.min(slowTrenchSpeed, speed), 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline(false)){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline(false)){
                    ++autoState;
                    if (shootPosition == 0){
                        ++autoState;
                        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    }else{
                        Double ypos, xpos;
                        if (shootPosition == 1){
                            xpos = destPoints[1];
                            ypos = (4.034663*2.0+destPoints[2])/3.0;
                        }else{
                            xpos = destPoints[1]*2.0-destPoints[0];
                            ypos = 4.034663;
                        }
                        AutoDrive.setSpline(xpos, ypos, 0.0, 0.0, speed, 50);
                    }
                }
                break;
            case 6:
                shootAuto(false);
                Double angle = (shootPosition==2?(Driver_Controller.pigeonOffset+((alliance=='R')?179.9:0.0)):RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
                if (AutoDrive.driveSpline(angle)) ++autoState;
                break;
            case 7:
                RobotContainer.stopMovement();
                ++autoState;
                break;
            case 8:
                shootAuto(true);
                break;
        }
    }
    
    public static void halfIntakeAuto(){
        ++cnt;
        switch(autoState){
            case 0:
                RobotContainer.stopMovement();
                if (cnt >= 50*shootTimeSec){
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                shootAuto(false);
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?0.5:-0.5);
                    AutoDrive.setSpline((14.552041+1.988947)/2+((alliance == 'R')?0.5:-0.5), yPosition, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline(false)){
                    enterAlliancePoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], 1.5*(destPoints[0]-destPoints[1])*Math.min(0.3*slowTrenchSpeed, endVeloMult), 0.0, Math.min(slowTrenchSpeed, speed), 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline(false)){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline((alliance == 'R')?0.0:180.0)){
                    ++autoState;
                    if (shootPosition == 0){
                        ++autoState;
                        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    }else{
                        Double ypos, xpos;
                        if (shootPosition == 1){
                            xpos = destPoints[1];
                            ypos = (4.034663*2.0+destPoints[2])/3.0;
                        }else{
                            xpos = destPoints[1]*2.0-destPoints[0];
                            ypos = 4.034663;
                        }
                        AutoDrive.setSpline(xpos, ypos, 0.0, 0.0, speed, 50);
                    }
                }
                break;
            case 6:
                shootAuto(false);
                Double angle = (shootPosition==2?(Driver_Controller.pigeonOffset+((alliance=='R')?179.9:0.0)):RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
                if (AutoDrive.driveSpline(angle)) ++autoState;
                break;
            case 7:
                RobotContainer.stopMovement();
                ++autoState;
                break;
            case 8:
                shootAuto(true);
                break;
        }
    }

    public static void depotAuto(){
        ++cnt;
        Double depotA = ((alliance == 'R')?85.0:-85.0);
        switch(autoState){
            case 0:
                RobotContainer.stopMovement();
                if (cnt >= 50*shootTimeSec){ // 5 seconds
                    Double depotX = ((alliance == 'R')?11.915394+4.62534-1.0:1.0),
                    depotY = 4.034663*((alliance == 'R')?0.25:1.75);
                    AutoDrive.setSpline(depotX, depotY, -2*((alliance == 'R')?1.0:-1.0)*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                shootAuto(true);
                break;
            case 1:
                Transport.setIntake(0.6);
                shootAuto(false);
                if (AutoDrive.driveSpline(depotA)){
                    Double depotX = ((alliance == 'R')?11.915394+4.62534-0.5:0.5),
                        depotY = 4.034663*((alliance == 'R')?0.35:1.65);
                    AutoDrive.setSpline(depotX, depotY, 0.0, ((alliance == 'R')?-1.0:1.0), 1.5, 50);
                    ++autoState;
                }
                break;
            case 2:
                Transport.setIntake(0.6);
                shootAuto(false);
                if (AutoDrive.driveSpline(depotA)){
                    Double depotX = ((alliance == 'R')?11.915394+4.62534-0.5:0.5),
                        depotY = 4.034663*((alliance == 'R')?0.75:1.25);
                    AutoDrive.setSpline(depotX, depotY, 0.0, ((alliance == 'R')?-1.0:1.0), 1.0, 50);
                    ++autoState;
                }
                break;
            case 3:
                Transport.setIntake(0.6);
                shootAuto(false);
                if (AutoDrive.driveSpline(depotA)){
                    Double desiredX = ((alliance == 'R')?
                        Turret.redTargetX[2]+1.538:
                        Turret.blueTargetX[2]-1.538),
                    desiredY = Turret.TargetY[2] +
                        ((alliance == 'R')?- 1.336606:1.336606);
                    AutoDrive.setSpline(desiredX, desiredY, 0.0, 0.0, speed/2, 50);
                    ++autoState;
                }
                break;
            case 4:
                Transport.setIntake(0.6);
                shootAuto(false);
                if (AutoDrive.driveSpline(depotA)){
                    Transport.setIntake(0.0);
                    ready = false;
                    ++autoState;
                }
                break;
            case 5:
                RobotContainer.stopMovement();
                shootAuto(true);
                break;
        }
    }
    public static int cnt = 0;
    public static void halfIntakeReturnAuto(){
        ++cnt;
        switch(autoState){
            case 0:// wait to shoot preloads, set up spline
                RobotContainer.stopMovement();
                if (cnt >= 50*shootTimeSec){
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:// do the first move through the trench (or not if close to trench)
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(false);
                break;
            case 2: // get to the neutral zone
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?0.5:-0.5);
                    AutoDrive.setSpline((14.552041+1.988947)/2+((alliance == 'R')?1.0:-1.0), yPosition, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 3: // go toward middle
                shootAuto(false);
                if (AutoDrive.driveSpline(false)){
                    enterAlliancePoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], 1.5*(destPoints[0]-destPoints[1])*Math.min(0.3*slowTrenchSpeed, endVeloMult), 0.0, Math.min(slowTrenchSpeed, speed), 50);
                    ++autoState;
                }
                break;
            case 4: // return to trench
                shootAuto(false);
                if (AutoDrive.driveSpline(false)){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5: // get to Alliance zone
                shootAuto(false);
                if (AutoDrive.driveSpline(false)){
                    ++autoState;
                    if (shootPosition == 0){
                        ++autoState;
                        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    }else{
                        Double ypos, xpos;
                        if (shootPosition == 1){
                            System.out.println("hee");
                            xpos = destPoints[1];
                            ypos = (4.034663*2.0+destPoints[2])/3.0;
                        }else{
                            xpos = destPoints[1]*2.0-destPoints[0];
                            ypos = 4.034663;
                        }
                        AutoDrive.setSpline(xpos, ypos, 0.0, 0.0, speed, 50);
                    }
                }
                break;
            case 6:
            System.out.println("here");
                shootAuto(false);
                Double angle = (shootPosition==2?(Driver_Controller.pigeonOffset+((alliance=='R')?179.9:0.0)):RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
                if (AutoDrive.driveSpline(angle)) ++autoState;
                break;
            case 7: // shoot until 5 secs left
                Driver_Controller.SwerveCommandXValue = 0.0;
                Driver_Controller.SwerveCommandYValue = 0.0;
                Integer time = Math.toIntExact(Math.round(DriverStation.getMatchTime()));
                if (cnt >= 15*50){
                    Turret.shooter1.set(0.0);
                    Turret.shooter2.set(0.0);
                    Transport.setTransport(0.0);
                    Transport.setSpindexer(0.0);
                    Transport.agitate(false);
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                    break;
                }
                shootAuto(true);
                break;
            case 8: // go to trench
                if (shouldSkipMove || AutoDrive.driveSpline(true)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 9: // go into neutral
                if (AutoDrive.driveSpline(true)){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?1.0:-1.0);
                    AutoDrive.setSpline((14.552041+1.988947)/2, yPosition, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 10: // go toward center and stop
                if (AutoDrive.driveSpline(false)){
                    RobotContainer.stopMovement();
                    ++autoState;
                }
                break;
        }
    }

    public static Double startIntakeAngle = 0.0;
    public static void hubIntakeAuto(){
        ++cnt;
        switch(autoState){
            case 0:
                RobotContainer.stopMovement();
                if (cnt >= 50*shootTimeSec){
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[0]*0.125+destPoints[1]*0.875, destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult/2, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(false);
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    AutoDrive.setSpline(destPoints[0]*0.125+destPoints[1]*0.85, 4.034663*2-destPoints[2]*0.9, 0.0, -(4.034663-destPoints[2])/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    enterAlliancePoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], 1.5*(destPoints[0]-destPoints[1])*Math.min(0.3*slowTrenchSpeed, endVeloMult), 0.0, Math.min(slowTrenchSpeed, speed), 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    ++autoState;
                    if (shootPosition == 0){
                        ++autoState;
                        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    }else{
                        Double ypos, xpos;
                        if (shootPosition == 1){
                            xpos = destPoints[1];
                            ypos = (4.034663*2.0+destPoints[2])/3.0;
                        }else{
                            xpos = destPoints[1]*2.0-destPoints[0];
                            ypos = 4.034663;
                        }
                        AutoDrive.setSpline(xpos, ypos, 0.0, 0.0, speed, 50);
                    }
                }
                break;
            case 6:
                shootAuto(false);
                Double angle = (shootPosition==2?(Driver_Controller.pigeonOffset+((alliance=='R')?179.9:0.0)):RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
                if (AutoDrive.driveSpline(angle)) ++autoState;
                break;
            case 7:
                RobotContainer.stopMovement();
                shootAuto(true);
                break;
        }
    }

    public static void halfHubAuto(){
        ++cnt;
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[0]*0.125+destPoints[1]*0.875, destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult/2, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(false);
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    AutoDrive.setSpline(destPoints[0]*0.125+destPoints[1]*0.85, 4.034663+((destPoints[2]>4.034663)?0.5+speed*0.1:-0.5-speed*0.1), 0.0, -(4.034663-destPoints[2])/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    enterAlliancePoints();
                    if (alliance == 'B') destPoints[0] += 1.0;
                    else destPoints[0] -= 1.0;
                    AutoDrive.setSpline(destPoints[0], destPoints[2], 1.5*(destPoints[0]-destPoints[1])*Math.min(0.3*slowTrenchSpeed, endVeloMult), 0.0, Math.min(slowTrenchSpeed, speed), 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    ++autoState;
                    if (shootPosition == 0){
                        ++autoState;
                        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    }else{
                        Double ypos, xpos;
                        if (shootPosition == 1){
                            xpos = destPoints[1];
                            ypos = (4.034663*2.0+destPoints[2])/3.0;
                        }else{
                            xpos = destPoints[1]*2.0-destPoints[0];
                            ypos = 4.034663;
                        }
                        AutoDrive.setSpline(xpos, ypos, 0.0, 0.0, speed, 50);
                    }
                }
                break;
            case 6:
                shootAuto(false);
                Double angle = (shootPosition==2?(Driver_Controller.pigeonOffset+((alliance=='R')?179.9:0.0)):RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
                if (AutoDrive.driveSpline(angle)) ++autoState;
                break;
            case 7:
                RobotContainer.stopMovement();
                shootAuto(true);
                break;
        }
    }

    public static void halfHubReturnAuto(){
        ++cnt;
        switch(autoState){
            case 0:
                if (cnt >= 50*shootTimeSec){
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    startIntakeAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    ++autoState;
                }
                shootAuto(true);
                break;
            case 1:
                if (shouldSkipMove || AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[0]*0.125+destPoints[1]*0.875, destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult/2, 0.0, speed, 50);
                    ++autoState;
                }
                shootAuto(false);
                break;
            case 2:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    AutoDrive.setSpline(destPoints[0]*0.125+destPoints[1]*0.85, 4.034663+((destPoints[2]>4.034663)?0.5+speed*0.1:-0.5-speed*0.1), 0.0, -(4.034663-destPoints[2])/2*endVeloMult, speed, 50);
                    ++autoState;
                }
                break;
            case 3:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    enterAlliancePoints();
                    if (alliance == 'B') destPoints[0] += 1.0;
                    else destPoints[0] -= 1.0;
                    AutoDrive.setSpline(destPoints[0], destPoints[2], 1.5*(destPoints[0]-destPoints[1])*Math.min(0.3*slowTrenchSpeed, endVeloMult), 0.0, Math.min(slowTrenchSpeed, speed), 50);
                    ++autoState;
                }
                break;
            case 4:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    Transport.setIntake(0.0);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 5:
                shootAuto(false);
                if (AutoDrive.driveSpline(startIntakeAngle)){
                    ++autoState;
                    if (shootPosition == 0){
                        ++autoState;
                        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
                    }else{
                        Double ypos, xpos;
                        if (shootPosition == 1){
                            xpos = destPoints[1];
                            ypos = (4.034663*2.0+destPoints[2])/3.0;
                        }else{
                            xpos = destPoints[1]*2.0-destPoints[0];
                            ypos = 4.034663;
                        }
                        AutoDrive.setSpline(xpos, ypos, 0.0, 0.0, speed, 50);
                    }
                }
                break;
            case 6:
                shootAuto(false);
                Double angle = (shootPosition==2?(Driver_Controller.pigeonOffset+((alliance=='R')?179.9:0.0)):RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
                if (AutoDrive.driveSpline(angle)) ++autoState;
                break;
            case 7:
                RobotContainer.stopMovement();
                shootAuto(true);
                if (cnt >= 15*50){
                    Turret.shooter1.set(0.0);
                    Turret.shooter2.set(0.0);
                    Transport.setTransport(0.0);
                    Transport.setSpindexer(0.0);
                    Transport.agitate(false);
                    enterNeutralPoints();
                    AutoDrive.setSpline(destPoints[0], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                    break;
                }
                break;
            case 8: // go to trench
                if (shouldSkipMove || AutoDrive.driveSpline(((alliance == 'R')?180.0:0.0))){
                    Transport.setIntake(0.6);
                    AutoDrive.setSpline(destPoints[1], destPoints[2], (destPoints[0]-destPoints[1])*endVeloMult, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 9: // go into neutral
                if (AutoDrive.driveSpline(false)){
                    Double yPosition = 4.034663+((destPoints[2] > 4.034663)?1.0:-1.0);
                    AutoDrive.setSpline((14.552041+1.988947)/2, yPosition, 0.0, 0.0, speed, 50);
                    ++autoState;
                }
                break;
            case 10: // go toward center and stop
                if (AutoDrive.driveSpline(false)){
                    RobotContainer.stopMovement();
                    ++autoState;
                }
                break;
        }
    }

    public static void driveStraightAuto(){
        ++cnt;
        if (cnt < 50*17){
            shootAuto(true);
            RobotContainer.stopMovement();
        }else{
            shootAuto(false);
            Double odox = RobotContainer.drivetrain.getState().Pose.getX();
            if (alliance == 'B'){
                if (odox < ((14.552041+1.988947)/2)-0.5){
                    Driver_Controller.SwerveCommandXValue = 2.0;
                    Driver_Controller.SwerveControlSet(true);
                }else{
                    RobotContainer.stopMovement();
                }
            }else{
                if (odox > ((14.552041+1.988947)/2)+0.5){
                    Driver_Controller.SwerveCommandXValue = -2.0;
                    Driver_Controller.SwerveControlSet(true);
                }else{
                    RobotContainer.stopMovement();
                }
            }
        }
    }

    public static void reset(){
        endVeloMult = 0.3*speed;
        revdIntakeCnt = 0;
        cnt = 0;
        Turret.encoderStatus = 's';
        autoState = 0;
        alliance = DriverStation.getAlliance().toString().charAt(9);
        ready = false;
    }
}