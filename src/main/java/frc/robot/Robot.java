// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Driver_Controller;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class Robot extends TimedRobot {
  public static Boolean resetLastPressed = false;
  public static char alliance = 'B';
  private Command m_autonomousCommand;
  private Vision vision, vision2;
  public final RobotContainer m_robotContainer;
  public static DigitalInput[] sensors = {new DigitalInput(0), new DigitalInput(1), new DigitalInput(2), new DigitalInput(3), new DigitalInput(5)};

  public Robot() {
    m_robotContainer = new RobotContainer();
  }
  @Override
  public void robotInit() {
    Intake.configureCurrentLimit(Intake.maxCurrent);
    Transport.configureCurrentLimit();
    Constants.Vision.readLayout();
    LED.assignPort();
    vision = new Vision(RobotContainer.drivetrain::addVisionMeasurement, new PhotonCamera(Constants.Vision.kCameraName), new PhotonPoseEstimator(Constants.Vision.kTagLayout, Constants.Vision.kRobotToCam));
    vision2 = new Vision(RobotContainer.drivetrain::addVisionMeasurement, new PhotonCamera(Constants.Vision.kCamera2Name), new PhotonPoseEstimator(Constants.Vision.kTagLayout, Constants.Vision.kRobotToCam2));
  }
  
  @Override
  public void robotPeriodic() {
    alliance = DriverStation.getAlliance().toString().charAt(9);
    Driver_Controller.SwerveInputPeriodic();
    CommandScheduler.getInstance().run(); 
    vision.periodic();
    vision2.periodic();
    if (System.nanoTime()%(500*1000*1000) < (20*1000*1000)){
      //System.out.println(RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if ((vision.camera.isConnected() && vision2.camera.isConnected()) == false){
      LED.setPattern('e'); // don't have cameras
    }else if (vision.seenTags || vision2.seenTags){
      LED.setPattern("rainbow"); // either camera seeing good stuff
    }else{
      LED.setPattern("warn");
    }
  }

  @Override
  public void disabledExit() {}
  
  public static int autoState = 0, waitCnt = 0, scorePos, shootingTime;
  public static Boolean keepSpinning;
  public static String selectedPass, hopperState, secondPass, endPos;

  @Override
  public void autonomousInit() {
    Intake.needReset = true;
    Autonomous.setupAutos();
    autoState = 0;
    waitCnt = RobotContainer.waitCounter.getSelected();
    Autonomous.speed = RobotContainer.speedChooser.getSelected();
    selectedPass = RobotContainer.autoChooser.getSelected();
    secondPass = RobotContainer.secondAutoChooser.getSelected();
    keepSpinning = RobotContainer.shootFastSelector.getSelected();
    scorePos = RobotContainer.shootPositionChooser.getSelected();
    endPos = RobotContainer.lastPositionChooser.getSelected();
    shootingTime = (int)(RobotContainer.shootTimer.getSelected()*50);
    hopperState = "retracted";
  }

  public static Boolean transporting = false, spinShooters = false;
  public static Double rollerPow;
  @Override
  public void autonomousPeriodic() {
    Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
    Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
    spinShooters = keepSpinning;
    transporting = false;
    Boolean orienting = false;
    rollerPow = 0.0;
    Transport.horizontalPower = 0.0;
    Transport.lowerVerticalPower = 0.0;
    Transport.upperVerticalPower = 0.0;

    switch (autoState){
      case 0://wait for the correct amount of time
        if (--waitCnt < 0){// waitCnt was chosen by smartDashboard
          Autonomous.setupAutos();
          autoState++;
        }
        break;

      case 1: // do our first pass or stay and shoot
        switch (selectedPass) {
          case "full":
            hopperState = "extend";
            if (Autonomous.fullPassAuto()){
              autoState++;
              Autonomous.setScoringSpline();
            }
            break;
          case "half":
            hopperState = "extend";
            if (Autonomous.halfPassAuto()){
              autoState++;
              Autonomous.setScoringSpline();
            }
            break;
          case "depot": // not sure this is worth the effort?
            autoState++;
            break;
          case "no intake":
            spinShooters = true;
            Double xError = ((alliance=='B')?2:(Autonomous.centerX*2-2))-odoX;
            Driver_Controller.SwerveCommandXValue = Math.max(-0.5, Math.min(xError, 0.5));
            Driver_Controller.SwerveCommandYValue = 0.0;
            if (xError < 0.1)
              autoState = 6; // skip to the "second" shooting part
            break;
        }
        break;

      case 2: // go to the correct shooting position
        if (AutoDrive.driveSpline(true)){
          autoState++;
          waitCnt = shootingTime; // set the timer according to SmartDashboard
        }
        orienting = true;
        spinShooters = true;
        break;

      case 3: // shoot until our scoring timer runs out
        RobotContainer.stopMovement();
        hopperState = "retract";
        if (Intake.leftLinkageMotor.getRotorPosition().getValueAsDouble() > 7) rollerPow = -1.0;// run the motor unless mostly retracted
        orienting = spinShooters = transporting = true;
        if (--waitCnt < 0){// waitCnt was chosen by smartDashboard
          autoState++;
          Autonomous.setupAutos();
        }
        break;

      case 4: // second pass or skip it
        switch(secondPass){
          case "none": // if we are staying, then skip to the end positioning state
            autoState = 7;
            break;
          case "half":
            hopperState = "extend";
            rollerPow = -1.0;
            if (Autonomous.closeHalfPass()){
              autoState++;
              Autonomous.setScoringSpline();
            }
            break;
          case "full":
            hopperState = "extend";
            rollerPow = -1.0;
            if (Autonomous.closeFullPass()){
              autoState++;
              Autonomous.setScoringSpline();
            }
            break;
        }
        break;

      case 5: // go to our shooting position
        if (AutoDrive.driveSpline(true)){
          autoState++;
          waitCnt = shootingTime; // set the timer according to SmartDashboard
        }
        orienting = true;
        spinShooters = true;
        break;

      case 6: // shoot our hopper
        RobotContainer.stopMovement();
        hopperState = "retract";
        if (Intake.leftLinkageMotor.getRotorPosition().getValueAsDouble() > 7) rollerPow = -1.0;
        orienting = spinShooters = transporting = true;
        if (--waitCnt < 0){// waitCnt was chosen by smartDashboard
          Autonomous.setupAutos();
          autoState++;
        }
        break;

      case 7: // go to ending position
        switch(endPos) {
          case "none":
            RobotContainer.stopMovement();
            autoState++;
            break;

          case "bump":
            hopperState = "extend";
            Driver_Controller.SwerveCommandEncoderValue = Autonomous.orientBump();
            if (Autonomous.toBump()){
              RobotContainer.stopMovement();
              autoState++;
            }
            break;
          case "neutral":
            hopperState = "extend";
            if (Autonomous.autoRamp()){
              autoState++;
              RobotContainer.stopMovement();
            }
            break;
        }
        break;
    }
    Double shooterPow = 0.0;
    if (spinShooters){
      shooterPow = Shooter.speedController(Shooter.interpolatePower());
    }
    Shooter.leftShooter.set(shooterPow);
    Shooter.rightShooter.set(shooterPow);
    if (orienting){
      
      Double hubX = (alliance == 'B')?Autonomous.hubX:(Autonomous.centerX*2-Autonomous.hubX);
      Double angle = Math.toDegrees(Math.atan2(Autonomous.centerY-odoY, hubX-odoX))+2;
      Driver_Controller.SwerveCommandEncoderValue = angle;
    }
    Transport.autoTransportPower();
    if (transporting){
      hopperState = "retract";
    }

    Double[] linkagePow = Intake.resetLinkageEncoders();
    Boolean resetting = (Math.abs(linkagePow[0]) > 0.001) || (Math.abs(linkagePow[1]) > 0.001);
    if (!resetting){
      linkagePow = Intake.moveIntakeTo(hopperState);
    }
    Transport.horizontalTransportMotor.set(TalonSRXControlMode.PercentOutput, Transport.horizontalPower);
    Transport.upperVerticalTransportMotor.set(TalonSRXControlMode.PercentOutput, -Transport.upperVerticalPower);
    Transport.lowerVerticalTransportMotor.set(Transport.lowerVerticalPower);
    Intake.intakeRollerMotor.set(rollerPow);
    Intake.motorList[0].set(linkagePow[0]);
    Intake.motorList[1].set(linkagePow[1]);
    Driver_Controller.SwerveControlSet(true);
  }

  @Override
  public void autonomousExit() {}

  public static Integer activeTime(){
    Integer time = Math.toIntExact(Math.round(DriverStation.getMatchTime()));
    String gameData = DriverStation.getGameSpecificMessage(); // alliance with first inactive shift
    if(time < 30){ // endgame
      if (gameData == ""){
        return time+10; // time until transition period ends
      }
      return time; // time until match ends
    }
    if(time < 55){ // last alliance shift
      if(gameData == ""){
        return time-30; // assume it is active, time until endgame
      }else if (gameData.charAt(0) == alliance){
        return time; // time until match ends, because after current alliance shift, it is endgame
      }else{
        return 30-time; // time until endgame, is negative/inactive
      }
    }
    if(time < 80){ // third alliance shift
      if(gameData == ""){
        return 55-time;
      }else if (gameData.charAt(0) == alliance){
        return 55-time; // time until last alliance shift, is negative because is inactive
      }else{
        return time-55; // time until last alliance shift because it is active now
      }
    }
    if(time < 105){ // second alliance shift
      if(gameData == ""){
        return time-80; // assume we won auto ofc
      }else if (gameData.charAt(0) == alliance){
        return time-80;
      }else{
        return 80-time;
      }
    }
    if(time < 130){ // first alliance shift
      if(gameData == ""){
        return 105-time; // assume we won auto ofc
      }else if (gameData.charAt(0) == alliance){
        return 105-time;
      }else{
        return time-105;
      }
    }
    // transition shift
    if(gameData == ""){
      return time-130;
    }else if (gameData.charAt(0) == alliance){
      return time-130;
    }else{
      return time-105;
    }
  }

  
  @Override
  public void teleopInit() {
    Driver_Controller.define_Controller();
    RobotContainer.rotaryCalc(true);
    //Intake.needReset = true;
    Driver_Controller.SwerveControlSet(false);
    alliance = DriverStation.getAlliance().toString().charAt(9);
  }

  @Override
  public void teleopPeriodic() {
    Double[] linkagePow = {0.0, 0.0};
    Double rollerPow = 0.0;
    Double shooterSpeed = Shooter.interpolatePower();
    if (Driver_Controller.buttonUseOffset()) shooterSpeed += 5*(Driver_Controller.lowerDriverSlider());
    Double shooterPow = 0.0;

    if (Driver_Controller.shooterSwitch()){
      shooterPow = Shooter.speedController(shooterSpeed);
    }else{
      if (vision.seenTags || vision2.seenTags){
        LED.setPattern("rainbow");
      }else{
        LED.setPattern("warn");
      }
    }

    if (Driver_Controller.buttonResetIntake() && (!resetLastPressed)){
      Intake.needReset = true;
    }
    resetLastPressed = Driver_Controller.buttonResetIntake();

    if (Driver_Controller.buttonReverseIntake()){
      rollerPow = 1.0;
    } else if (Driver_Controller.switchIntakeRoller() && (Intake.leftLinkageMotor.getRotorPosition().getValueAsDouble() > 10) && (Intake.rightLinkageMotor.getRotorPosition().getValueAsDouble() > 10)){
      rollerPow = -1.0;
    }

    linkagePow = Intake.resetLinkageEncoders();
    Boolean resetting = (Math.abs(linkagePow[0]) > 0.001) || (Math.abs(linkagePow[1]) > 0.001);
    if (!resetting){
      
      if (Driver_Controller.buttonIntakeIn()){
        linkagePow = Intake.moveIntakeTo("retract");
      }else if (Driver_Controller.buttonIntakeOut()){
        linkagePow = Intake.moveIntakeTo("extend");
      }else{
        linkagePow = Intake.moveIntakeTo("neutral");
      }
    }
    Transport.transportPower();

    if (Driver_Controller.buttonEBrake()){
      linkagePow[0] = 0.0;
      linkagePow[1] = 0.0;
      rollerPow = 0.0;
      Transport.horizontalPower = 0.0;
      Transport.upperVerticalPower = 0.0;
      Transport.lowerVerticalPower = 0.0;
      RobotContainer.stopMovement();
      shooterPow = 0.0;
    }else{
      if (Driver_Controller.buttonBumpOrient()){
        Driver_Controller.SwerveControlSet(true);
        Driver_Controller.SwerveCommandEncoderValue = Autonomous.orientBump();
        Double sliderMult = Driver_Controller.upperDriverSlider()*0.7+0.8;
        Driver_Controller.SwerveCommandXValue = -RobotContainer.betterJoystickCurve(Driver_Controller.m_Controller0.getLeftX()-0.02, Driver_Controller.m_Controller0.getLeftY()-0.05)[0]*sliderMult;
        Driver_Controller.SwerveCommandYValue = -RobotContainer.betterJoystickCurve(Driver_Controller.m_Controller0.getLeftX()-0.02, Driver_Controller.m_Controller0.getLeftY()-0.05)[1]*sliderMult;
      }else if (Driver_Controller.buttonAutoOrient()){
        Double odoX = RobotContainer.drivetrain.getState().Pose.getX();
        Double odoY = RobotContainer.drivetrain.getState().Pose.getY();
        Driver_Controller.SwerveControlSet(true);
        if (((alliance == 'B') && (odoX < Autonomous.hubX)) || ((alliance == 'R') && (odoX > Autonomous.centerX*2-Autonomous.hubX))){
          Double hubX = (alliance == 'B')?Autonomous.hubX:(Autonomous.centerX*2-Autonomous.hubX);
          Double angle = Math.toDegrees(Math.atan2(Autonomous.centerY-odoY, hubX-odoX))+2;
          Driver_Controller.SwerveCommandEncoderValue = angle;
        }else{
          if (alliance == 'B'){
            Driver_Controller.SwerveCommandEncoderValue = -180;
          }else Driver_Controller.SwerveCommandEncoderValue = 0;
        }
        
        Double sliderMult = Driver_Controller.upperDriverSlider()*0.7+0.8;
        Driver_Controller.SwerveCommandXValue = -RobotContainer.betterJoystickCurve(Driver_Controller.m_Controller0.getLeftX()-0.02, Driver_Controller.m_Controller0.getLeftY()-0.05)[0]*sliderMult;
        Driver_Controller.SwerveCommandYValue = -RobotContainer.betterJoystickCurve(Driver_Controller.m_Controller0.getLeftX()-0.02, Driver_Controller.m_Controller0.getLeftY()-0.05)[1]*sliderMult;
      }else{
        Driver_Controller.SwerveControlSet(false);
      }
    }
    Intake.intakeRollerMotor.set(rollerPow);
    Intake.motorList[0].set(linkagePow[0]);
    Intake.motorList[1].set(linkagePow[1]);
    Transport.horizontalTransportMotor.set(TalonSRXControlMode.PercentOutput, Transport.horizontalPower);
    Transport.upperVerticalTransportMotor.set(TalonSRXControlMode.PercentOutput, -Transport.upperVerticalPower);
    Transport.lowerVerticalTransportMotor.set(Transport.lowerVerticalPower);
    Shooter.leftShooter.set(shooterPow);
    Shooter.rightShooter.set(shooterPow);
    if (System.nanoTime()%(500*1000*1000) < (20*1000*1000)){
      System.out.println(shooterSpeed);
      System.out.println(Shooter.dist);
      System.out.println(Math.abs(Shooter.rightShooter.getVelocity().getValueAsDouble())-shooterSpeed);
      System.out.println();
    }
    // if (Driver_Controller.m_Controller2.getRawButton(1))Intake.leftLinkageMotor.set(0.2); else
    // if (Driver_Controller.m_Controller2.getRawButton(2))Intake.leftLinkageMotor.set(-0.2); else Intake.leftLinkageMotor.set(0.0);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}