// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Driver_Controller;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Shooter;

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
    Intake.configureCurrentLimit();
    Transport.configureCurrentLimit();
    Constants.Vision.readLayout();
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
    /*if (System.nanoTime()%(500*1000*1000) < (20*1000*1000)){
      for (int i = 0; i < 5; ++i){
        System.out.print(i);
        System.out.print(" ");
        System.out.print(sensors[i].get());
        System.out.println();
      }
    }*/
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}
  
  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

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
  }

  @Override
  public void teleopPeriodic() {
    Double[] linkagePow = {0.0, 0.0};
    Double rollerPow = 0.0;
    Double shooterPow = 0.0;

    if (Driver_Controller.m_Controller3.getRawButton(9)){
      shooterPow = Shooter.speedController(10+25*(1+Driver_Controller.lowerDriverSlider()));
    }

    if (Driver_Controller.buttonResetIntake() && (!resetLastPressed)){
      Intake.needReset = true;
    }
    resetLastPressed = Driver_Controller.buttonResetIntake();

    if (Driver_Controller.buttonReverseIntake()){
      rollerPow = 0.7;
    } else if (Driver_Controller.switchIntakeRoller()){
      rollerPow = -0.7;
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
      Driver_Controller.SwerveControlSet(false);
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
      System.out.println(10+25*(1+Driver_Controller.lowerDriverSlider()));
      System.out.println(Math.abs(Shooter.rightShooter.getVelocity().getValueAsDouble()));
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