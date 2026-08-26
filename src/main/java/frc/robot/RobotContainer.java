// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;



import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Driver_Controller;

import frc.robot.subsystems.Rotary_Controller;
import frc.robot.Constants;
//import frc.robot.ArmController;
public class RobotContainer {
    public static boolean wasCommandAngle = false, needToReset = true;
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static double TurnModifier = 0.2;
    private static double rotaryOffset = 0;
    /* Setting up bindings for necessary control of the swerve drive platform */
    public static Double deadbandV = MaxSpeed * 0.1/5, angDeadband = MaxAngularRate * 0.1;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(deadbandV).withRotationalDeadband(angDeadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();    

    public RobotContainer() {

        configureBindings();
        
    }
    final static double pos[] = { -1.0, -0.75, -0.5, -0.1, -0.03, 0, 0.03, 0.1, 0.5, 0.75, 1 };
    final static double pwr[] = { -1, -0.3, -0.1, -0.02, 0, 0, 0, 0.02, 0.1, .3, 1 };

    public static double joystick_curve(double joy) {
        for (int i = 0; i < 10; i++) {
            if ((pos[i] <= joy) && (pos[i + 1] >= joy)) {
                return (((joy - pos[i]) / (pos[i + 1] - pos[i]) * (pwr[i + 1] - pwr[i]) + pwr[i]) * MaxSpeed);
            }
        }
        return (0);
    }

    public static double[] betterJoystickCurve(double x, double y) {
        double radius = Math.sqrt((x * x) + (y * y));
        double angle = Math.atan2(y, x);
        radius = joystick_curve(Math.min(1.0, Math.max(-1.0, radius)));
        double[] returnValue = { Math.sin(angle) * radius, Math.cos(angle) * radius };
        return returnValue;
    }

    public static double rotaryCalc(Boolean resetToRobot){
        double pigeonYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        //pigeonYaw = ((drivetrain.getState().Pose.getRotation().getDegrees()+ 360*1000 + 180)%360);
        double rotaryJoystickInput;    //Define the variable rotaryJoystickInput as a double

        if (Driver_Controller.SwerveCommandControl) {
            rotaryJoystickInput = Driver_Controller.SwerveEncoderPassthrough;
            wasCommandAngle = true;
        } else {
            rotaryJoystickInput = Rotary_Controller.RotaryJoystick(Driver_Controller.m_Controller1); // Get input from the rotary controller (ID from m_controller1)
            if (wasCommandAngle || needToReset || resetToRobot) {
                rotaryOffset = (pigeonYaw + (360 * 1000)) % 360;
                wasCommandAngle = false;
                rotaryOffset = (rotaryOffset - rotaryJoystickInput);
                needToReset = false;
            }

            rotaryJoystickInput = rotaryJoystickInput + rotaryOffset;
        }
        //System.out.println(rotaryJoystickInput);
        
        //Math to calculate the maximum turn speed
        double diff = pigeonYaw - (rotaryJoystickInput);
        double diffmod180 = ((diff + 360*1000 + 180)%360) - 180;
        double powerCurved = -diffmod180;
        powerCurved = Math.max(-45,Math.min(45,powerCurved));
        double angleDiff = ((pigeonYaw + (360 * 1000) + 180) % 180) - (rotaryJoystickInput - 180);
                    
        // Stop turing if within a degree of target
        if((angleDiff <= 0.5) && (angleDiff >= -0.5)){
            return 0;
        }

        //Limit the power to a max of 7
        final Double maxPow = 80.0;
        powerCurved = Math.min(maxPow, Math.max(-maxPow, powerCurved));          
        // if (powerCurved < 8 && powerCurved > 2){
        //     powerCurved = 8;// * ((powerCurved + 3)/ 10);
        // }
        // if (powerCurved > -8 && powerCurved < -2){
        //     powerCurved = -8;// * ((Math.abs(powerCurved) + 3)/ 10);
        // }
        // Return a slightly lowered powercurved value (i.e. Slightly lowered turn speed)
        return powerCurved * 0.1;
    }

    public static void stopMovement(){
        Driver_Controller.SwerveCommandEncoderValue = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        Driver_Controller.SwerveCommandXValue = 0.0;
        Driver_Controller.SwerveCommandYValue = 0.0;
        Driver_Controller.SwerveControlSet(true);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(Driver_Controller.SwerveXPassthrough) // Drive forward with negative Y (forward)
                    .withVelocityY(Driver_Controller.SwerveYPassthrough) // Drive left with negative X (left)
                    .withRotationalRate(rotaryCalc(false)* MaxAngularRate * TurnModifier) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        Driver_Controller.needBrake.whileTrue(drivetrain.applyRequest(() -> brake));
    }
}
