package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Constants {
    public static final int rightLinkageMotorID = 15;//positive power is extend
    public static final int leftLinkageMotorID = 16;//positive power is extend
    public static final int intakeRollerMotorID = 17;//positive power is intake
    public static final int horizontalTransportMotorID = 18;//positive power is transport to shooter
    public static final int upperVerticalTransportMotorID = 19;//positive power is transport to shooter
    public static final int lowerVerticalTransportMotorID = 20;
    public static final int leftShooterMotorID = 21;//positive power is shooting
    public static final int rightShooterMotorID = 22;//positive power is shooting

    public static final int intakePositionSensorLPort = 0;
    public static final int intakePositionSensorRPort = 8;

    public static class Vision {
        public static final String kCameraName = "FrontCam";
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.33, 0.0, 0.0), new Rotation3d(0, Math.toRadians(-25), 0));

        public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public static final String kCamera2Name = "BackCam";
        public static final Transform3d kRobotToCam2 =
                new Transform3d(new Translation3d(-0.33, -0.229, 0.0), new Rotation3d(0, Math.toRadians(25), Math.toRadians(180)));

        // The layout of the AprilTags on the field
        public static void readLayout(){
            try{
            kTagLayout = new AprilTagFieldLayout("/home/admin/2026-rebuilt-welded.json");
            }catch(IOException e){
                e.printStackTrace();
            }
        }

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final Command TestPath = new PathPlannerAuto("Test Auto");
}
