package frc.robot.subsystems;
import frc.robot.*;
import edu.wpi.first.wpilibj2.command.*;



public class OrientationManager /*implements Subsystem*/{

    public static double[] Targetpos = {6,7};
    //public static int ticklestein = 4;

    public static void PointatTarget(){

        Driver_Controller.SwerveControlSet(true);
        double myposY = RobotContainer.drivetrain.getState().Pose.getY();
        double myposX = RobotContainer.drivetrain.getState().Pose.getX();
        double arrowY,arrowX;
        arrowX = Targetpos[0] - myposX;//                                 if this dont work swap the values 
        arrowY = Targetpos[1] - myposY;
        double angleofdesire = Math.atan2(arrowY, arrowX);
        Driver_Controller.SwerveCommandEncoderValue = angleofdesire;
        //Driver_Controller.SwerveControlSet(false);
    }
    public static double[] distance(){
        double myposY = RobotContainer.drivetrain.getState().Pose.getY();
        double myposX = RobotContainer.drivetrain.getState().Pose.getX();
        double arrow[] = {0,0};
        arrow[0]= Targetpos[0] - myposX;//;)                                if this dont work swap the values 
        arrow[1] = Targetpos[1] - myposY;
        return arrow;
    }

}