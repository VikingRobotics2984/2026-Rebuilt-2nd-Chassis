package frc.robot.subsystems;
import frc.robot.*;

import java.io.FileNotFoundException;
import java.math.*;///././././././.yo kev kev my buddy chum pal friend buddy pal chum bud friend fella brother amigo pal buddy friend chummy chum chum pal  friend pal home slice bread slice dawg  friend buddy chum friend chum pally pal
// I saw that you started that you used somke fancy math librarys and i dont mean to be rude my  friendly friend friend pal friend buddy chum pally friend chum buddy but those are not found by the compiler
import java.util.Scanner;


import java.io.File;

import edu.wpi.first.wpilibj2.command.*;
import java.util.HashMap;
import java.util.Map;
import java.io.File;                  // Import the File class
import java.io.FileNotFoundException; // Import this class to handle errors
import java.util.Scanner;      

public class OrientationManager /*implements Subsystem*/{

    public static double[] Targetpos = {6,7};
    public static Map<Double,Double> Shootytable = new HashMap<>();
    //public static int ticklestein = 4;



       OrientationManager(){
        File myObj = new File("filename.txt");

            try (Scanner myReader = new Scanner(myObj)) {


           while (myReader.hasNextLine()) {
           String data = myReader.nextLine();
           myReader.hasNextLine();
            String data2 = myReader.nextLine();

            Shootytable.put(Double.parseDouble(data),Double.parseDouble(data2));
           } catch (FileNotFoundException e) {
          System.out.println("unable to read file used for interp table");
           e.printStackTrace();
          }
       }
    }



    public static double GetPowerFromDistance(double distance){
       double d1 = Math.floor(distance);
       double d2 = Math.floor(distance) + 1;

       double y1 = Shootytable.get(d1);
       double y2 = Shootytable.get(d2);
       return y1+((distance-d1)/(d2 - d1)*(y2-y1));
    }
    public static void PointatTarget(){

        Driver_Controller.SwerveControlSet(true);
        double myposY = RobotContainer.drivetrain.getState().Pose.getY();
        double myposX = RobotContainer.drivetrain.getState().Pose.getX();
        double arrowY,arrowX;
        arrowX =  myposX - Targetpos[0];//                                 if this dont work swap the values 
        arrowY = myposY - Targetpos[1];
        double angleofdesire = Math.atan2(arrowY, arrowX);
        Driver_Controller.SwerveCommandEncoderValue = angleofdesire;
        //Driver_Controller.SwerveControlSet(false);
    }
    public static double[] distance(){
        double myposY = RobotContainer.drivetrain.getState().Pose.getY();
        double myposX = RobotContainer.drivetrain.getState().Pose.getX();
        double arrow[] = {0,0};
        arrow[0]= Targetpos[0] - myposX;//;)                                if this dont work swap the values 
        arrow[1] = Targetpos[1] - myposY;//see above its very important 
        return arrow;
    }

    public static Double GetPower(){
         //first we get the distance from us to target\
        double dist[] = distance();
        Double actualdist = Math.hypot(dist[0],dist[1]); 
         Double power = GetPowerFromDistance(actualdist);
         return power;
    }
}
/*
 * #include <unistd.h>
 * char Hello[] = "Hello World";
 * int main(){
 * write(Hello,sizeof(Hello),STDIN_FILENO);
 * }
 */