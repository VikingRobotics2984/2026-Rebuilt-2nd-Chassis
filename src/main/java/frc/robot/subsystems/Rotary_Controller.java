package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

public class Rotary_Controller {
    public static double RotaryJoystick(Joystick xboxController){
        int fineAxis = (int)((1-xboxController.getRawAxis(5))*100);
        int coarseAxis = 2400+(int)((1-xboxController.getRawAxis(2))*1200);
            int coarseInterval = coarseAxis/200, closest = 35900;
            for (int i = -1; i <= 1; ++i){
                int newAngle = ((coarseInterval+i)*200+fineAxis);
                if (Math.abs(newAngle-coarseAxis) < Math.abs(closest-coarseAxis))
                    closest = newAngle;
            }
            double angle = (((closest%2400)*3.0/20.0)-180);
            return angle;
    }
}
