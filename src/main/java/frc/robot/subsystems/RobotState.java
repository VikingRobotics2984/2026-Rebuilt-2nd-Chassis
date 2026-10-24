// package frc.robot.subsystems;

// import frc.robot.RobotContainer;

// public class RobotState {
//     public Double xPos;
//     public Double yPos;
//     public Double targetAngle;
//     public Double xJoy;
//     public Double yJoy;
//     public Double lowerDriverSlider;
//     public Boolean buttonAutoOrient;
//     public Boolean buttonVisionToggle;
//     public Boolean buttonUseOffset;
//     public Boolean buttonBumpOrient;
//     public Boolean buttonPreloadTransport;
//     public Boolean buttonTransport;
//     public Boolean buttonReverseTransport;
//     public Boolean buttonReverseShooter;
//     public Boolean shooterSwitch;
//     public Boolean autoShoot;
//     public Boolean buttonResetIntake;
//     public Boolean buttonIntakeOut;
//     public Boolean buttonIntakeIn;
//     public Boolean switchIntakeRoller;

//     public int kitchenStove;

//     public RobotState(){
//         lowerDriverSlider = Driver_Controller.lowerDriverSlider();
//         buttonAutoOrient = Driver_Controller.buttonAutoOrient();
//         buttonVisionToggle = Driver_Controller.buttonVisionToggle();
//         buttonUseOffset = Driver_Controller.buttonUseOffset();
//         buttonBumpOrient = Driver_Controller.buttonBumpOrient();
//         buttonTransportReverse = Driver_Controller.buttonTransportReverse();
//         intakeSwitch = Driver_Controller.intakeSwitch();
//         kitchenStove = Driver_Controller.kitchenStove();
//         manualSwitch = Driver_Controller.manualSwitch();
//         pauseTurret = Driver_Controller.pauseTurret();
//         runShooterSwitch = Driver_Controller.runShooterSwitch();
//         targetAngle = Rotary_Controller.RotaryJoystick(Driver_Controller.m_Controller1)+RobotContainer.rotaryOffset;
//         transportSwitch = Driver_Controller.transportSwitch();
//         xJoy = Driver_Controller.SwerveXPassthrough;
//         xPos = RobotContainer.drivetrain.getState().Pose.getX();
//         yJoy = Driver_Controller.SwerveYPassthrough;
//         yPos = RobotContainer.drivetrain.getState().Pose.getY();
//     }

//     public void update(String line){
//         String[] words = line.trim().split("\\s+");
//         if (words.length < 18) return;
//         buttonEBrake = Boolean.getBoolean(words[0]);
//         buttonIntakeReverse = Boolean.getBoolean(words[1]);
//         buttonLimitShuttle = Boolean.getBoolean(words[2]);
//         buttonResetTurret = Boolean.getBoolean(words[3]);
//         buttonRestrictTransport = Boolean.getBoolean(words[4]);
//         buttonShooterReverse = Boolean.getBoolean(words[5]);
//         buttonTransportReverse = Boolean.getBoolean(words[6]);
//         intakeSwitch = Boolean.getBoolean(words[7]);
//         kitchenStove = Integer.parseInt(words[8]);
//         manualSwitch = Boolean.getBoolean(words[9]);
//         pauseTurret = Boolean.getBoolean(words[10]);
//         runShooterSwitch = Boolean.getBoolean(words[11]);
//         targetAngle = Double.parseDouble(words[12]);
//         transportSwitch = Boolean.getBoolean(words[13]);
//         xJoy = Double.parseDouble(words[14]);
//         xPos = Double.parseDouble(words[15]);
//         yJoy = Double.parseDouble(words[16]);
//         yPos = Double.parseDouble(words[17]);
//     }

//     public String getString(){
//         String ret = buttonEBrake + " " + 
//         buttonIntakeReverse + " " + 
//         buttonLimitShuttle + " " + 
//         buttonResetTurret + " " + 
//         buttonRestrictTransport + " " + 
//         buttonShooterReverse + " " + 
//         buttonTransportReverse + " " + 
//         intakeSwitch + " " + 
//         kitchenStove + " " + 
//         manualSwitch + " " + 
//         pauseTurret + " " + 
//         runShooterSwitch + " " + 
//         targetAngle + " " + 
//         transportSwitch + " " + 
//         xJoy + " " + 
//         xPos + " " + 
//         yJoy + " " + 
//         yPos;
//         return ret;
//     }
// }