package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SerialPort;

public class LED {
    public static SerialPort arduino = null;
    public static Boolean working = false;
    public static final Boolean printPattern = false;

    public static final char rainbow = 'w';
    public static final char warning = 'a';
    public static final char error = 'e';

    public static void assignPort(){
        try{
            arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
            working = true;
        }catch(Exception e){
            try{
                arduino = new SerialPort(9600, SerialPort.Port.kUSB);
                working = true;
            }catch(Exception e1){
            }
        }
    }
    public static char curPattern = '0';

    public static void setPattern(char newPattern){
        if (newPattern >= 'A' && newPattern <= 'Z'){ // if is a capital letter
            newPattern += 'a'-'A'; // change to lowercase
        }
        if (newPattern >= 'a' && newPattern <= 'z'){ // if is a latter
            if (newPattern == curPattern){ // don't send the same pattern multiple times
                return;
            }
            curPattern = newPattern;
            if (printPattern){
                System.out.println(newPattern);
            }
            try{
                arduino.writeString(""+newPattern);
            }catch(Exception e){
                if (printPattern)System.out.println("UH OH leds failed");
            }
        }
    }
}
