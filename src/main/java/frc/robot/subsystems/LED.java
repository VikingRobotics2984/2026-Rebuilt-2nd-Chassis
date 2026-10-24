package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SerialPort;

public class LED {
    public static SerialPort arduino = null;
    public static Boolean working = false;
    public static final Boolean printPattern = false;

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
    public static char curDistance = '0';

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
                System.out.println("UH OH leds failed");
            }
        }
    }

    public static void setDistance(char newDistance){
        if (newDistance >= 'A' && newDistance <= 'Z'){ // if is a capital letter
            newDistance += 'a'-'A'; // change to lowercase
        }
        if (newDistance >= 'a' && newDistance <= 'z'){ // if is a latter
            if (newDistance == curDistance){ // don't send the same pattern multiple times
                return;
            }
            curDistance = newDistance;
            if (printPattern){
                System.out.println(newDistance);
            }
            try{
                arduino.writeString(""+newDistance);
            }catch(Exception e){
                System.out.println("UH OH leds failed");
            }
        }
    }

    public static void setPattern(String newPattern){
        switch(newPattern){
            case "rainbow":
            case "Rainbow":
                setPattern('w'); // rainbow pattern
                break;
            case "Error":
            case "error":
                setPattern('e'); // solid red
                break;
            case "Warn":
            case "warn":
            case "Warning":
            case "warning":
                setPattern('a'); // solid red
                break;
        }
    }
}
