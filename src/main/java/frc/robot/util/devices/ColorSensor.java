package frc.robot.util.devices;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensor extends ColorSensorV3{
    private static final I2C.Port port = I2C.Port.kOnboard;
    
    public ColorSensor(Port port) {
        super(port);
        //TODO Auto-generated constructor stub
    }

    

    private static ColorSensorV3 instance;

    public static ColorSensorV3 getInstance() {
        if (instance == null) {
            instance = new ColorSensorV3(port);
        }
        return instance;
    }


}
