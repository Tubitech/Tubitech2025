package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedSubsystem extends SubsystemBase {
    public AddressableLED led = new AddressableLED(0);  // TODO set this port
    private static final int ledCount = 4;  // TODO set this led count
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(ledCount);
    int lstval = -1;
    LedSubsystem(){
        led.setLength(buffer.getLength());
        led.start();
    }
    
    @Override
    public void periodic(){
        // 24 olasilik;
        led.setData(buffer);
    }

    public void update(RobotState stat){
        int val = stat.getint();
        if (val == lstval)
            return;
        lstval = val;
        for (int i = 0; i < ledCount; i++)
            buffer.setRGB(i, (val % 3) * 255 / 2, ((val / 3) % 3) * 255 / 2, (val / 9) * 255 / 2);
    }
}
