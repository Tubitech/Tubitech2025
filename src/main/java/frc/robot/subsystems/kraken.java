package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class kraken extends SubsystemBase{
    private TalonFX kraken  = new TalonFX(0);
    public kraken(){

    }
    @Override
    public void periodic(){
        kraken.set(0.5);
        kraken.set(-0.5);
        kraken.getDeviceTemp();
        kraken.getAcceleration();
        kraken.getDutyCycle();
        kraken.getIsProLicensed();
        kraken.getPosition();
    }
}
