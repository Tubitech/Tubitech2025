package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimbIOHardware implements ClimbIO {
    protected final TalonFX climbMotor = new TalonFX(Constants.ClimbConstants.kClimbTalonCanId.getDeviceNumber(),Constants.ClimbConstants.kClimbTalonCanId.getBus());
    public ClimbIOHardware(){
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Audio.BeepOnBoot = true;
        config.Audio.BeepOnConfig = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        

    }

}
