package frc.robot.utils;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.dtos.EncoderConfig;

import static frc.robot.Constants.DISTANCE_PER_PULSE;

public class ThroughBoreEncoder {
    private final DutyCycleEncoder dutyCycleEncoder;
    private final Encoder relativeEncoder;

    public ThroughBoreEncoder(EncoderConfig encoderConfig) {
        this(encoderConfig.dutyCycleChannel(), encoderConfig.channelA(), encoderConfig.channelB());
    }

    public ThroughBoreEncoder(int dutyCycleChannel, int channelA, int channelB){
        dutyCycleEncoder = new DutyCycleEncoder(dutyCycleChannel);
        relativeEncoder = new Encoder(channelA, channelB);
        
        relativeEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public int getRelative(){
        return relativeEncoder.get();
    }

    public double getRelativeDist(){
        return relativeEncoder.getDistance();
    }

    public double getAbsolutePos(){
        return dutyCycleEncoder.get();
    }

    public double getAbsolutePosDegree(){
        return dutyCycleEncoder.get()*360;
    }
}