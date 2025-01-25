package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private SparkMax spark1 = new SparkMax(0, MotorType.kBrushless);
    private SparkMax spark2 = new SparkMax(0, MotorType.kBrushless);


    private DutyCycleEncoder throughbore1 = new DutyCycleEncoder(0);
    private DutyCycleEncoder throughbore2 = new DutyCycleEncoder(1);

    public ArmSubsystem(){

    }
}
