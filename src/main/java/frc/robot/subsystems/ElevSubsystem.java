package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevSubsystem extends SubsystemBase {
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10, 10);
    private final ProfiledPIDController controller = new ProfiledPIDController(1, 1, 1, constraints, 1);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(1, 1, 1);

    private final TalonFX elev1 = new TalonFX(1, "Canivore");
    private final TalonFX elev2 = new TalonFX(2, "Canivore");

    private VoltageOut voltage1 = new VoltageOut(0.0);
    private VoltageOut voltage2 = new VoltageOut(0.0);

    public final CommandXboxController driverXbox = new CommandXboxController(0);

    double target = 0;


    public ElevSubsystem() {
        
        voltage1.EnableFOC = true;
        voltage2.EnableFOC = true;

        elev1.setControl(voltage1);
        elev2.setControl(voltage2);
    }
    
    @Override
    public void periodic(){
        /* // TEST CODE
        if(driverXbox.getRightTriggerAxis() > 0.03) {
            elev1.setVoltage(-driverXbox.getRightTriggerAxis() * 3);
            elev2.setVoltage(-driverXbox.getRightTriggerAxis() * 3);
        } else {
            elev1.setVoltage(0);
            elev2.setVoltage(0);
        }
        */
        // setElevPositionPeriodic();
    }

    private void setElevPositionPeriodic(){
        setMotor();
    }
    
    public boolean isInPosition(){
        return controller.atGoal();
    }

    public double getPos(){
        return 0; // !!! TODO return a encoder value
    }

    private void setMotor(){
        double v = controller.calculate(getPos()) + feedforward.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity);
        elev1.setControl(voltage1.withOutput(v));
        elev2.setControl(voltage2.withOutput(v));
    }
    
    public void setElev(double x){
        target = x;
        controller.setGoal(target);
    }

    public Command setElevPosition(double targetP){
        return runOnce(() -> {
          setElev(targetP);  
        });
    }
    public Command waitUntilArmFinishes(){
        return new WaitUntilCommand(() -> isInPosition());
    }
    public Command setPositionAndWait(double targetP){
        return new ParallelCommandGroup(setElevPosition(targetP), waitUntilArmFinishes());
    }
}