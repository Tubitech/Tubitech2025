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

public class ArmSubsystem extends SubsystemBase {
    private final TrapezoidProfile.Constraints constraintsL1 = new TrapezoidProfile.Constraints(10, 10);
    private final ProfiledPIDController controllerL1 = new ProfiledPIDController(1, 1, 1, constraintsL1, 1);
    private final ArmFeedforward feedforwardL1 = new ArmFeedforward(1, 1, 1);

    private final TrapezoidProfile.Constraints constraintsL2 = new TrapezoidProfile.Constraints(10, 10);
    private final ProfiledPIDController controllerL2 = new ProfiledPIDController(1, 1, 1, constraintsL2, 1);
    private final ArmFeedforward feedforwardL2 = new ArmFeedforward(1, 1, 1);

    private final TalonFX motor1 = new TalonFX(3, "Canivore");
    private final TalonFX motor2 = new TalonFX(4, "Canivore");

    private VoltageOut voltage1 = new VoltageOut(0.0);
    private VoltageOut voltage2 = new VoltageOut(0.0);

    private final SparkMax reader1 = new SparkMax(11, MotorType.kBrushed);
    private final SparkMax reader2 = new SparkMax(13, MotorType.kBrushed);

    private final RelativeEncoder encoder1, encoder2;

    private DutyCycleEncoder throughbore1 = new DutyCycleEncoder(0);
    private DutyCycleEncoder throughbore2 = new DutyCycleEncoder(1);


    /*Constants şimdilik burda */
    private final static double L1 = 40 / 2.54; // estimated values, measure later
    private final static double L2 = 29 / 2.54;
    private static final double THETA1_MIN = Math.toRadians(-90.0);
    private static final double THETA1_MAX = Math.toRadians(0.0);
    private static final double THETA2_MIN = Math.toRadians(-120.0);
    private static final double THETA2_MAX = Math.toRadians(120.0);

    public final CommandXboxController driverXbox = new CommandXboxController(0);

    double targetTheta1 = 0, targetTheta2 = 0;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_angle = Radians.mutable(0);
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
    
    
    private double getFirstJointAngle(){
        return throughbore1.get() * ArmConstants.elevgearratio * 2 * Math.PI;  // radians
    }
    private double getSecondJointAngle(){
        return throughbore2.get() * ArmConstants.gripperratio * 2 * Math.PI;  // radians
    }

    private Consumer<Voltage> setVoltage = (Voltage k) -> {
        motor1.setControl(voltage1.withOutput(k.in(Volts)));
    };

    private final SysIdRoutine m_sysIdRoutine;

    public ArmSubsystem() {
        encoder1 = reader1.getEncoder();
        encoder2 = reader2.getEncoder();

        voltage1.EnableFOC = true;
        voltage2.EnableFOC = true;

        motor1.setControl(voltage1);
        motor2.setControl(voltage2);

        m_sysIdRoutine =  new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.1).per(Seconds), Volts.of(3), Seconds.of(2)), 
            new SysIdRoutine.Mechanism(
                setVoltage,
                log -> {
                    log.motor("motor 1")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            motor1.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(getFirstJointAngle() / 2 / Math.PI, Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(encoder1.getVelocity() * ArmConstants.elevgearratio, RotationsPerSecond));
                },
                this));
    }
    
    @Override
    public void periodic(){
        /* // TEST CODE
        double G_FORCE = 0.0;
        double lift = driverXbox.getLeftY() * 3 + G_FORCE;
        System.out.println(getFirstJointAngle());
        if(Math.abs(lift) > 0.03) motor1.setControl(voltage1.withOutput(lift));
        else motor1.setControl(voltage1.withOutput(0));

        if(Math.abs(driverXbox.getRightY()) > 0.03) motor2.setControl(voltage2.withOutput(driverXbox.getRightY() * -3));
        else motor2.setControl(voltage2.withOutput(0));
        */
        setArmPositionPeriodic();
    }


    /*Forward Kinematics */
    private double[] estimatePos(double angle1, double angle2){
        double a = L1 * Math.cos(angle1);
        double b = L1 * Math.sin(angle2);
        double angle3 = angle1 + angle2;
        return new double[] {a + L2 * Math.cos(angle3), b + L2 * Math.sin(angle3)};
    }

    double[] getCurrentGripperPos(){
        return estimatePos(getFirstJointAngle(), getSecondJointAngle());
    }

    boolean compareDV(double[] vector1, double[] vector2) {
        return (Math.abs(vector1[0] - vector2[0]) < 1.0 && Math.abs(vector1[1] - vector2[1]) < 1.0);
    }

    /*Inverse Kinematics */
    private double[] calculateAngles(double dx, double dy){

        double cosTheta2 = (dx * dx + dy * dy - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        double theta2Up = Math.acos(cosTheta2);  // Dirsek yukarıda
        double theta2Down = -Math.acos(cosTheta2);

        double k1Up = L1 + L2 * Math.cos(theta2Up);
        double k2Up = L2 * Math.sin(theta2Up);
        double theta1Up = Math.atan2(dy, dx) - Math.atan2(k2Up, k1Up);

        double k1Down = L1 + L2 * Math.cos(theta2Down);
        double k2Down = L2 * Math.sin(theta2Down);
        double theta1Down = Math.atan2(dy, dx) - Math.atan2(k2Down, k1Down);

        theta1Down = clampAngle(theta1Down, THETA1_MIN, THETA1_MAX);
        theta2Down = clampAngle(theta2Down, THETA2_MIN, THETA2_MAX);

        double[] reachFromUp = estimatePos(theta1Up, theta2Up);
        double[] reachFromDown = estimatePos(theta1Up, theta2Down);
        double[] desiredPosition = {dx, dy};

        if(compareDV(reachFromUp, desiredPosition)) {
            return new double[] {theta1Up, theta2Up};
        } else {
            return new double[] {theta1Down, theta2Down};
        }
    }

    private void setArmPositionPeriodic(){
        setL1Motor();
        setL2Motor();
    }
    
    public boolean isInPosition(){
        return (controllerL1.atGoal() && controllerL2.atGoal());
    }
    private void setL1Motor(){
        double v = controllerL1.calculate(getSecondJointAngle()) + feedforwardL1.calculate(controllerL1.getSetpoint().position, controllerL1.getSetpoint().velocity);
        //motor1.setControl(voltage1.withOutput(v));
    }
    private void setL2Motor(){
        double v = controllerL2.calculate(getSecondJointAngle()) + feedforwardL2.calculate(controllerL2.getSetpoint().position, controllerL2.getSetpoint().velocity);
        //motor2.setControl(voltage2.withOutput(v));
    }
    
    private double clampAngle(double angle, double minAngle, double maxAngle) {
        if (angle < minAngle) {
            angle = minAngle;
        }
        if (angle > maxAngle) {
            angle = maxAngle;
        }
        return angle;
    }

    public void setArm(double x, double y){
        double[] targetAngles = calculateAngles(x, y);
        targetTheta1 = targetAngles[0];
        targetTheta2 = targetAngles[1];

        controllerL1.setGoal(targetTheta1);
        controllerL2.setGoal(targetTheta2);
    }

    public Command setArmPosition(double targetX, double targetY){
        return runOnce(() -> {
          setArm(targetX, targetY);  
        });
    }
    public Command waitUntilArmFinishes(){
        return new WaitUntilCommand(() -> isInPosition());
    }
    public Command setPositionAndWait(double targetX, double targetY){
        return new ParallelCommandGroup(setArmPosition(targetX, targetY), waitUntilArmFinishes());
    }
    /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}