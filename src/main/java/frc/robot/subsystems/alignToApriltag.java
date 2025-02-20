package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveDrive;

public class alignToApriltag extends SubsystemBase{
    private RobotContainer robotContainer;
    private SwerveSubsystem drive;
    private String ll1name = "ll1";
    private String ll2name = "ll2";
    
    private int desiredIdForLL1 = 0;
    private int desiredIdForLL2 = 0;


    public alignToApriltag(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
        this.drive = robotContainer.getDrivebase();
    }
    // XXXXXXXX

    double offsetX = 0;
    public double getLL1X(){
        return LimelightHelpers.getTX(ll1name)+offsetX;
    }
    public double getLL2X(){
        return LimelightHelpers.getTX(ll2name)+offsetX;
    }
    private boolean isInDesiredXLL1(){
        double idISee = LimelightHelpers.getFiducialID(ll1name);
        if(idISee != desiredIdForLL1){
            return false;
        }
        if(0<Math.abs(getLL1X())&&Math.abs(getLL1X())<1){
            return true;
        }
        return false;

    }
    private boolean isInDesiredXLL2(){
        double idISee = LimelightHelpers.getFiducialID(ll2name);
        if(idISee != desiredIdForLL2){
            return false;
        }
        if(0<Math.abs(getLL2X())&&Math.abs(getLL2X())<1){
            return true;
        }
        return false;

    }
    private Command alingToApX(int ap, int ll, int offset){
        if(ll==2){
            desiredIdForLL2 = ap;
            return  Commands.run(()->{
                double shouldBeZero = getLL2X();
                offsetX = offset;
                double driveX = shouldBeZero * 0.1;
                drive.drive(new Translation2d(driveX,0), 0, false);
            }).until(()->isInDesiredXLL2());
        }
        else{
            desiredIdForLL1 = ap;
            return Commands.run(()->{
                double shouldBeZero = getLL1X();
                offsetX = offset;
                double driveX = shouldBeZero * 0.1;
                drive.drive(new Translation2d(driveX,0), 0, false);
            }).until(()->isInDesiredXLL1());

        }
    }
    //  YYYYYYYYYYYYYYYYYYYYYY
    private double desiredY = 90;
    public double getLL1Y(){
        return LimelightHelpers.getTA(ll1name);
    }
    public double getLL2Y(){
        return LimelightHelpers.getTA(ll2name);
    }
    private boolean isInDesiredYLL1(){
        double idISee = LimelightHelpers.getFiducialID(ll1name);
        if(idISee != desiredIdForLL1){
            return false;
        }
        if(getLL1Y()>desiredY){
            return true;
        }
        return false;

    }
    private boolean isInDesiredYLL2(){
        double idISee = LimelightHelpers.getFiducialID(ll2name);
        if(idISee != desiredIdForLL2){
            return false;
        }
        if(getLL2Y()>desiredY){
            return true;
        }
        return false;

    }
    private Command alingToApY(int ap, int ll, double desiredY){
        if(ll==2){
            desiredIdForLL2 = ap;
            this.desiredY = desiredY;
            return  Commands.run(()->{
                double shouldBeZero = desiredY- getLL2Y();
                double driveY = shouldBeZero * 0.1;
                drive.drive(new Translation2d(0,driveY), 0, false);
            }).until(()->isInDesiredYLL2());
        }
        else{
            desiredIdForLL1 = ap;
            this.desiredY = desiredY;
            return Commands.run(()->{
                double shouldBeZero = desiredY- getLL1Y();
                double driveY = shouldBeZero * 0.1;
                drive.drive(new Translation2d(0,driveY), 0, false);
            }).until(()->isInDesiredYLL1());

        }
    }
    //  Roooooot

    private double getApRotForLL1(){
        double rot = Vision.getAprilTagPose(desiredIdForLL1, null).getRotation().getRadians();
        return rot;
    }
    private double getApRotForLL2(){
        double rot = Vision.getAprilTagPose(desiredIdForLL2, null).getRotation().getRadians();
        return rot;
    }
    private double getRobotRot(){
        return drive.getPose().getRotation().getRadians();
    }
    private boolean isInDesiredRotLL1(){
        double idISee = LimelightHelpers.getFiducialID(ll1name);
        if(idISee != desiredIdForLL1){
            return false;
        }
        if(0<Math.abs(getApRotForLL1()-getRobotRot())&&Math.abs(getApRotForLL1()-getRobotRot())<1){
            return true;
        }
        return false;
    }
    private boolean isInDesiredRotLL2(){
        double idISee = LimelightHelpers.getFiducialID(ll1name);
        if(idISee != desiredIdForLL1){
            return false;
        }
        if(0<Math.abs(getApRotForLL2()-getRobotRot())&&Math.abs(getApRotForLL2()-getRobotRot())<1){
            return true;
        }
        return false;
    }
    private Command alignToApRot(int ap, int ll){
        if(ll==2){
            desiredIdForLL2 = ap;
            return  Commands.run(()->{
                double shouldBeEq = getApRotForLL2();
                double robotRot = getRobotRot();
                double rot = (robotRot-shouldBeEq) * 0.1;
                drive.drive(new Translation2d(0,0), rot, false);
            }).until(()->isInDesiredRotLL2());
        }
        else{
            desiredIdForLL1 = ap;
            return Commands.run(()->{
                double shouldBeEq = getApRotForLL1();
                double robotRot = getRobotRot();
                double rot = (robotRot-shouldBeEq) * 0.1;
                drive.drive(new Translation2d(0,0), rot, false);
            }).until(()->isInDesiredRotLL1());

        }
    }
    public Command alingXYRot(int ApId, int ll, int offsetXP, double desiredYP){
        return Commands.sequence(alingToApX(ApId, ll, offsetXP),alignToApRot(ApId, ll),alingToApY(ApId, ll, desiredYP)).withTimeout(10);
    }
}
