package frc.robot;

public class RobotState {
    public boolean isAlgIn = false;
    public boolean isCoralIn = false;
    public String coralDirection = "v";

    public boolean isCoralAvailable(){
        return isCoralIn;
    }
    public boolean isAlgAvailable(){
        return isAlgIn;
    }
}
