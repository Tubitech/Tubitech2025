package frc.robot;

public class RobotState {
    public boolean isAlgIn = false;
    public boolean isCoralIn = false;
    public String coralDirection = "v";

    public enum State1{
        al,
        koy
    }

    public enum State2{
        coral,
        alg
    }

    public enum State3{
        sol,
        sag
    }

    public State1 alkoy = State1.al;
    public State2 coralg = State2.coral;
    public State3 yon = State3.sol;


    public boolean isCoralAvailable(){
        return isCoralIn;
    }
    public boolean isAlgAvailable(){
        return isAlgIn;
    }

    public int getint(){
        int a = 0;
        if(alkoy == State1.koy) a += 1;
        if(coralg == State2.coral) a += 2;
        if(a == 3 && yon == State3.sag) a += 1;
        return a;
        /*int magicallist[][] = { // also the line number in states.txt
            {1, 10, -1, -1},
             { 2, 6, -1, -1 },
              { 3, 4, 5, -1},
               { -1, -1, -1, -1 },
               { -1, -1, -1, -1 },
               { -1, -1, -1, -1 },
              { 7, 8, 9, -1 },
               { -1, -1, -1, -1 },
               { -1, -1, -1, -1 },
               { -1, -1, -1, -1 },
             { 11, 22, -1, -1},
              { 12, 17, -1, -1 },
               { 13, 14, 15, 16 },
                { -1, -1, -1, -1 }, 
                { -1, -1, -1, -1 }, 
                { -1, -1, -1, -1 },
                { -1, -1, -1, -1 }, 
               { 18, 19, 20, 21 },
                { -1, -1, -1, -1 }, 
                { -1, -1, -1, -1 }, 
                { -1, -1, -1, -1 }, 
                { -1, -1, -1, -1},
              { 23, 24, -1, -1},
               { -1, -1, -1, -1},
               { -1, -1, -1, -1} };
        int pnt = 0;
        if (alkoy == State1.al){
            pnt = magicallist[pnt][0];
        }else if (alkoy == State1.koy){
            pnt = magicallist[pnt][1];
        }
        if (coralg == State2.coral){
            pnt = magicallist[pnt][0];
        }else if (coralg == State2.alg){
            pnt = magicallist[pnt][1];
        }
        if (special == State3.durum1){
            pnt = magicallist[pnt][0];
        }else if (special == State3.durum2){
            pnt = magicallist[pnt][1];
        }else if (special == State3.durum3){
            pnt = magicallist[pnt][2];
        }
        if (level == State4.L1){
            pnt = magicallist[pnt][0];
        }else if (level == State4.L2){
            pnt = magicallist[pnt][1];
        }else if (level == State4.L3){
            pnt = magicallist[pnt][2];
        }else if (level == State4.L4){
            pnt = magicallist[pnt][3];
        }
        return pnt;*/
    }
}
