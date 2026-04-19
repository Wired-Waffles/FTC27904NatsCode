package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class OpModeStorage {
    boolean isAutoDrive = true;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;


    public boolean isAutoDrive() {
        return isAutoDrive;
    }
    public void setIfAutoDrive(boolean isAutoDrive){
        this.isAutoDrive = isAutoDrive;
    }

}
