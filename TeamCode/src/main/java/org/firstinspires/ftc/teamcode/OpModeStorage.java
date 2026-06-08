package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class OpModeStorage {
    boolean isAutoDrive = true;
    public static double kp = 0.007;
    public static double ks = 0.09;
    public static double kv = 0.0004325;
    //sotm
    public static double kt = 0.5;
    @IgnoreConfigurable
    public static Alliance alliance = Alliance.BLUE;
    @IgnoreConfigurable
    public static Pose pose = new Pose(56,8, Math.toRadians(90));


    public boolean isAutoDrive() {
        return isAutoDrive;
    }
    public void setIfAutoDrive(boolean isAutoDrive){
        this.isAutoDrive = isAutoDrive;
    }
    public Alliance getAlliance() {
        return alliance;
    }
    public void setAllianceRed(){
        this.alliance = Alliance.RED;
    }
    public void setAllianceBlue(){
        this.alliance = Alliance.BLUE;
    }
    public void exportPose(Pose pose){this.pose = pose;}
    public Pose getPose(){return pose;}

}
