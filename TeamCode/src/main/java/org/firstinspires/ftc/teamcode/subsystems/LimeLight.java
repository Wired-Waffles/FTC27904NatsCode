package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Alliance;

public class LimeLight extends SubsystemBase {
    Limelight3A limelight;
    Pose pose;
    Pose3D limelightPose;
    LLResult res;
    Alliance alliance;
    public LimeLight(HardwareMap hardwareMap, Alliance alliance){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(2);
        limelight.start();
        this.alliance = alliance;

    }


    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        this.res = result;
        //double robotYaw = pose.getHeading();
        //limelight.updateRobotOrientation(robotYaw);
        /*
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                FtcDashboard.getInstance().getTelemetry().addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }*/
        if (result != null && result.isValid()) {
            Pose3D botpose_mt1 = result.getBotpose();
            if (botpose_mt1 != null) {
                double x = botpose_mt1.getPosition().x;
                double y = botpose_mt1.getPosition().y;
                FtcDashboard.getInstance().getTelemetry().addData("MT1 Location", "(" + x + ", " + y + ")");
                limelightPose = botpose_mt1;
            }
        }
    }

    public void setPose(Pose pose){
        this.pose = pose;
    }
    public Pose getPoseFromLimelight(){
        return new Pose(limelightPose.getPosition().x, limelightPose.getPosition().y, limelightPose.getOrientation().getYaw(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
    public void killLimelight(){
        limelight.stop();
    }
    public void startLimelight(){
        limelight.start();
    }
    public boolean canRelocalize(){
        return res != null && res.isValid() && limelightPose != null;
    }
}
