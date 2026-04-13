package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Alliance;

@Config
public class Turret extends SubsystemBase {

    MotorEx turret;


    public static double lowerLimit = -110;
    public static double upperLimit = 207.0642;



    double goalX, goalY;


    public static double currentRelativePos;



    double robotToGoalAngle;
    double turretToGoalAngle;

    public double distanceToGoal = 0;

    public double heading;

    public double offsetY;
    public double offsetX;
    double driveGearTeeth = 32.0;
    double bigGearTeeth = 110.0;
    double gearRatio = driveGearTeeth/bigGearTeeth;

    Alliance alliance;
    boolean isTracking = false;

    public void setGoalPos(double x, double y) {
        goalX = x;
        goalY = y;
    }
    Follower follower;

    public Turret(HardwareMap hardwareMap, Follower follower, Alliance alliance) {
        turret = new MotorEx(hardwareMap, "turret", Motor.GoBILDA.RPM_223);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setInverted(true);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        this.follower = follower;
        this.alliance = alliance;
        if (alliance == Alliance.RED) {
            setGoalPos(130, 130);
        } else if (alliance == Alliance.BLUE) {
            setGoalPos(13, 130);
        } else {
            setGoalPos(130, 130);
        }

    }



    @Override
    public void periodic() {
        Pose robotPos = follower.getPose();


        heading = robotPos.getHeading();
        double dx = goalX-robotPos.getX();
        double dy = goalY-robotPos.getY();

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy, dx));//direction
        distanceToGoal = Math.hypot(dx,dy);//magnitude

        turretToGoalAngle =AngleUnit.normalizeDegrees(Math.toDegrees(robotPos.getHeading()) - robotToGoalAngle );

        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal() );
        FtcDashboard.getInstance().getTelemetry().addData("turret drive gear pos", turret.getCurrentPosition());
        if (isTracking) {
            TurretSetPos(turretToGoalAngle);
        }
    }

    public void TurretSetPos(double PosDeg){
        double countPerDegree = turret.getCPR() / 360;

        int turretTargetPos = (int) (PosDeg * (countPerDegree * gearRatio));/*
        if (turretTargetPos > (int) (185 * (countPerDegree * gearRatio))){
            turretTargetPos -= (int) (360 * (countPerDegree * gearRatio));
        } else if (turretTargetPos > (int) (-185 * (countPerDegree * gearRatio))){
            turretTargetPos += (int) (360 * (countPerDegree * gearRatio));
        }*/

        turret.setTargetPosition(turretTargetPos);
    }

    public double getTurretToGoalAngle() {
        return turretToGoalAngle;
    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }
    public void startTracking(){
        isTracking = true;
    }
    public void stopTracking(){
        isTracking = false;
    }
    public boolean reachedTarget(){
        return turret.atTargetPosition();
    }
}