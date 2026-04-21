package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class Turret extends SubsystemBase {

    MotorEx turret;



    public static int lowerLimit = -625;
    public static int upperLimit = 870;
    public static double turretkP = 0.005;



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
    double gearRatio = bigGearTeeth/driveGearTeeth;

    Alliance alliance;
    boolean isTracking = false;

    public void setGoalPos(double x, double y) {
        goalX = x;
        goalY = y;
    }
    Follower follower;
    Pose robotPos;

    public Turret(HardwareMap hardwareMap, Follower follower, Alliance alliance) {
        turret = new MotorEx(hardwareMap, "turret", Motor.GoBILDA.RPM_223);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setInverted(false);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.resetEncoder();
        turret.setTargetPosition(0);
        turret.set(1);


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
        robotPos = follower.getPose();


        heading = robotPos.getHeading();
        double dx = goalX-robotPos.getX();
        double dy = goalY-robotPos.getY();

        robotToGoalAngle = Math.toDegrees(Math.atan2(dx, dy));


        turretToGoalAngle = robotToGoalAngle - Math.toDegrees(robotPos.getHeading());

        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal() );
        FtcDashboard.getInstance().getTelemetry().addData("turret drive gear pos", turret.getCurrentPosition());
        if (isTracking) {
            TurretSetPos(turretToGoalAngle);
        }
    }

    public void TurretSetPos(double PosDeg){
        double countPerDegree = (turret.getCPR() * gearRatio) / 360.0;

        int turretTargetPos = (int) (PosDeg * countPerDegree);
        /*
        if (turretTargetPos > (int) (185 * (countPerDegree * gearRatio))){
            turretTargetPos -= (int) (360 * (countPerDegree * gearRatio));
        } else if (turretTargetPos > (int) (-185 * (countPerDegree * gearRatio))){
            turretTargetPos += (int) (360 * (countPerDegree * gearRatio));
        }
        */
        if (turretTargetPos > upperLimit) {
            turretTargetPos = upperLimit;
        } else if (turretTargetPos < lowerLimit) {
            turretTargetPos = lowerLimit;
        }

        turret.setTargetPosition(turretTargetPos);
        turret.setPositionCoefficient(turretkP);
        if (!turret.atTargetPosition()) {
            turret.set(1);
        }
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
    public int getPos() {
        return turret.getCurrentPosition();
    }
    public boolean reachedTarget(){
        return turret.atTargetPosition();
    }
}