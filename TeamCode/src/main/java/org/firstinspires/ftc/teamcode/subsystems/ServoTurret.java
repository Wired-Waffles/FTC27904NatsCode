package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Alliance;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class ServoTurret{

    ServoEx turret1;
    ServoEx turret2;



    public static int lowerLimit = 0;
    public static int upperLimit = 1;



    double goalX, goalY;



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
    Pose robotPos;

    public ServoTurret(HardwareMap hardwareMap, Alliance alliance) {
//        turret = new MotorEx(hardwareMap, "turret", Motor.GoBILDA.RPM_223);
//        turret.setRunMode(Motor.RunMode.PositionControl);
//        turret.setInverted(false);
//        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        turret.resetEncoder();
//        turret.setTargetPosition(0);
//        turret.set(1);
        turret1 = new ServoEx(hardwareMap, "turret1");
        turret2 = new ServoEx(hardwareMap, "turret2");
        turret1.setInverted(true);


        this.alliance = alliance;
        if (alliance == Alliance.RED) {
            setGoalPos(130, 134);
        } else if (alliance == Alliance.BLUE) {
            setGoalPos(13, 134);
        } else {
            setGoalPos(130, 134);
        }

    }


    public void run(Pose pose) {
        robotPos = pose;


        heading = robotPos.getHeading();
        double dx = goalX-robotPos.getX();
        double dy = goalY-robotPos.getY();

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy, dx));
        distanceToGoal = Math.hypot(dy, dx);


        turretToGoalAngle = robotToGoalAngle - Math.toDegrees(robotPos.getHeading());

        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal() );
        FtcDashboard.getInstance().getTelemetry().addData("turret drive gear pos", turret1.get());
        if (isTracking) {
            TurretSetPos(turretToGoalAngle);
        } else {
            TurretSetPos(0);
        }
    }

    public void TurretSetPos(double PosDeg) {
        if (PosDeg > 135) {
            PosDeg = 135;
        } else if (PosDeg < -135) {
            PosDeg = -135;
        }
        double countPerDegree = (double) 1/270;
        double turretTargetPos = (countPerDegree * PosDeg) + 0.5;
        //int turretTargetPos = (int) Math.round((PosDeg / 360) * turret.getCPR() * gearRatio);


//        if (turretTargetPos > (int) (185 * (countPerDegree * gearRatio))){
//            turretTargetPos -= (int) (360 * (countPerDegree * gearRatio));
//        } else if (turretTargetPos > (int) (-185 * (countPerDegree * gearRatio))){
//            turretTargetPos += (int) (360 * (countPerDegree * gearRatio));
//        }


        if (turretTargetPos > upperLimit) {
            turretTargetPos = upperLimit;
        } else if (turretTargetPos < lowerLimit) {
            turretTargetPos = lowerLimit;
        }
        turret1.set(turretTargetPos);
        turret2.set(turretTargetPos);
    }

    public void TurretRAWSetPos(double pos){
        turret1.set(pos);
        turret2.set(pos);
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
    public boolean isTracking(){
        return isTracking;
    }
    public Command start(){
        return instant(this::startTracking);
    }
    public Command stop(){
        return instant(this::stopTracking);
    }
    public double getPos() { return turret1.get(); }

}
