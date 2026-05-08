package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {
    private MotorEx shooter1;
    private MotorEx shooter2;
    private ServoEx hoodServo;
    InterpLUT speedInterpLUT = new InterpLUT();
    InterpLUT servoPosInterpLUT = new InterpLUT();
    PIDFController shooterPIDF = new PIDFController(0.00001,0,0,0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.065, 0.00046, 0);
    double distance;
    double targetVelo = 0;
    boolean shooting = false;
    double power = 0;

    public void run(){
        FtcDashboard.getInstance().getTelemetry().addData("target velo", targetVelo);
        FtcDashboard.getInstance().getTelemetry().addData("current velo", shooter1.getVelocity());
        FtcDashboard.getInstance().getTelemetry().addData("current draw", shooter1.getCurrent(CurrentUnit.MILLIAMPS));
        if (shooting) {
            power = shooterPIDF.calculate(shooter1.getVelocity(), targetVelo) + feedforward.calculate(targetVelo);
            shooter1.set(power);
            shooter2.set(power);
        } else {
            shooter1.set(0);
            shooter2.set(0);
        }
    }

    public Shooter(final HardwareMap hardwareMap) {
        this.shooter1 = new MotorEx(hardwareMap, "shooter1");
        this.shooter2 = new MotorEx(hardwareMap, "shooter2");
        this.hoodServo = new ServoEx(hardwareMap, "hoodServo");
        shooter1.setRunMode(MotorEx.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter1.setInverted(true);
        //input then output
        //so liek distance then tps ok
        //distace in inches btw
        //speed interplut
        speedInterpLUT.add(10, 850); //untested
        speedInterpLUT.add(20, 900); //untested
        speedInterpLUT.add(30, 970);
        speedInterpLUT.add(40, 1050);
        speedInterpLUT.add(50, 1050);
        speedInterpLUT.add(60, 1150);
        speedInterpLUT.add(70, 1200);
        speedInterpLUT.add(80, 1250);

        speedInterpLUT.add(125, 1500);
        speedInterpLUT.add(136, 1550);

        //hood interplut
        servoPosInterpLUT.add(10, 0);
        servoPosInterpLUT.add(20, 0);
        servoPosInterpLUT.add(30, 0);
        servoPosInterpLUT.add(40, 0);
        servoPosInterpLUT.add(50, 0);
        servoPosInterpLUT.add(60, 0);
        servoPosInterpLUT.add(70, 0);
        servoPosInterpLUT.add(80, 0);

        servoPosInterpLUT.add(125, 0.3);
        servoPosInterpLUT.add(136, 0.3);
    }

    public void velocity(double targetVelocity){
        targetVelo = targetVelocity;
        shooting = true;
    }
    public void hoodPos(double pos){
        hoodServo.set(pos);
    }
    public void setPIDFCoeffs(double kP, double kI, double kD, double kF){
        shooterPIDF.setPIDF(kP,kI,kD,kF);

    }


    public void setFeedforward(double kS, double kV ,double kA){
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }
    public void interpLUTShoot(double dist) {
        velocity(speedInterpLUT.get(dist));
        hoodPos(servoPosInterpLUT.get(dist));
    }
    public void setDistance(double distance){
        this.distance = distance;
    }

    public void reset() {
        shooterPIDF.reset();
    }

    public double getCurrentVelo(){
        return shooter1.getVelocity();
    }
    public double getTargetVelo(){
        return targetVelo;
    }

    public double getShooterPower() {
        return power;
    }

    public double getError(){
        return targetVelo-shooter1.getVelocity();
    }
    public double getkP() {
        return shooterPIDF.getP();
    }

    public double getkV() {
        return feedforward.kv;
    }

    public double getkS() {
        return feedforward.ks;
    }
    public void kill() {
        shooting = false;
    }
    public Command setVelo(double velo) {
        return instant(() -> velocity(velo)).requiring(shooter1, shooter2);
    }
    // beware using this command will use lots of power to slow down the shooter w/ pidf,
    // so basically it'll run backwards until the flywheel is stopped
    // use if stopper is failing, or to stop shooting someone in the head
    public Command hardStop() {
        return instant(() -> velocity(0)).requiring(shooter1, shooter2);
    }
    //sets power to 0 and turn pidf off
    public Command off() {
        return instant(this::kill).requiring(shooter1, shooter2);
    }
    //only use this when testing or just setting calcing interplut with a button
    //ideally run regular method in main loop
    public Command interpLUTVelo(double distance) {
        return instant(() -> interpLUTShoot(distance)).requiring(shooter1, shooter2);
    }

}
