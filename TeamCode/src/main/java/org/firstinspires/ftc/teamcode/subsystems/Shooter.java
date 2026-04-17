package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter extends SubsystemBase {
    private MotorEx shooter1;
    private MotorEx shooter2;
    private ServoEx hoodServo;
    InterpLUT speedInterpLUT = new InterpLUT();
    InterpLUT servoPosInterpLUT = new InterpLUT();
    PIDFController shooterPIDF = new PIDFController(0.1,0,0,0.1);
    double distance;
    double targetVelo;
    double power;


    @Override
    public void periodic(){
        FtcDashboard.getInstance().getTelemetry().addData("target velo", targetVelo);
        FtcDashboard.getInstance().getTelemetry().addData("current velo", shooter1.getVelocity());
        FtcDashboard.getInstance().getTelemetry().addData("current draw", shooter1.getCurrent(CurrentUnit.MILLIAMPS));
    }

    public Shooter(final HardwareMap hardwareMap) {
        this.shooter1 = new MotorEx(hardwareMap, "shooter1");
        this.shooter2 = new MotorEx(hardwareMap, "shooter2");
        this.hoodServo = new ServoEx(hardwareMap, "hoodServo");
        shooter1.setRunMode(MotorEx.RunMode.VelocityControl);
        shooter2.setRunMode(Motor.RunMode.VelocityControl);
        shooter1.setInverted(true);
        shooter1.setVeloCoefficients(1,0,0.1);
        shooter2.setVeloCoefficients(1,0,0.1);
        //input then output
        //so liek distance then tps oki
        //random values so it doesnt like explode rn
        //replace later
        //distace in inches btw
        speedInterpLUT.add(10, 1800);
        speedInterpLUT.add(20, 2000);
        speedInterpLUT.add(30, 2300);
        //etc etc
        servoPosInterpLUT.add(10, 0);
        servoPosInterpLUT.add(20, 0.1);
        servoPosInterpLUT.add(30, 0.3);
    }

    public void velocity(double targetVelocity){
        double output = shooterPIDF.calculate(shooter1.getVelocity(), targetVelocity);
        shooter1.set(output);
        shooter2.set(output);
        //shooter1.setVelocity(targetVelocity);
        //shooter2.setVelocity(targetVelocity);
        targetVelo = targetVelocity;
        power = output;
    }
    public void hoodPos(double pos){
        hoodServo.set(pos);
    }
    public void setPIDFCoeffs(double kP, double kI, double kD, double kF){
        shooterPIDF.setPIDF(kP,kI,kD,kF);
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

}
