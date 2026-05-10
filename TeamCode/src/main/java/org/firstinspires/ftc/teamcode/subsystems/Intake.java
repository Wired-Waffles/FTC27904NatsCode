package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    MotorEx intake;
    ServoEx stopper;
    Telemetry telemetry;
    private boolean slowMode = false;
    private Mode mode = Mode.OFF;
    public static double fastPower = -1;
    public static double slowPower = -1;
    public static double offPower = 0;
    public static double reversePower = 1;
    public static double shortReverseTimeMs = 400;


    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        intake = new MotorEx(hardwareMap, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setInverted(false);
        stopper = new ServoEx(hardwareMap, "stopper");
        this.telemetry = telemetry;
    }

    public void run(){
        intake.set(-1);
    }

    public void eject(){
        intake.set(0.5);
    }

    public void kill(){
        intake.set(0);
    }

    public void Hold(){
        intake.set(-0.3);
    }
    public void openStopper(){
        stopper.set(0.5);
    }
    public void closeStopper(){
        stopper.set(0.7);
    }
    public void setStopperPos(double pos){
        stopper.set(pos);
    }
    public double getStopperPos(){
        return stopper.getRawPosition();
    }

    public Command on() {
        return instant(() -> mode = Mode.ON).requiring(intake);
    }

    public Command off() {
        return instant(() -> mode = Mode.OFF).requiring(intake);
    }

    public Command reverse() {
        return instant(() -> mode = Mode.REVERSE).requiring(intake);
    }

    public Command shortReverse() {
        return reverse().then(waitMs(shortReverseTimeMs)).then(on());
    }

    public Command toggle() {
        return conditional(() -> mode == Mode.OFF, on(), off());
    }
    public void periodic() {
            switch (mode) {
                case ON:
                    intake.set(slowMode ? slowPower : fastPower);
                    break;
                case OFF:
                    intake.set(offPower);
                    break;
                case REVERSE:
                    intake.set(reversePower);
                    break;
            }

            telemetry.addData("Intake Current", intake.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake Velocity", intake.getVelocity());
        };
    }

    enum Mode {
        ON,
        OFF,
        REVERSE
    }

