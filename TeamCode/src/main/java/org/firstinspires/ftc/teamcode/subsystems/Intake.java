package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Intake extends SubsystemBase {
    MotorEx intake;
    ServoEx stopper;


    public Intake(HardwareMap hardwareMap){
        intake = new MotorEx(hardwareMap, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setInverted(false);
        stopper = new ServoEx(hardwareMap, "stopper");
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

}
