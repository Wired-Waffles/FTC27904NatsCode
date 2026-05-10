package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Blocker {
    public static double blockPosition = 0.5;
    public static double unblockPosition = 0.7;
    public static double assemblyPosition = 0;

    private final Servo blockerServo;

    public Blocker(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, "stopper");
    }

    public void blockPos() {
        blockerServo.setPosition(blockPosition);
    }

    public void unblockPos() {
        blockerServo.setPosition(unblockPosition);
    }

    public void assembly() {
        blockerServo.setPosition(assemblyPosition);
    }
    public Command block(){
        return instant(this::blockPos).requiring(blockerServo);
    }
    public Command unblock(){
        return instant(this::unblockPos).requiring(blockerServo);
    }
}
