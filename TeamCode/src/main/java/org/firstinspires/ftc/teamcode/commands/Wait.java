package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import java.util.concurrent.TimeUnit;

public class Wait extends CommandBase {
    Timing.Timer timer;
    long waitTime;
    public Wait(long waitTimeInSeconds){
        this.waitTime = waitTime;
    }

    @Override
    public void initialize() {
        timer = new Timing.Timer(waitTime, TimeUnit.SECONDS);
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
