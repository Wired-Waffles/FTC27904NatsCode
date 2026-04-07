package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class OpenStopper extends CommandBase {
    Intake intake;
    public OpenStopper(Intake intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.openStopper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
