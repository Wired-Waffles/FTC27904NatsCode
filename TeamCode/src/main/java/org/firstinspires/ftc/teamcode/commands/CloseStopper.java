package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class CloseStopper extends CommandBase {
    Intake intake;
    public CloseStopper(Intake intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.closeStopper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
