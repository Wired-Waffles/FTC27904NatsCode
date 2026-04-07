package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeKill extends CommandBase {
    Intake intake;
    public IntakeKill(Intake intake){
        this.intake = intake;
        addRequirements(this.intake);
    }
    @Override
    public void initialize(){
        intake.kill();
        intake.closeStopper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
