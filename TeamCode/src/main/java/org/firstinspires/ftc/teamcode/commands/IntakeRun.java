package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeRun extends CommandBase {
    Intake intake;
    public IntakeRun(Intake intake){
        this.intake = intake;
        addRequirements(this.intake);
    }
    @Override
    public void initialize(){
        intake.run();
        intake.closeStopper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
