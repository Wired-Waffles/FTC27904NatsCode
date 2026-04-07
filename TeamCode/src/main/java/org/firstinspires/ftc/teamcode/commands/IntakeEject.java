package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeEject extends CommandBase{
        Intake intake;
        public IntakeEject(Intake intake){
            this.intake = intake;
            addRequirements(this.intake);
        }
        @Override
        public void initialize(){
            intake.eject();
            intake.closeStopper();
        }

        @Override
        public boolean isFinished() {
            return true;
        }


}
