package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.Supplier;

public class intakeFromRamp extends SequentialCommandGroup {
    public intakeFromRamp(Supplier<PathChain> toRamp, Supplier<PathChain> pickup, Supplier<PathChain> backOut, Follower follower, Intake intake, OpModeStorage variable){
        addCommands(
                new InstantCommand(() -> variable.setIfAutoDrive(true)),
                new FollowPathCommand(follower, toRamp.get()),
                new IntakeRun(intake),
                new FollowPathCommand(follower, pickup.get(), true),
                new Wait(5),
                new FollowPathCommand(follower, backOut.get()),
                new IntakeKill(intake),
                new InstantCommand(() -> variable.setIfAutoDrive(false))
        );
        addRequirements(intake);
    }
}
