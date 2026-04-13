package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.BlueTeleOP;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.Supplier;

public class CollectFromHuman extends SequentialCommandGroup {
    public CollectFromHuman(Supplier<PathChain> toHuman, Supplier<PathChain> pickup, Supplier<PathChain> backOut, Follower follower, Intake intake, OpModeStorage variable){
        addCommands(
            new InstantCommand(() -> variable.setIfAutoDrive(true)),
            new FollowPathCommand(follower, toHuman.get()),
            new IntakeRun(intake),
            new FollowPathCommand(follower, pickup.get(), true),
            new Wait(1),
            new FollowPathCommand(follower, backOut.get()),
            new IntakeKill(intake),
            new InstantCommand(() -> variable.setIfAutoDrive(false))
        );
        addRequirements(intake);
    }

}
