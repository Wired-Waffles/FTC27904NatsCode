package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class ShootAndHold extends SequentialCommandGroup {
    public ShootAndHold(Shooter shooter, Intake intake, Turret turret, Follower follower){
        Pose shootPos = follower.getPose();
        addCommands(
                new HoldPointCommand(follower, shootPos, true),
                new ShootAllBalls(intake, shooter, turret)
        );
        addRequirements(shooter, intake, turret);
    }

}
