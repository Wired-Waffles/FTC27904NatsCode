package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.IntakeKill;
import org.firstinspires.ftc.teamcode.commands.IntakeRun;
import org.firstinspires.ftc.teamcode.commands.ShootAllBalls;
import org.firstinspires.ftc.teamcode.commands.Wait;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
@Autonomous(name = "red close")
public class RedCloseZoneAuto extends CommandOpMode {
    Follower follower;
    PathChain startToShoot, pickup1, pickup1ToShoot, pickup2, pickup2ToShoot, pickup3, pickup3ToShoot, leave;
    Pose startPose = new Pose(23, 124, 324);
    Pose pickup1Pose = new Pose(16, 84);
    Pose shootPose = new Pose(60, 84, 180);
    Pose pickup2Pose = new Pose(9, 36);
    Pose endPose = new Pose(60, 60);
    Intake intake;
    Shooter shooter;
    Turret turret;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    public void buildPaths(){
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), 180)
                .build();
        pickup1ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, shootPose))
                .setLinearHeadingInterpolation(180, shootPose.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(60, 60), pickup2Pose))
                .build();
        pickup2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, shootPose))
                .setLinearHeadingInterpolation(180, shootPose.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, follower, Alliance.BLUE);
        buildPaths();
        schedule(
                new FollowPathCommand(follower, startToShoot),
                //cycleee oneeee
                new ShootAllBalls(intake, shooter, turret),
                new IntakeRun(intake),
                new FollowPathCommand(follower, pickup1),
                new Wait(1),
                new IntakeKill(intake),
                new FollowPathCommand(follower, pickup1ToShoot),

                //cycleeeeee twooooooooooooooooooooooo
                new ShootAllBalls(intake, shooter, turret),
                new IntakeRun(intake),
                new FollowPathCommand(follower, pickup2),
                new Wait(1),
                new IntakeKill(intake),
                new FollowPathCommand(follower, pickup2ToShoot),

                //leave that starting line
                new ShootAllBalls(intake, shooter, turret),
                new FollowPathCommand(follower, leave)

        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
