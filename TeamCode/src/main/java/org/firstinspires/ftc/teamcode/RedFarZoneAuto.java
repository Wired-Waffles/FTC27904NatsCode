package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeKill;
import org.firstinspires.ftc.teamcode.commands.IntakeRun;
import org.firstinspires.ftc.teamcode.commands.ShootAllBalls;
import org.firstinspires.ftc.teamcode.commands.Wait;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class RedFarZoneAuto extends CommandOpMode {
    Follower follower;
    PathChain pickup1, pickup1ToShoot, pickup2, pickup2ToShoot, pickup3, pickup3ToShoot, leave;
    Pose startPose = new Pose(121, 124





















































































            , 90);
    Pose pickup1Pose = new Pose(134, 9);
    Pose shootPose = new Pose(86, 12, 90);
    Pose pickup2Pose = new Pose(133, 36);
    Pose pickup3Pose = new Pose(133, 60);
    Pose endPose = new Pose(86, 36);
    Intake intake;
    Shooter shooter;
    Turret turret;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    public void buildPaths(){
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(57, 17), pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), 180)
                .build();
        pickup1ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, new Pose(31, 15), shootPose))
                .setLinearHeadingInterpolation(180, shootPose.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(60, 36), pickup2Pose))
                .build();
        pickup2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, shootPose))
                .setLinearHeadingInterpolation(180, shootPose.getHeading())
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(64, 64), pickup3Pose))
                .build();
        pickup3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, shootPose))
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

                //cycllllllleeeeeeeeeeeee threeeeeeeeeee
                new ShootAllBalls(intake, shooter, turret),
                new IntakeRun(intake),
                new FollowPathCommand(follower, pickup3),
                new Wait(1),
                new IntakeKill(intake),
                new FollowPathCommand(follower, pickup3ToShoot),

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
