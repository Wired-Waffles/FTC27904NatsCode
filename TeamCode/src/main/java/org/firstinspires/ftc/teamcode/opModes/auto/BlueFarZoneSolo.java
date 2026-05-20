package org.firstinspires.ftc.teamcode.opModes.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import static org.firstinspires.ftc.teamcode.OpModeStorage.*;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Example Auto", group = "Far zone")
public class BlueFarZoneSolo extends LinearOpMode {

    private Follower follower;
    Alliance alliance = Alliance.BLUE;
    PathChain pickup1, pickup1ToShoot, pickup2, pickup2ToShoot, pickup3, pickup3ToShoot, leave;

    Pose startPose = new Pose(54, 9, Math.toRadians(90));
    Pose pickup1Pose = new Pose(10, 9);
    Pose shootPose = new Pose(56, 12, Math.toRadians(90));
    Pose pickup2Pose = new Pose(9, 36);
    Pose pickup3Pose = new Pose(9, 60);
    Pose endPose = new Pose(56, 36);
    Intake intake;
    Shooter shooter;
    Turret turret;

    public void buildPaths(){
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(57, 17), pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180))
                .build();
        pickup1ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, new Pose(31, 15), shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(60, 36), pickup2Pose))
                .build();
        pickup2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(64, 64), pickup3Pose))
                .build();
        pickup3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .build();
    }

    public Command autoRoutine() {
        return sequential(
                instant(() -> turret.startTracking()),
                shooter.interpLUTVelo(turret.getDistanceToGoal()),
                intake.stopperOpen(),
                waitMs(1500),
                intake.on(),
                waitMs(1000),
                intake.stopperClose(),
                follow(follower, pickup1),
                follow(follower, pickup1ToShoot),
                shooter.interpLUTVelo(turret.getDistanceToGoal()),
                intake.stopperOpen(),
                waitMs(1000),
                intake.stopperClose(),
                follow(follower, pickup2),
                follow(follower, pickup2ToShoot),
                shooter.interpLUTVelo(turret.getDistanceToGoal()),
                intake.stopperOpen(),
                waitMs(1000),
                intake.stopperClose(),
                follow(follower, pickup3),
                follow(follower, pickup3ToShoot),
                shooter.interpLUTVelo(turret.getDistanceToGoal()),
                intake.stopperOpen(),
                waitMs(1000),
                intake.stopperClose(),
                intake.off(),
                instant(() -> turret.stopTracking()),
                follow(follower, leave)
        );
    }



    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, alliance);
        intake = new Intake(hardwareMap, telemetry);
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        waitForStart();
        schedule(autoRoutine());
        OpModeStorage.alliance = this.alliance;
        while (opModeIsActive()) {
            follower.update();
            turret.run(follower.getPose());
            shooter.run();
            intake.periodic();
            shooter.setPIDFCoeffs(kp, 0, 0, 0);
            shooter.setFeedforward(ks, kv, 0);
            Scheduler.execute();
            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Current path distance remaining", follower.getCurrentPath().getDistanceRemaining());
            telemetry.addData("Path number", follower.getCurrentPathNumber());
            telemetry.update();
            pose = follower.getPose();
        }

    }
}
