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
    PathChain pickup1, pickup2, pickup3, pickup3ToShoot, pickupReleased, leave;

    Pose startPose = new Pose(56, 8.5, Math.toRadians(180));
    Pose pickup1Pose = new Pose(12, 8.5, Math.toRadians(180));
    Pose shootPose = new Pose(56, 16);
    Pose pickup2Pose = new Pose(17, 36);
    Pose pickup3Pose = new Pose(20, 60);
    Pose pickupReleasedPose = new Pose(12, 40);
    Pose openGateNoIntakePose = new Pose(17, 67, 180);
    Pose endPose = new Pose(56, 27);
    Intake intake;
    Shooter shooter;
    Turret turret;

    public void buildPaths(){
        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pickup1Pose, shootPose))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(54, 37), pickup2Pose))
                .addPath(new BezierCurve(pickup2Pose, new Pose(54, 37), shootPose)).setReversed()
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(54, 60), pickup3Pose))
                .addPath(new BezierCurve(pickup3Pose, new Pose(26, 66), openGateNoIntakePose))
                .build();
        pickup3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(openGateNoIntakePose, shootPose)).setReversed()
                .build();
        pickupReleased = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(55, 41), pickupReleasedPose))
                .addPath(new BezierCurve(pickupReleasedPose, new Pose(55, 41), shootPose)).setReversed()
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .build();
    }

    public Command autoRoutine() {
        return sequential(
                instant(() -> turret.startTracking()),
                //preload
                shooter.setVelo(1550),
                intake.stopperOpen(),
                waitMs(1500),
                intake.transfer(),
                waitMs(1000),
                intake.stopperClose(),
                intake.off(),
                //one
                follow(follower, pickup1),
                shooter.setVelo(1500),
                intake.stopperOpen(),
                waitMs(1000),
                intake.stopperClose(),
                //two
                follow(follower, pickup2),
                shooter.setVelo(1500),
                intake.stopperOpen(),
                waitMs(1000),
                intake.stopperClose(),
                //three
                follow(follower, pickup3),
                follow(follower, pickup3ToShoot),
                shooter.setVelo(1500),
                intake.stopperOpen(),
                waitMs(1000),
                intake.stopperClose(),
                //pickupLeftovers
                follow(follower, pickupReleased),
                shooter.setVelo(1500),
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
            //telemetry.addData("Current path distance remaining", follower.getCurrentPath().getDistanceRemaining());
            //telemetry.addData("Path number", follower.getCurrentPathNumber());
            telemetry.update();
            pose = follower.getPose();
        }

    }
}
