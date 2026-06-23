package org.firstinspires.ftc.teamcode.opModes.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.teamcode.OpModeStorage.kp;
import static org.firstinspires.ftc.teamcode.OpModeStorage.ks;
import static org.firstinspires.ftc.teamcode.OpModeStorage.kv;
import static org.firstinspires.ftc.teamcode.OpModeStorage.pose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ServoTurret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous (name = "Blue close auto")
public class BlueCloseOne extends LinearOpMode {
    //esther wanted to call this steves lava chicken
    private Follower follower;

    Alliance alliance = Alliance.BLUE;

    Intake intake;
    Shooter shooter;
    ServoTurret turret;

    Pose startPose = new Pose(34, 134, Math.toRadians(90));
    Pose pickUpOneAlign = new Pose(56, 42, Math.toRadians(180));
    Pose pickUpOne = new Pose(13, 42);
    Pose shootPose = new Pose(34, 121, Math.toRadians(180));
    Pose endPos = new Pose(48, 121);

    PathChain alignPickUpClose, pickUpClose, pickUpCloseToShoot, leave;

    public void buildPaths() {
        alignPickUpClose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        pickUpClose = follower.pathBuilder()
                .addPath(new BezierLine(pickUpOneAlign, pickUpOne))
                .setConstantHeadingInterpolation(pickUpOneAlign.getHeading())
                .build();
        pickUpCloseToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickUpOne, shootPose))
                .setLinearHeadingInterpolation(pickUpOneAlign.getHeading(), shootPose.getHeading())
                .build();
        leave = pickUpCloseToShoot = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPos))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
    }

    public Command autoRoutine() {
        return sequential(
                turret.start(),
                shooter.setHoodPos(0.3),
                shooter.setVelo(1080),
                follow(follower, alignPickUpClose),
                //shoot preload
                waitMs(1500),
                intake.stopperOpen(),
                intake.transfer(),
                waitMs(2000),
                //pickup close
//                follow(follower, alignPickUpClose),
//                intake.stopperClose(),
//                intake.transfer(),
//                follow(follower, pickUpClose),
//                waitMs(1000),
//                follow(follower, pickUpCloseToShoot),
//                //shoot pickup close
//                waitMs(1500),
//                intake.stopperOpen(),
//                intake.transfer(),
//                waitMs(2000)
                //leave
                intake.off(),
                follow(follower, leave)
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        turret = new ServoTurret(hardwareMap, alliance);
        intake = new Intake(hardwareMap, telemetry);
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        waitForStart();
        schedule(autoRoutine());
        OpModeStorage.alliance = this.alliance;
        shooter.setPIDFCoeffs(kp, 0, 0, 0);
        shooter.setFeedforward(ks, kv, 0);
        while (opModeIsActive()) {
            follower.update();
            turret.run(follower.getPose());
            shooter.run();
            intake.periodic();

            Scheduler.execute();
            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            if (follower.getCurrentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.getCurrentPath().getDistanceRemaining());
                telemetry.addData("Path number", follower.getCurrentPathNumber());
            }
            telemetry.update();
            pose = follower.getPose();
        }
    }
}