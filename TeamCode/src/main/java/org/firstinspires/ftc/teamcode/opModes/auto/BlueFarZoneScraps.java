package org.firstinspires.ftc.teamcode.opModes.auto; // make sure this aligns with class location

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.OpModeStorage.kp;
import static org.firstinspires.ftc.teamcode.OpModeStorage.ks;
import static org.firstinspires.ftc.teamcode.OpModeStorage.kv;
import static org.firstinspires.ftc.teamcode.OpModeStorage.pose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Blue partner far zone", group = "Far zone")
public class BlueFarZoneScraps extends LinearOpMode {

    private Follower follower;
    Alliance alliance = Alliance.BLUE;
    long shootWaitMs = 1000;
    long intakeWaitMs = 1000; //only used at human player zone

    Pose startPose = new Pose(56, 8.5, Math.toRadians(180));
    Pose pickUpCloseSpikePose = new Pose(11, 36);
    Pose shootPoseOne = new Pose(56, 16);
    Pose shootPoseTwo = new Pose(46, 9, Math.toRadians(180));
    Pose humanPlayerPickupPose = new Pose(12,9);
    Pose humanPlayerPickupHigherPose = new Pose(12, 13);
    Pose secretTunnelAlignPose = new Pose(8, 13, Math.toRadians(90));
    Pose purgeTunnelArtifactsPose = new Pose (8,40);
    Pose endPose = new Pose(36, 9);

    Intake intake;
    Shooter shooter;
    Turret turret;
    PathChain pickUpCloseSpike,
            changeShootPose,
            goToHumanPlayer,
            humanPlayerToShoot,
            goToHumanPlayerHigher,
            humanPlayerHigherToShoot,
            purgeSecretTunnelFromHuman,
            purgeToShoot,
            leave;


    public void buildPaths(){
        pickUpCloseSpike = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(53, 40), pickUpCloseSpikePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .addPath(new BezierCurve(pickUpCloseSpikePose, new Pose(51, 34), shootPoseOne))
                .setConstantHeadingInterpolation(startPose.getHeading()).setReversed()
                .build();
        changeShootPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseOne, shootPoseTwo))
                .setConstantHeadingInterpolation(shootPoseTwo.getHeading())
                .build();
        goToHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseTwo, humanPlayerPickupPose))
                .build();
        humanPlayerToShoot = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerPickupPose, shootPoseTwo))
                .setReversed()
                .build();
        goToHumanPlayerHigher = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseTwo, humanPlayerPickupHigherPose))
                .setConstantHeadingInterpolation(shootPoseTwo.getHeading())
                .build();
        humanPlayerHigherToShoot = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerPickupHigherPose, shootPoseTwo))
                .setConstantHeadingInterpolation(shootPoseTwo.getHeading())
                .build();
        purgeSecretTunnelFromHuman = follower.pathBuilder()
                .addPath(new BezierCurve(humanPlayerPickupPose, new Pose(17, 11), secretTunnelAlignPose))
                .setLinearHeadingInterpolation(shootPoseTwo.getHeading(), secretTunnelAlignPose.getHeading())
                .addPath(new BezierLine(secretTunnelAlignPose, purgeTunnelArtifactsPose))
                .addPath(new BezierCurve(purgeTunnelArtifactsPose, new Pose(38, 41), shootPoseTwo))
                .setLinearHeadingInterpolation(secretTunnelAlignPose.getHeading(), shootPoseTwo.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseTwo, endPose))
                .setConstantHeadingInterpolation(shootPoseTwo.getHeading())
                .build();

    }


    public Command autoRoutine() {
        return sequential(
                //preload
                instant(turret::startTracking),
                parallel(
                        intake.stopperOpen(),
                        shooter.setVelo(1550),
                        shooter.setHoodPos(0.3)
                ),
                waitMs(1000), //spinup
                intake.transfer(),
                waitMs(shootWaitMs),
                //collect close spike
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, pickUpCloseSpike)
                ),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(shootWaitMs),
                //start scrap pickup
                follow(follower, changeShootPose),
                //pick up from human player
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, goToHumanPlayer)
                ),
                waitMs(intakeWaitMs),
                follow(follower, humanPlayerToShoot),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(shootWaitMs),
                //pick up above human player zone
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, goToHumanPlayerHigher)
                ),
                waitMs(intakeWaitMs),
                follow(follower, humanPlayerHigherToShoot),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(shootWaitMs),
                // human player and purge tunnel
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, goToHumanPlayer)
                ),
                waitMs(intakeWaitMs),
                follow(follower, purgeSecretTunnelFromHuman),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(shootWaitMs),
                follow(follower, leave)
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //DONT MESS WITH THIS YIBO OR ANYONE
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

        //this runs a lot
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
            if (follower.getCurrentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.getCurrentPath().getDistanceRemaining());
                telemetry.addData("Path number", follower.getCurrentPathNumber());
            }
            telemetry.update();
            pose = follower.getPose();
        }

    }
}