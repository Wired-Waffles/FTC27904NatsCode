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

@Autonomous(name = "Red far gate intake 21", group = "Far zone")
public class RedFarZoneSolo21 extends LinearOpMode {
    private Follower follower;
    Alliance alliance = Alliance.RED;
    double shootWaitMs = 1000;
    double intakeWaitMs = 1000; //only used at human player zone
    double gateIntakeWaitMs = 3000;

    Pose startPose = new Pose(88, 7.5, Math.toRadians(90));
    Pose pickupMiddleSpikePose = new Pose(126, 60);
    Pose pickUpCloseSpikePose = new Pose(133, 36);
    Pose shootPoseOne = new Pose(88, 16);
    Pose shootPoseTwo = new Pose(98, 9, Math.toRadians(180));
    Pose humanPlayerPickupPose = new Pose(132,9);
    Pose gateIntake = new Pose(131, 60, Math.toRadians(27));
    Pose endPose = new Pose(108, 9);

    Intake intake;
    Shooter shooter;
    Turret turret;
    PathChain pickUpCloseSpike,
            changeShootPose,
            goToHumanPlayer,
            humanPlayerToShoot,
            pickUpMiddleSpike,
            toGateIntake,
            returnFromGate,
            leave;


    public void buildPaths(){
        pickUpMiddleSpike = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(88, 60), pickupMiddleSpikePose))
                .addPath(new BezierCurve(pickupMiddleSpikePose, new Pose(88, 60), shootPoseOne))
                .build();
        pickUpCloseSpike = follower.pathBuilder()
                .addPath(new BezierCurve(shootPoseOne, new Pose(88, 35), pickUpCloseSpikePose))
                .addPath(new BezierCurve(pickUpCloseSpikePose, new Pose(88, 35), shootPoseOne))
                .build();
        changeShootPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseOne, shootPoseTwo))
                .setConstantHeadingInterpolation(shootPoseTwo.getHeading())
                .build();
        toGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(shootPoseOne, new Pose(102, 45), gateIntake))
                .build();
        returnFromGate = follower.pathBuilder()
                .addPath(new BezierLine(gateIntake, shootPoseOne))
                .build();
        goToHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseTwo, humanPlayerPickupPose))
                .build();
        humanPlayerToShoot = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerPickupPose, shootPoseTwo))
                .setReversed()
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

                //collect middle spike
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, pickUpMiddleSpike)
                ),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
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
                //thats 9 balls alr
                //gate intake timeeee

                //cycle 1
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, toGateIntake)
                ),
                waitMs(gateIntakeWaitMs),
                follow(follower, returnFromGate),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(shootWaitMs),
                // number 2
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, toGateIntake)
                ),
                waitMs(gateIntakeWaitMs),
                follow(follower, returnFromGate),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(shootWaitMs),
                //number 3
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, toGateIntake)
                ),
                waitMs(gateIntakeWaitMs),
                follow(follower, returnFromGate),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(shootWaitMs),
                //finish with human player
                follow(follower, changeShootPose),
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