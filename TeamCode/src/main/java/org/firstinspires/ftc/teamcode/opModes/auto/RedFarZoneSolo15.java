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

@Autonomous(name = "Red far zone 15 solo", group = "Far zone")
public class RedFarZoneSolo15 extends LinearOpMode {

    private Follower follower;
    Alliance alliance = Alliance.BLUE;
    Pose startPose = new Pose(88, 7, Math.toRadians(90));
    Pose shootPose = new Pose(88, 16);
    Pose pickUpMiddleSpikePose = new Pose(128, 60);
    Pose openGatePose = new Pose(126, 70, Math.toRadians(180));
    Pose pickUpHumanPlayerPose = new Pose(132, 9);
    Pose pickUpCloseSpikePose = new Pose(133, 36);
    Pose pickUpSpillagePose = new Pose(133, 21);
    Pose endPose = new Pose(88, 24, Math.toRadians(90));
    PathChain startToShoot, pickUpMiddleSpikeAndOpenGate, pickUpHumanPlayer, pickUpCloseSpike, pickUpSpillage, leave;



    Intake intake;
    Shooter shooter;
    Turret turret;

    public void buildPaths(){
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .build();
        pickUpMiddleSpikeAndOpenGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(88, 60), pickUpMiddleSpikePose))
                .addPath(new BezierCurve(pickUpMiddleSpikePose, new Pose(133, 58), openGatePose)).setConstantHeadingInterpolation(openGatePose.getHeading())
                .addPath(new BezierCurve(openGatePose, new Pose(88, 60), shootPose)).setReversed()
                .build();
        pickUpHumanPlayer = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(104, 8), pickUpHumanPlayerPose))
                .addPath(new BezierCurve(pickUpHumanPlayerPose, new Pose(104, 8), shootPose)).setReversed()
                .build();
        pickUpCloseSpike = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(89, 36), pickUpCloseSpikePose))
                .addPath(new BezierCurve(pickUpCloseSpikePose, new Pose(89, 36), shootPose)).setReversed()
                .build();
        pickUpSpillage = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(100, 21), pickUpSpillagePose))
                .addPath(new BezierCurve(pickUpSpillagePose, new Pose(100, 21), shootPose)).setReversed()
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public Command autoRoutine() {
        return sequential(
                //preload
                instant(turret::startTracking),
                parallel(
                        intake.stopperOpen(),
                        shooter.setVelo(1500),
                        shooter.setHoodPos(0.3),
                        follow(follower, startToShoot)
                ),
                waitMs(1000),
                intake.transfer(),
                //middle + open gate
                waitMs(1500),
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, pickUpMiddleSpikeAndOpenGate)
                ),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(1500),
                //human player zone
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, pickUpHumanPlayer)
                ),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(1500),

                //close spike
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, pickUpCloseSpike)
                ),
                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(1500),

                //artifacts from the ramp we opened
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, pickUpSpillage)
                ),

                parallel(
                        intake.stopperOpen(),
                        intake.transfer()
                ),
                waitMs(1500),

                //leave
                parallel(
                        intake.on(),
                        intake.stopperClose(),
                        follow(follower, leave)
                )

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
