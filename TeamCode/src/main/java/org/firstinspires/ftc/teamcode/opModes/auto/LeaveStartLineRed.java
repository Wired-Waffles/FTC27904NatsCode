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
import org.firstinspires.ftc.teamcode.subsystems.ServoTurret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Red far leave the line", group = "Leave line")
public class LeaveStartLineRed extends LinearOpMode {

    private Follower follower;
    Alliance alliance = Alliance.RED;
    Pose startPose = new Pose(88, 7, Math.toRadians(90));
    Pose endPose = new Pose(88, 24, Math.toRadians(90));
    PathChain startToShoot, pickUpMiddleSpikeAndOpenGate, pickUpHumanPlayer, pickUpCloseSpike, pickUpSpillage, leave;



    Intake intake;
    Shooter shooter;
    ServoTurret turret;

    public void buildPaths(){
        leave = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public Command autoRoutine() {
        return sequential(
                //preload
                //leave
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
