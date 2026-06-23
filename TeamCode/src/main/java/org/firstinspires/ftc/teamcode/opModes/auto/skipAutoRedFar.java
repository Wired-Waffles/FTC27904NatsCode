package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.OpModeStorage.pose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Reset Pose Red Far Zone")
public class skipAutoRedFar extends LinearOpMode {
    private Follower follower;
    Alliance alliance = Alliance.RED;

    Pose startPose = new Pose(88, 8, Math.toRadians(90));


    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        pose = startPose;
        waitForStart();
        OpModeStorage.alliance = this.alliance;
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            // Feedback to Driver Hub for debugging
            telemetry.addData("Pose Reset Complete", "Please now go to TeleOP >:D");
        }

    }
}
