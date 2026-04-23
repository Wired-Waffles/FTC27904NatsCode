package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveRobotOrientated;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Red 3 Ball Auto")
public class Red3BallFar extends LinearOpMode {
    MecanumDriveRobotOrientated drive = new MecanumDriveRobotOrientated();
    Turret turret;
    Shooter shooter;
    Follower follower;
    Alliance red = Alliance.RED;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap, follower, red);
        shooter = new Shooter(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            shooter.velocity(1550);
            turret.TurretSetPos(143);
            shooter.hoodPos(0.3);
            Thread.sleep(2000);
            intake.openStopper();
            intake.run();
            Thread.sleep(3000);
            intake.kill();
            Thread.sleep(1000);
            drive.drive(0, -0.2, 0, telemetry);
            Thread.sleep(2000);
            drive.drive(0, 0, 0, telemetry);
            Thread.sleep(30000000);

        }
    }
}
