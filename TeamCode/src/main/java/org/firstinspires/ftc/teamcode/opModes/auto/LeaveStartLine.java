package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveRobotOrientated;
@Autonomous(name = "Leave the start line and wait")
public class LeaveStartLine extends LinearOpMode {
    MecanumDriveRobotOrientated drive = new MecanumDriveRobotOrientated();
    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drive.drive(0.2, 0, 0, telemetry);
            Thread.sleep(2000);
            drive.drive(0, 0, 0, telemetry);
            Thread.sleep(30000000);

        }
    }
}
