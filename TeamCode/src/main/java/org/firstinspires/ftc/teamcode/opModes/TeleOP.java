package org.firstinspires.ftc.teamcode.opModes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import java.util.function.Supplier;
@Configurable
@TeleOp (name = "TeleOP")
public class TeleOP extends LinearOpMode {
    @IgnoreConfigurable
    static PoseHistory poseHistory;

    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    LimeLight limelight;
    Shooter shooter;
    Intake intake;
    Turret turret;
    Blocker blocker;
    OpModeStorage variables;
    public static boolean autoDrive;
    double closeZoneVelo = 1150;
    double farZoneVelo = 1500;
    double driveDivisor = 2;
    public static double kp;
    public static double ks;
    public static double kv;
    int turretPos = 0;




    @Override
    public void runOpMode() {
        ElapsedTime loopTimer = new ElapsedTime();
        variables = new OpModeStorage();
        follower = Constants.createFollower(hardwareMap);
        limelight = new LimeLight(hardwareMap, variables.getAlliance());
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, variables.getAlliance());
        blocker = new Blocker(hardwareMap);
        variables = new OpModeStorage();
        follower.setStartingPose(variables.getPose());
        follower.startTeleopDrive();
        limelight.setPose(follower.getPose());
        variables.setIfAutoDrive(false);
        kp = OpModeStorage.kp;
        kv = OpModeStorage.kv;
        ks = OpModeStorage.ks;

        Command stayHere = hold(follower);
        Command automaticShootAndHold = sequential(
                shooter.interpLUTVelo(turret.getDistanceToGoal()),
                hold(follower),
                blocker.unblock(),
                intake.on(),
                waitMs(2000),
                intake.off(),
                blocker.block()
        );
        waitForStart();
        while (opModeIsActive()) {
            Scheduler.execute();
            limelight.run();
            turret.run(follower.getPose());
            shooter.run();
            intake.periodic();

//        if (shooter.getError() > - 50 && shooter.getError() < 50 && shooter.getTargetVelo() > 0){
//            if (!gamepad1.isRumbling()){gamepad1.rumble(100);}
//            if (!gamepad2.isRumbling()){gamepad2.rumble(100);}
//        }

            shooter.setPIDFCoeffs(kp, 0, 0, 0);
            shooter.setFeedforward(ks, kv, 0);
            limelight.setPose(follower.getPose());
            if (limelight.canRelocalize()) {
                follower.setPose(new Pose(limelight.getPoseFromLimelight().getX(), limelight.getPoseFromLimelight().getY(), limelight.getPoseFromLimelight().getHeading()));
            }
            follower.setTeleOpDrive(-gamepad1.left_stick_y / driveDivisor, -gamepad1.left_stick_x / driveDivisor, -gamepad1.right_stick_x / driveDivisor, true);

            if (gamepad1.leftBumperWasPressed()) {schedule(intake.on());}
            if (gamepad1.leftBumperWasReleased()) {schedule(intake.off());}
            if (gamepad1.triangleWasPressed()) {schedule(blocker.block());}
            if (gamepad1.circleWasPressed()) {schedule(blocker.unblock());}
            if (gamepad1.dpadUpWasPressed()) {schedule(shooter.interpLUTVelo(turret.getDistanceToGoal()));}
            if (gamepad1.dpadLeftWasPressed()) {schedule(shooter.setVelo(1100));}
            if (gamepad1.dpadRightWasPressed()) {schedule(shooter.setVelo(1550));}
            if (gamepad1.dpadDownWasPressed()) {schedule(shooter.off());}
            if (gamepad1.squareWasPressed()) {turret.startTracking();}
            if (gamepad1.squareWasReleased()) {turret.stopTracking();}


            follower.update();
            if (!turret.isTracking()) {
                turret.TurretSetPos(0);
            }
            Tuning.drawCurrent();
            //turret.startTracking();
            telemetryData.addData("--------------------------", "");
            telemetryData.addData("OPMODE TELEMETRY", "");
            telemetryData.addData("Alliance", (variables.getAlliance() == Alliance.RED) ? "RED" : "BLUE");
            telemetryData.addData("Loop Time", loopTimer.milliseconds());
            telemetryData.addData("--------------------------", "");
            telemetryData.addData("DRIVETRAIN TELEMETRY", "");
            telemetryData.addData("X", follower.getPose().getX());
            telemetryData.addData("Y", follower.getPose().getY());
            telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetryData.addData("--------------------------", "");
            telemetryData.addData("SHOOTER TELEMETRY", "");
            telemetryData.addData("Shooter CURRENT speed", shooter.getCurrentVelo());
            telemetryData.addData("Shooter TARGET speed", shooter.getTargetVelo());
            telemetryData.addData("Shooter POWER", shooter.getShooterPower());
            telemetryData.addData("--------------------------", "");
            telemetryData.addData("TURRET TELEMETRY", "");
            telemetryData.addData("turret pos in ticks", turret.getPos());
            telemetryData.addData("turret angle", turret.getTurretToGoalAngle());
            telemetryData.addData("turret distance", turret.getDistanceToGoal());
            telemetryData.addData("turret reached", turret.reachedTarget());
            telemetryData.addData("--------------------------", "");
            telemetryData.addData("LIMELIGHT TELEMETRY", "");
            telemetryData.addData("canRelocalise", limelight.canRelocalize());
            if (limelight.canRelocalize()) {
                telemetryData.addData("limelight x", limelight.getPoseFromLimelight().getX());
                telemetryData.addData("limelight y", limelight.getPoseFromLimelight().getY());
                telemetryData.addData("limelight heading", Math.toDegrees(limelight.getPoseFromLimelight().getHeading()));
            }
            telemetryData.addData("--------------------------", "");

            telemetryData.update();
            loopTimer.reset();
        }
    }


}