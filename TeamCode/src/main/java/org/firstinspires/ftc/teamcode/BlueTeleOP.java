package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;
import org.firstinspires.ftc.teamcode.commands.IntakeEject;
import org.firstinspires.ftc.teamcode.commands.IntakeKill;
import org.firstinspires.ftc.teamcode.commands.IntakeRun;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.Supplier;

@TeleOp (name = "BLUE TEAM")
public class BlueTeleOP extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    LimeLight limelight;
    Shooter shooter;
    Intake intake;
    Turret turret;
    GamepadEx coreDriver;
    GamepadEx controlPanel;
    Pose rampCycleCollectPose;
    Pose alignToRampCyclePose;
    Pose alignToHumanPlayer;
    Pose collectFromHumanPlayer;
    Pose park;
    Pose farZone;
    Pose closeZone;
    Supplier<PathChain> toHumanPlayer, collectViaHumanPlayer, leaveHumanPlayer, toRamp, collectViaRamp, leaveRamp, autoPark, toCloseZone, toFarZone;
    OpModeStorage variables;
    public static boolean autoDrive;
    double closeZoneVelo = 1150;
    double farZoneVelo = 1500;
    double driveDivisor = 2;
    public static double kp = 0.007;
    public static double ks = 0.09;
    public static double kv = 0.0004325;
    int turretPos = 0;




    @Override
    public void initialize() {
        rampCycleCollectPose = new Pose(13, 61, 150);
        alignToRampCyclePose = new Pose(121, 61, 150);
        alignToHumanPlayer = new Pose(120, 10, 0);
        collectFromHumanPlayer = new Pose(134, 10, 0);
        park = new Pose(107, 33, 90);
        farZone = new Pose(60, 12, 110);
        closeZone = new Pose(54,90, 135);
        toHumanPlayer = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, alignToHumanPlayer))
                .setLinearHeadingInterpolation(follower.getHeading(), alignToHumanPlayer.getHeading(), 0.8)
                .build();
        collectViaHumanPlayer = () -> follower.pathBuilder()
                .addPath(new BezierLine(alignToHumanPlayer, collectFromHumanPlayer))
                .setLinearHeadingInterpolation(alignToHumanPlayer.getHeading(), collectFromHumanPlayer.getHeading())
                .build();
        leaveHumanPlayer = () -> follower.pathBuilder()
                .addPath(new BezierLine(collectFromHumanPlayer, alignToHumanPlayer))
                .setLinearHeadingInterpolation(collectFromHumanPlayer.getHeading(), alignToHumanPlayer.getHeading(), 0.8)
                .build();
        toRamp = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, alignToRampCyclePose))
                .setLinearHeadingInterpolation(follower.getHeading(), alignToRampCyclePose.getHeading())
                .build();
        collectViaRamp = () -> follower.pathBuilder()
                .addPath(new BezierLine(alignToRampCyclePose, rampCycleCollectPose))
                .build();
        leaveRamp = () -> follower.pathBuilder()
                .addPath(new BezierLine(rampCycleCollectPose, alignToRampCyclePose))
                .setLinearHeadingInterpolation(rampCycleCollectPose.getHeading(), alignToRampCyclePose.getHeading())
                .build();
        autoPark = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, park))
                .setLinearHeadingInterpolation(follower.getHeading(), park.getHeading())
                .build();
        toCloseZone = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, closeZone))
                .setLinearHeadingInterpolation(follower.getHeading(), closeZone.getHeading())
                .build();
        toFarZone = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, farZone))
                .setLinearHeadingInterpolation(follower.getHeading(), farZone.getHeading())
                .build();





        coreDriver = new GamepadEx(gamepad1);
        controlPanel = new GamepadEx(gamepad2);

        follower = Constants.createFollower(hardwareMap);
        super.reset();
        limelight = new LimeLight(hardwareMap, Alliance.BLUE);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, follower, Alliance.BLUE);
        variables = new OpModeStorage();
        follower.setStartingPose(new Pose(56, 9, 90));
        follower.startTeleopDrive();
        limelight.setPose(follower.getPose());
        variables.setIfAutoDrive(false);

//        Button shootButton = new GamepadButton(
//                controlPanel, GamepadKeys.Button.RIGHT_BUMPER
//        ).whenPressed(
//                new ShootAndHold(shooter, intake, turret, follower, variables)
//        );
        Button intakeButton = new GamepadButton(
                controlPanel, GamepadKeys.Button.LEFT_BUMPER
        ).whenPressed(new IntakeRun(intake)).whenReleased(new IntakeKill(intake));
        Button ejectButton = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_DOWN
        ).whenPressed(new IntakeEject(intake)).whenReleased(new IntakeKill(intake));
//        Button humanPlayerCollectButton = new GamepadButton(
//                controlPanel, GamepadKeys.Button.TRIANGLE
//        ).whenPressed(new CollectFromHuman(toHumanPlayer, collectViaHumanPlayer, leaveHumanPlayer, follower, intake, variables));
//        Button rampCollectButton = new GamepadButton(
//                controlPanel, GamepadKeys.Button.SQUARE
//        ).whenPressed(new CollectFromHuman(toRamp, collectViaRamp, leaveRamp, follower, intake, variables));
//        Button toCloseZoneButton = new GamepadButton(
//                coreDriver, GamepadKeys.Button.DPAD_LEFT
//        ).whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> variables.setIfAutoDrive(true)),
//                new FollowPathCommand(follower, toCloseZone.get()),
//                new InstantCommand(() -> variables.setIfAutoDrive(false))
//        ));
//        Button toFarZoneButton = new GamepadButton(
//                coreDriver, GamepadKeys.Button.DPAD_RIGHT
//        ).whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> variables.setIfAutoDrive(true)),
//                new FollowPathCommand(follower, toFarZone.get()),
//                new InstantCommand(() -> variables.setIfAutoDrive(false))
//        ));
//        Button park = new GamepadButton(
//                controlPanel, GamepadKeys.Button.PS
//        ).whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> variables.setIfAutoDrive(true)),
//                new FollowPathCommand(follower, autoPark.get()),
//                new InstantCommand(() -> variables.setIfAutoDrive(false))
//        ));
//        Button openStopper = new GamepadButton(
//                controlPanel, GamepadKeys.Button.OPTIONS
//        ).whenPressed(new OpenStopper(intake)).whenReleased(new CloseStopper(intake));

        Button interpLUTShoot = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_UP
        ).whenPressed(() -> shooter.interpLUTShoot(turret.distanceToGoal));
        Button basicCloseZoneShoot = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_LEFT
        ).whenPressed(() -> shooter.velocity(closeZoneVelo)).whenPressed(() -> shooter.hoodPos(0));
        Button basicFarZoneShoot = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_RIGHT
        ).whenPressed(() -> shooter.velocity(farZoneVelo)).whenPressed(() -> shooter.hoodPos(0.3));
        Button killShooter = new GamepadButton(
                controlPanel, GamepadKeys.Button.SQUARE
        ).whenPressed(() -> shooter.velocity(0));

        Button fullPowerButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.SHARE
        ).whenPressed(() -> driveDivisor = 5).whenReleased(() -> driveDivisor = 2);


    }

    @Override
    public void run() {
        super.run();
//        if (shooter.getError() > - 50 && shooter.getError() < 50 && shooter.getTargetVelo() > 0){
//            if (!gamepad1.isRumbling()){gamepad1.rumble(100);}
//            if (!gamepad2.isRumbling()){gamepad2.rumble(100);}
//        }
        shooter.setPIDFCoeffs(kp, 0, 0, 0);
        shooter.setFeedforward(ks, kv, 0);
        limelight.setPose(follower.getPose());
        if (limelight.canRelocalize()){
            follower.setPose(new Pose(limelight.getPoseFromLimelight().getX(), limelight.getPoseFromLimelight().getY(), follower.getHeading()));
        }
        follower.setTeleOpDrive(-gamepad1.left_stick_y/driveDivisor, -gamepad1.left_stick_x/driveDivisor, -gamepad1.right_stick_x/driveDivisor, true);

        follower.update();
        turret.TurretSetPos(0);
//        turret.startTracking();

        telemetryData.addData("--------------------------", "");
        telemetryData.addData("DRIVETRAIN TELEMETRY", "");
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
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
            telemetryData.addData("limelight heading", limelight.getPoseFromLimelight().getHeading());
        }
        telemetryData.addData("--------------------------", "");

        telemetryData.update();
    }

}