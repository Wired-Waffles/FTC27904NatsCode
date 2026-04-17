package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.CloseStopper;
import org.firstinspires.ftc.teamcode.commands.CollectFromHuman;
import org.firstinspires.ftc.teamcode.commands.IntakeEject;
import org.firstinspires.ftc.teamcode.commands.IntakeKill;
import org.firstinspires.ftc.teamcode.commands.IntakeRun;
import org.firstinspires.ftc.teamcode.commands.OpenStopper;
import org.firstinspires.ftc.teamcode.commands.ShootAllBalls;
import org.firstinspires.ftc.teamcode.commands.ShootAndHold;
import org.firstinspires.ftc.teamcode.commands.Wait;
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
    double closeZoneVelo = 1800;
    double farZoneVelo = 2500;
    double driveDivisor = 2;




    @Override
    public void initialize() {
        rampCycleCollectPose = new Pose(131, 61, 30);
        alignToRampCyclePose = new Pose(121, 61, 30);
        alignToHumanPlayer = new Pose(120, 10, 0);
        collectFromHumanPlayer = new Pose(136, 10, 0);
        park = new Pose(107, 32, 90);
        farZone = new Pose(89, 10, 70);
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
        follower.startTeleopDrive();
        limelight.setPose(follower.getPose());
        variables.setIfAutoDrive(false);
        Button shootButton = new GamepadButton(
                controlPanel, GamepadKeys.Button.RIGHT_BUMPER
        ).whenPressed(
                new ShootAndHold(shooter, intake, turret, follower, variables)
        );
        Button intakeButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.LEFT_BUMPER
        ).whenPressed(new IntakeRun(intake)).whenReleased(new IntakeKill(intake));
        Button ejectButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_DOWN
        ).whenPressed(new IntakeEject(intake)).whenReleased(new IntakeKill(intake));
        Button humanPlayerCollectButton = new GamepadButton(
                controlPanel, GamepadKeys.Button.TRIANGLE
        ).whenPressed(new CollectFromHuman(toHumanPlayer, collectViaHumanPlayer, leaveHumanPlayer, follower, intake, variables));
        Button rampCollectButton = new GamepadButton(
                controlPanel, GamepadKeys.Button.SQUARE
        ).whenPressed(new CollectFromHuman(toRamp, collectViaRamp, leaveRamp, follower, intake, variables));
        Button toCloseZoneButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_LEFT
        ).whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> variables.setIfAutoDrive(true)),
                new FollowPathCommand(follower, toCloseZone.get()),
                new InstantCommand(() -> variables.setIfAutoDrive(false))
        ));
        Button toFarZoneButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_RIGHT
        ).whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> variables.setIfAutoDrive(true)),
                new FollowPathCommand(follower, toFarZone.get()),
                new InstantCommand(() -> variables.setIfAutoDrive(false))
        ));
        Button park = new GamepadButton(
                controlPanel, GamepadKeys.Button.PS
        ).whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> variables.setIfAutoDrive(true)),
                new FollowPathCommand(follower, autoPark.get()),
                new InstantCommand(() -> variables.setIfAutoDrive(false))
        ));
        Button openStopper = new GamepadButton(
                controlPanel, GamepadKeys.Button.OPTIONS
        ).whenPressed(new OpenStopper(intake)).whenReleased(new CloseStopper(intake));

        Button escape = new GamepadButton(
                controlPanel, GamepadKeys.Button.CROSS
        ).cancelWhenPressed(shooter.getCurrentCommand()).whenPressed(() -> variables.setIfAutoDrive(false));
        Button interpLUTShoot = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_UP
        ).cancelWhenPressed(shooter.getCurrentCommand()).whenPressed(() -> shooter.interpLUTShoot(turret.distanceToGoal));
        Button basicCloseZoneShoot = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_LEFT
        ).whenPressed(() -> shooter.velocity(closeZoneVelo));
        Button basicFarZoneShoot = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_RIGHT
        ).whenPressed(() -> shooter.velocity(farZoneVelo));
        Button killShooter = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_DOWN
        ).cancelWhenPressed(shooter.getCurrentCommand()).whenPressed(() -> shooter.velocity(0));
        Button fullPowerButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.SHARE
        ).whenPressed(() -> driveDivisor = 1).whenReleased(() -> driveDivisor = 2);






    }

    @Override
    public void run() {
        super.run();
        if (shooter.getError() > - 50 && shooter.getError() < 50 && shooter.getTargetVelo() > 0){
            if (!gamepad1.isRumbling()){gamepad1.rumble(100);}
            if (!gamepad2.isRumbling()){gamepad2.rumble(100);}
        }

        shooter.setPIDFCoeffs(OpModeStorage.kp, OpModeStorage.ki,OpModeStorage.kd,OpModeStorage.kf);
        limelight.setPose(follower.getPose());
        if (limelight.canRelocalize()){
            follower.setPose(limelight.getPoseFromLimelight());
        }
        follower.setTeleOpDrive(-gamepad1.left_stick_y/driveDivisor, -gamepad1.left_stick_x/driveDivisor, -gamepad1.right_stick_x/driveDivisor, true);

        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }

}