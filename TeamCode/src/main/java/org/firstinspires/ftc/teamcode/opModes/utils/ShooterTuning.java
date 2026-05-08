package org.firstinspires.ftc.teamcode.opModes.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Shooter PIDF tuning")
@Configurable
public class ShooterTuning extends CommandOpMode {
    GamepadEx coreDriver, controlPanel;
    Shooter shooter;
    Intake intake;
    LimeLight limelight;
    Turret turret;
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static double velocity = 1500;
    public static double hoodPos = 0;

    public static double kp = 0.007;
    public static double ks = 0.09;
    public static double kv = 0.0004325;

    boolean startShooter = false;

    @Override
    public void initialize() {
        super.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        limelight = new LimeLight(hardwareMap, Alliance.RED);
        turret = new Turret(hardwareMap, follower, Alliance.RED);

        coreDriver = new GamepadEx(gamepad1);
        controlPanel = new GamepadEx(gamepad2);

        Button shootButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.RIGHT_BUMPER
        ).whenPressed(
                new InstantCommand(() -> shooter.velocity(velocity))
        );
        Button addMoreVelo = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_UP
        ).whenPressed(new InstantCommand(()->velocity += 100));
        Button addLessVelo = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_DOWN
        ).whenPressed(new InstantCommand(()-> velocity -= 100));
        Button addMoreHoodAngle = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_RIGHT
        ).whenPressed(new InstantCommand(()-> hoodPos+=0.05));
        Button addLessHoodAngle = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_LEFT
        ).whenPressed(new InstantCommand(()-> hoodPos-=0.05));
        Button intakeButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.LEFT_BUMPER
        ).whenPressed(new InstantCommand(() -> intake.run())).whenReleased(new InstantCommand(() -> intake.kill()));
        Button setShooterPosButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.CIRCLE
        ).whenPressed(new InstantCommand(() -> shooter.hoodPos(hoodPos)));
        Button manualTurret = new GamepadButton(
                coreDriver, GamepadKeys.Button.TRIANGLE
        ).whenPressed(new InstantCommand(() -> turret.TurretSetPos(90)));
    follower.startTeleopDrive();
    turret.startTracking();

    }

    @Override
    public void run() {
        super.run();
        shooter.setPIDFCoeffs(kp, 0, 0, 0);
        shooter.setFeedforward(ks, kv, 0);
        limelight.setPose(follower.getPose());
        telemetryData.addData( "shooter velo",shooter.getCurrentVelo());
        telemetryData.addData("target velo", shooter.getTargetVelo());
        telemetryData.addData("kp", kp);
        telemetryData.addData("kv", kv);
        telemetryData.addData("ks", ks);
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Turret distance to goal", turret.getDistanceToGoal());
        telemetryData.addData("Turret angle to goal", turret.getTurretToGoalAngle());
        telemetryData.addData("Turret pos", turret.getPos());
        telemetryData.addData("Robot pos", follower.getPose());
        telemetryM.addData( "shooter velo",shooter.getCurrentVelo());
        telemetryM.addData("target velo", shooter.getTargetVelo());
        telemetryM.addData("kp", kp);
        telemetryM.addData("kv", kv);
        telemetryM.addData("ks", ks);
        telemetryM.addData("power", shooter.getShooterPower());
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        telemetryM.update();
        telemetryData.update();

        if (limelight.canRelocalize()) {
            //follower.setPose(limelight.getPoseFromLimelight());
            follower.setPose(new Pose(limelight.getPoseFromLimelight().getX(), limelight.getPoseFromLimelight().getY(), follower.getHeading()));
        }
        follower.update();
    }

}
