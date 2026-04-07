package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.commands.ShootAllBalls;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

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

    @Override
    public void initialize() {
        coreDriver = new GamepadEx(gamepad1);
        controlPanel = new GamepadEx(gamepad2);

        follower = Constants.createFollower(hardwareMap);
        super.reset();

        limelight = new LimeLight(hardwareMap, Alliance.BLUE);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, follower, Alliance.BLUE);
        follower.startTeleopDrive();
        limelight.setPose(follower.getPose());
        Button shootButton = new GamepadButton(
                controlPanel, GamepadKeys.Button.RIGHT_BUMPER
        ).whenPressed(new ShootAllBalls(intake, shooter, turret));
        Button intakeButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.LEFT_BUMPER
        ).whenPressed(new IntakeRun(intake)).whenReleased(new IntakeKill(intake));
        Button ejectButton = new GamepadButton(
                controlPanel, GamepadKeys.Button.DPAD_DOWN
        ).whenPressed(new IntakeEject(intake)).whenReleased(new IntakeKill(intake));



    }

    @Override
    public void run() {
        super.run();

        limelight.setPose(follower.getPose());
        if (limelight.canRelocalize()){
            follower.setPose(limelight.getPoseFromLimelight());
        }

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}