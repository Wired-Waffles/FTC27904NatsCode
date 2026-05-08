package org.firstinspires.ftc.teamcode.opModes.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.IntakeKill;
import org.firstinspires.ftc.teamcode.commands.IntakeRun;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Stopper tuning")
@Configurable
public class StopperTuning extends CommandOpMode {
    GamepadEx coreDriver, controlPanel;
    Intake intake;
    Turret turret;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static double velocity = 1800;
    public static double openPos = 0;
    public static double closedPos = 0.5;
    Follower follower;
    @Override
    public void initialize() {
        super.reset();
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, follower, Alliance.RED);
        coreDriver = new GamepadEx(gamepad1);
        controlPanel = new GamepadEx(gamepad2);
        Button intakeButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.LEFT_BUMPER
        ).whenPressed(new IntakeRun(intake)).whenReleased(new IntakeKill(intake));
        Button openStopper = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_UP
        ).whenPressed(new InstantCommand(() -> intake.setStopperPos(openPos)));
        Button closeStopper = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_DOWN
        ).whenPressed(new InstantCommand(() -> intake.setStopperPos(closedPos)));

    }

    @Override
    public void run() {
        super.run();
        telemetryM.addData("Stopper pos", intake.getStopperPos());
        telemetryM.addData("Turret Pos", turret.getPos());
        telemetryM.update();
    }

}