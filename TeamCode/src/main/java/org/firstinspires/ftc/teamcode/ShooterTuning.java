package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.IntakeKill;
import org.firstinspires.ftc.teamcode.commands.IntakeRun;
import org.firstinspires.ftc.teamcode.commands.ShootAndHold;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
@TeleOp(name = "Shooter PIDF tuning")
@Configurable
public class ShooterTuning extends CommandOpMode {
    GamepadEx coreDriver, controlPanel;
    Shooter shooter;
    Intake intake;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static double velocity = 1800;

    public static double kp = 0.000;
    public static double kf = 0.000255;
    @Override
    public void initialize() {
        super.reset();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        coreDriver = new GamepadEx(gamepad1);
        controlPanel = new GamepadEx(gamepad2);

        Button shootButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.RIGHT_BUMPER
        ).whenPressed(
                new InstantCommand(() -> shooter.velocity(velocity))
        );
        Button addMoreKP = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_UP
        ).whenPressed(new InstantCommand(()->kp+= 0.0001));
        Button addSlightlyMoreKP = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_LEFT
        ).whenPressed(new InstantCommand(()->kp+= 0.00001));
        Button addLessKP = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_UP
        ).whenPressed(new InstantCommand(()->kp-= 0.0001));
        Button addSlightlyLessKP = new GamepadButton(
                coreDriver, GamepadKeys.Button.DPAD_LEFT
        ).whenPressed(new InstantCommand(()->kp-= 0.00001));
        Button intakeButton = new GamepadButton(
                coreDriver, GamepadKeys.Button.LEFT_BUMPER
        ).whenPressed(new IntakeRun(intake)).whenReleased(new IntakeKill(intake));
        Button addMoreKF = new GamepadButton(
                coreDriver, GamepadKeys.Button.TRIANGLE
        ).whenPressed(new InstantCommand(()->kf+= 0.0001));
        Button addSlightlyMoreKF = new GamepadButton(
                coreDriver, GamepadKeys.Button.SQUARE
        ).whenPressed(new InstantCommand(()->kf+= 0.00001));
        Button addLessKF = new GamepadButton(
                coreDriver, GamepadKeys.Button.CROSS
        ).whenPressed(new InstantCommand(()->kf-= 0.0001));
        Button addSlightlyLessKF = new GamepadButton(
                coreDriver, GamepadKeys.Button.CIRCLE
        ).whenPressed(new InstantCommand(()->kf-= 0.00001));
        Button killShooter = new GamepadButton(
                coreDriver, GamepadKeys.Button.OPTIONS
        ).whenPressed(new InstantCommand(()-> shooter.velocity(0)));



    }

    @Override
    public void run() {
        super.run();
        shooter.setPIDFCoeffs(kp, 0, 0, kf);
        telemetryData.addData( "shooter velo",shooter.getCurrentVelo());
        telemetryData.addData("target velo", shooter.getTargetVelo());
        telemetryData.addData("kp", kp);
        telemetryData.addData("kf", kf);
        telemetryM.addData( "shooter velo",shooter.getCurrentVelo());
        telemetryM.addData("target velo", shooter.getTargetVelo());
        telemetryM.addData("kp", kp);
        telemetryM.addData("kf", kf);
        telemetryM.addData("power", shooter.getShooterPower());
        telemetryM.update();
    }

}
