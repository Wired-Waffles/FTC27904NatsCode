package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.concurrent.TimeUnit;

public class ShootAllBalls extends CommandBase {
    Intake intake;
    Shooter shooter;
    Turret turret;
    Timing.Timer transferTimer = new Timing.Timer(5, TimeUnit.SECONDS);
    public ShootAllBalls(Intake intake, Shooter shooter, Turret turret){
        this.intake = intake;
        this.shooter = shooter;
        this.turret = turret;
        addRequirements(intake, shooter, turret);
    }

    @Override
    public void initialize() {
        transferTimer.start();
        intake.openStopper();
        shooter.reset();
    }

    @Override
    public void execute() {
        turret.startTracking();
        shooter.interpLUTShoot(turret.getDistanceToGoal());

        if ((shooter.getError() > -50 && shooter.getError() < 50) && turret.reachedTarget()){
            intake.run();
        } else {
            intake.kill();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.kill();
        turret.stopTracking();
        intake.closeStopper();
        shooter.velocity(0);
    }

    @Override
    public boolean isFinished(){
        return transferTimer.done();
    }
}
