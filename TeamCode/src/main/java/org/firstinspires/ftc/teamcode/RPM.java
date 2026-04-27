package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RPM extends LinearOpMode {
    Shooter Shooter = new Shooter();

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter.Init(hardwareMap);
        int InitialTicks = Shooter.ShooterMotor.getCurrentPosition();

        waitForStart();

        Shooter.MaxShooterPower = 1;
        Shooter.SetShooterPower(1);

        sleep(1000);

        Shooter.SetShooterPower(0);

        while (true) {
            sleep(1);

            telemetry.addData("RPM", (Shooter.ShooterMotor.getCurrentPosition() - InitialTicks) / Shooter.ShooterMotor.getMotorType().getTicksPerRev() * 60);
        }
    }
}
