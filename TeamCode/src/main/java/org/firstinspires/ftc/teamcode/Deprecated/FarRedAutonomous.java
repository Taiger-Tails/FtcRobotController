package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Shooter;

@Deprecated
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class FarRedAutonomous extends LinearOpMode {
    Drive Drive = new Drive();
    Shooter Shooter = new Shooter();
    final Constants Constants = new Constants();
    final double MaxDrivePower = 0.7;

    @Override
    public void runOpMode() {
        Shooter.Init(hardwareMap);
        Drive.Init(hardwareMap);

        waitForStart();

        Drive.ResetIMU();

        Drive.DriveFieldRelative(0, 1 * MaxDrivePower, 0);

        sleep(1400);

        Drive.DriveFieldRelative(0, 0, 0);

        Shooter.SetShooterPower(Constants.MAX_SHOOTER_POWER);

        Drive.DriveFieldRelative(0, 0, -1);

        sleep(250);

        Drive.DriveFieldRelative(0, 0, 0);

        sleep(2000);

        for (double i = 1; i <= 3; i++) {
            Shooter.SetServoPower(0.9);

            sleep(500);

            Shooter.SetServoPower(0);

            sleep(1500);
        }

        Shooter.SetShooterPower(0);

        Drive.DriveFieldRelative(0, -1, 0);

        sleep(600);

        Drive.DriveFieldRelative(0, 0, 0);
    }
}
