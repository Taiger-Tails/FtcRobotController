package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Shooter;

@Deprecated
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class CloseAutonomous extends LinearOpMode {
    Drive Drive = new Drive();
    Shooter Shooter = new Shooter();
    final Constants Constants = new Constants();

    final double MaxShooterPower = 0.467;
    final double MaxDrivePower = 0.67;

    @Override
    public void runOpMode() {
        Shooter.Init(hardwareMap);
        Drive.Init(hardwareMap);

        waitForStart();

        super.waitForStart();

        Drive.ResetIMU();

        Drive.DriveFieldRelative(0, -1 * MaxShooterPower, 0);

        sleep(1500);

        Drive.DriveFieldRelative(0, 0, 0);

        Shooter.SetShooterPower(Constants.MAX_SHOOTER_POWER);

        sleep(3000);

        for (double i = 1; i <= 3; i++) {
            Shooter.SetServoPower(0.9);

            sleep(500);

            Shooter.SetServoPower(0);

            sleep(1500);
        }

        Shooter.SetShooterPower(0);

        Drive.DriveFieldRelative(0, -1 * MaxDrivePower, 0);

        sleep(700);

        Drive.DriveFieldRelative(0, 0, 0);
    }
}
