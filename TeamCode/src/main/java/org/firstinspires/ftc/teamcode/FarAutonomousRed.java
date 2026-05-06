package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Datatypes.Coordinate2d;
import org.firstinspires.ftc.teamcode.Datatypes.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class FarAutonomousRed extends LinearOpMode {
    BaseAutonomous Autonomous = new BaseAutonomous();
    Constants Constants = new Constants();

    @Override
    public void runOpMode() {
        Autonomous.Init(hardwareMap, telemetry);

        Autonomous.MAX_SHOOTER_POWER = 0.9;

        waitForStart();

        Autonomous.SetShooterPower(1);

        Autonomous.Rotate(-1);

        sleep(175);

        Autonomous.Rotate(0);

        sleep(2500);

        for (double i = 1; i <= 3; i++) {
            Autonomous.SetServoPower(1);

            sleep(500);

            Autonomous.SetServoPower(0);

            sleep(1500);
        }

        Autonomous.SetShooterPower(0);

        sleep(100);

        Autonomous.DriveToCoordinate(new Coordinate2d(new Vector2d(0, -Constants.INCHES_PER_SQUARE), 0));
    }
}
