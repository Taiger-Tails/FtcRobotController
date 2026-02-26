package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class CloseAutonomous extends LinearOpMode {
    BaseAutonomous Autonomous = new BaseAutonomous();
    Constants Constants = new Constants();

    @Override
    public void runOpMode() {
        Autonomous.Init(hardwareMap, telemetry);

        waitForStart();

        Autonomous.DriveInches(new Vector2d(-Constants.INCHES_PER_SQUARE * 2, 0), -1, 0.01);

        sleep(100);

        Autonomous.SetShooterPower(1);

        sleep(3000);

        for (double i = 1; i <= 3; i++) {
            Autonomous.SetServoPower(1);

            sleep(500);

            Autonomous.SetServoPower(0);

            sleep(1500);
        }

        Autonomous.SetShooterPower(0);

        sleep(100);

        Autonomous.DriveInches(new Vector2d(-Constants.INCHES_PER_SQUARE, Constants.INCHES_PER_SQUARE), 1, 1);
    }
}
