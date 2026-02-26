package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class CloseAutonomousLeave extends LinearOpMode {
    BaseAutonomous Autonomous = new BaseAutonomous();
    Constants Constants = new Constants();

    @Override
    public void runOpMode() {
        Autonomous.Init(hardwareMap, telemetry);

        waitForStart();

        Autonomous.DriveInches(new Vector2d(-Constants.INCHES_PER_SQUARE * 3, Constants.INCHES_PER_SQUARE), -0.5, 0.01);
    }
}
