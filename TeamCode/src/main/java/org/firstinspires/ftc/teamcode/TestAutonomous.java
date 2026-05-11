package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Datatypes.Coordinate2d;
import org.firstinspires.ftc.teamcode.Datatypes.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TestAutonomous extends LinearOpMode {
    BaseAutonomous Autonomous = new BaseAutonomous();
    Constants Constants = new Constants();

    @Override
    public void runOpMode() {
        Autonomous.Init(hardwareMap, telemetry);
        Autonomous.MAX_SHOOTER_POWER = 0.9;

        waitForStart();

        Autonomous.DriveToCoordinate(
                new Coordinate2d(new Vector2d(-(3 * Constants.INCHES_PER_SQUARE), 0),
                        new Vector2d(3 * Constants.INCHES_PER_SQUARE,
                                6 * Constants.INCHES_PER_SQUARE
                        )));
    }
}
