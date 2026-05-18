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
        Autonomous.Init(hardwareMap);
        Autonomous.MAX_SHOOTER_POWER = 0.9;

        waitForStart();

        while (true) {
            Autonomous.DriveToCoordinate(
                    new Coordinate2d(new Vector2d(1, 1), 0)
            );
            sleep(10);
        }
    }
}
