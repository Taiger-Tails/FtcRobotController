package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Datatypes.Coordinate2d;
import org.firstinspires.ftc.teamcode.Datatypes.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class FarAutonomousBlueLeave extends LinearOpMode {
    BaseAutonomous Autonomous = new BaseAutonomous();
    Constants Constants = new Constants();

    @Override
    public void runOpMode() {
        Autonomous.Init(hardwareMap);

        waitForStart();

        Autonomous.DriveToCoordinate(new Coordinate2d(new Vector2d(0, Constants.INCHES_PER_SQUARE * 1.25), 0));
    }
}
