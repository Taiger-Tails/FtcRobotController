package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Datatypes.Coordinate2d;
import org.firstinspires.ftc.teamcode.Datatypes.Vector2d;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TestAutonomous extends LinearOpMode {
    BaseAutonomous Autonomous = new BaseAutonomous();
    Constants Constants = new Constants();

    @Override
    public void runOpMode() {
        Autonomous.Init(hardwareMap);

        waitForStart();

        while (true) {
            Autonomous.DriveToCoordinate(
                    new Coordinate2d(new Vector2d(0, 1 * Constants.INCHES_PER_SQUARE), Autonomous.Drive.Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))
            );

            Autonomous.DriveToCoordinate(
                    new Coordinate2d(new Vector2d(0, 0), Autonomous.Drive.Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))
            );
        }
    }
}
