package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.teamcode.Datatypes.Coordinate2d;
import org.firstinspires.ftc.teamcode.Datatypes.Vector2d;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Map;
import java.util.Set;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class CloseAutonomous extends LinearOpMode {
    BaseAutonomous Autonomous = new BaseAutonomous();
    Constants Constants = new Constants();

    public void runOpMode() {
        Autonomous.Init(hardwareMap);

        waitForStart();

        Autonomous.DriveToCoordinate(new Coordinate2d(new Vector2d(0, -Constants.INCHES_PER_SQUARE * 2), 0));

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

        Autonomous.DriveToCoordinate(new Coordinate2d(new Vector2d(Constants.INCHES_PER_SQUARE, -Constants.INCHES_PER_SQUARE * 3), 0));
    }
}
