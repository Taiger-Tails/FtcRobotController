package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.Datatypes.Coordinate2d;
import org.firstinspires.ftc.teamcode.Datatypes.Vector2d;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

public class BaseAutonomous {
    final private Drive Drive = new Drive();
    final private Shooter Shooter = new Shooter();
    final private Constants Constants = new Constants();

    private Telemetry Telemetry;

    public double MAX_SHOOTER_POWER = 0.467;
    public double MAX_FORWARD_POWER = 1;
    public double MAX_STRAFE_POWER = 1;
    public double MAX_SERVO_POWER = 0.9;
    public double MAX_ROTATION_POWER = 1;

    public Vector2d CurrentPosition = new Vector2d(0, 0);

    public void Init(HardwareMap HwMap, Telemetry telemetry) {
        Shooter.Init(HwMap);
        Drive.Init(HwMap);
        Drive.ResetIMU();

        Telemetry = telemetry;
    }

    public void DriveToCoordinate(Coordinate2d Coordinate) {
        Vector2d InitialPosition = new Vector2d(CurrentPosition.x, CurrentPosition.z);
        Vector2d Point = Coordinate.Position;

        double InitialPositionFrontRight = Drive.FrontRight.getCurrentPosition();
        double InitialPositionFrontLeft = Drive.FrontLeft.getCurrentPosition();
        double InitialPositionBackRight = Drive.BackRight.getCurrentPosition();
        double InitialPositionBackLeft = Drive.BackLeft.getCurrentPosition();

        Vector2d Direction = new Vector2d(
                Point.x - CurrentPosition.x,
                Point.z - CurrentPosition.z
        );

        while (true) {
            double FrontRightTicks = Drive.FrontRight.getCurrentPosition() - InitialPositionFrontRight;
            double FrontLeftTicks = Drive.FrontLeft.getCurrentPosition() - InitialPositionFrontLeft;
            double BackRightTicks = Drive.BackRight.getCurrentPosition() - InitialPositionBackRight;
            double BackLeftTicks = Drive.BackLeft.getCurrentPosition() - InitialPositionBackLeft;

            double Yaw = Drive.Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            Vector2d Traveled = new Vector2d(
                    ((BackLeftTicks + BackRightTicks + FrontLeftTicks + FrontRightTicks) / 4) * Constants.INCHES_PER_TICK,
                    ((-FrontLeftTicks + FrontRightTicks + BackLeftTicks - BackRightTicks) / 4) * Constants.INCHES_PER_TICK
            );

            int Multiplier = (int)Math.pow(10, Constants.GOAL_DIGIT_PRECISION);

            if (Math.floor(CurrentPosition.x * Multiplier) == Math.floor(Point.x * Multiplier)
                    && Math.floor(CurrentPosition.z * Multiplier) == Math.floor(Point.z * Multiplier)
                    && Math.floor(Coordinate.Rotation * Multiplier) == Math.floor(Yaw * Multiplier))
            { Drive.DriveFieldRelative(0, 0, 0); break; }

            Drive.DriveFieldRelative(
                    MAX_FORWARD_POWER * Math.max(Math.min(Direction.z / Constants.GOAL_ARRIVE_SMOOTHNESS, 1), -1),
                    MAX_STRAFE_POWER * Math.max(Math.min(Direction.x / Constants.GOAL_ARRIVE_SMOOTHNESS, 1), -1),
                    MAX_ROTATION_POWER * Math.max(Math.min((Yaw + Coordinate.Rotation) / Constants.GOAL_ARRIVE_ROTATION_SMOOTHNESS, 1), -1)
            );

            CurrentPosition.add(Traveled);
        }
    }

    @Deprecated
    public void Rotate(double Power) { Drive.DriveFieldRelative(0, 0, Power); }

    public void ResetIMU() {
        Drive.ResetIMU();
    }

    public void SetShooterPower(double Power) { Shooter.SetShooterPower(Power * MAX_SHOOTER_POWER); }

    public void SetServoPower(double Power) {
        Shooter.SetServoPower(Power * MAX_SERVO_POWER);
    }
}
