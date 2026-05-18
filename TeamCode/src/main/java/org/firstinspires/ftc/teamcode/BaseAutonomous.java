package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Datatypes.Coordinate2d;
import org.firstinspires.ftc.teamcode.Datatypes.Vector2d;

public class BaseAutonomous {
    final private Drive Drive = new Drive();
    final private Shooter Shooter = new Shooter();
    final private Constants Constants = new Constants();

    public double MAX_SHOOTER_POWER = 0.467;
    public double MAX_FORWARD_POWER = 1;
    public double MAX_STRAFE_POWER = 1;
    public double MAX_SERVO_POWER = 0.9;
    public double MAX_ROTATION_POWER = 1;

    private double InitialPositionFrontRight;
    private double InitialPositionFrontLeft;
    private double InitialPositionBackRight;
    private double InitialPositionBackLeft;

    public Vector2d CurrentPosition = new Vector2d(0, 0);

    public void Init(HardwareMap HwMap) {
        Shooter.Init(HwMap);
        Drive.Init(HwMap);
        Drive.ResetIMU();

        InitialPositionFrontRight = Drive.FrontRight.getCurrentPosition();
        InitialPositionFrontLeft = Drive.FrontLeft.getCurrentPosition();
        InitialPositionBackRight = Drive.BackRight.getCurrentPosition();
        InitialPositionBackLeft = Drive.BackLeft.getCurrentPosition();
    }

    public void DriveToCoordinate(Coordinate2d Coordinate) {
        Vector2d InitialPosition = new Vector2d(CurrentPosition.x, CurrentPosition.z);
        Vector2d Point = Coordinate.Position;

        while (true) {
            double FrontRightTicks = Drive.FrontRight.getCurrentPosition() - InitialPositionFrontRight;
            double FrontLeftTicks = Drive.FrontLeft.getCurrentPosition() - InitialPositionFrontLeft;
            double BackRightTicks = Drive.BackRight.getCurrentPosition() - InitialPositionBackRight;
            double BackLeftTicks = Drive.BackLeft.getCurrentPosition() - InitialPositionBackLeft;

            Vector2d Direction = new Vector2d(
                    Point.x - CurrentPosition.x,
                    Point.z - CurrentPosition.z
            );

            double Yaw = Drive.Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            boolean StopRotating = (
                    (float)(Math.floor(Coordinate.Rotation) - Constants.GOAL_ARRIVE_ROTATION_ERROR_MARGIN) == (float)(Math.floor(Yaw) - Constants.GOAL_ARRIVE_ROTATION_ERROR_MARGIN) ||
                    (float)(Math.floor(Coordinate.Rotation) - Constants.GOAL_ARRIVE_ROTATION_ERROR_MARGIN) == (float)(Math.floor(Yaw) - Constants.GOAL_ARRIVE_ROTATION_ERROR_MARGIN)
            );

            Vector2d Traveled = new Vector2d(
                    ((BackLeftTicks + BackRightTicks + FrontLeftTicks + FrontRightTicks) / 4) * Constants.INCHES_PER_TICK,
                    ((-FrontLeftTicks + FrontRightTicks + BackLeftTicks - BackRightTicks) / 4) * Constants.INCHES_PER_TICK
            );

            int Multiplier = (int)Math.pow(10, Constants.GOAL_DIGIT_PRECISION);

            if (Math.floor(CurrentPosition.x * Multiplier) == Math.floor(Point.x * Constants.INCHES_PER_TICK * Multiplier)
                    && Math.floor(CurrentPosition.z * Multiplier) == Math.floor(Point.z * Constants.INCHES_PER_TICK * Multiplier)
                    && StopRotating)
            { Drive.DriveFieldRelative(0, 0, 0); break; }

            Drive.DriveFieldRelative(
                    MAX_FORWARD_POWER * Math.max(Math.min(Direction.x / Constants.GOAL_ARRIVE_SMOOTHNESS, 1), -1),
                    MAX_STRAFE_POWER * Math.max(Math.min(Direction.z / Constants.GOAL_ARRIVE_SMOOTHNESS, 1), -1),
                     StopRotating ? 0 : MAX_ROTATION_POWER * Math.max(Math.min((Coordinate.Rotation - Yaw) / Constants.GOAL_ARRIVE_ROTATION_SMOOTHNESS, 1), -1)
            );

            CurrentPosition.x = InitialPosition.x + Traveled.x;
            CurrentPosition.z = InitialPosition.z + Traveled.z;
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
