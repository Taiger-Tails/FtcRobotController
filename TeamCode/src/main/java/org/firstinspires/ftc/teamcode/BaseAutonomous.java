package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BaseAutonomous {
    private Drive Drive = new Drive();
    private Shooter Shooter = new Shooter();
    final private Constants Constants = new Constants();

    private Telemetry Telemetry;

    public double MAX_SHOOTER_POWER = 0.467;
    public double MAX_FORWARD_POWER = 0.5;
    public double MAX_STRAFE_POWER = 0.5;
    public double MAX_SERVO_POWER = 0.9;

    public void Init(HardwareMap HwMap, Telemetry telemetry) {
        Shooter.Init(HwMap);
        Drive.Init(HwMap);
        Drive.ResetIMU();

        Telemetry = telemetry;
    }

    public void DriveInches(Vector2d Direction, double Forward, double Strafe) {
        double initialPositionFrontRight = Drive.FrontRight.getCurrentPosition();
        double initialPositionFrontLeft = Drive.FrontLeft.getCurrentPosition();
        double initialPositionBackRight = Drive.BackRight.getCurrentPosition();
        double initialPositionBackLeft = Drive.BackLeft.getCurrentPosition();

        if (Direction.z > 0) {
            Forward = Math.abs(Forward);
        } else {
            Forward = -Math.abs(Forward);
        }

        if (Direction.x > 0) {
            Strafe = Math.abs(Forward);
        } else {
            Strafe = -Math.abs(Forward);
        }

        while (true) {
            double FrontRightTicks = Drive.FrontRight.getCurrentPosition() - initialPositionFrontRight;
            double FrontLeftTicks = Drive.FrontLeft.getCurrentPosition() - initialPositionFrontLeft;
            double BackRightTicks = Drive.BackRight.getCurrentPosition() - initialPositionBackRight;
            double BackLeftTicks = Drive.BackLeft.getCurrentPosition() - initialPositionBackLeft;

            Vector2d Traveled = new Vector2d(
                    ((BackLeftTicks + BackRightTicks + FrontLeftTicks + FrontRightTicks) / 4) * Constants.INCHES_PER_TICK,
                    ((-FrontLeftTicks + FrontRightTicks + BackLeftTicks - BackRightTicks) / 4) * Constants.INCHES_PER_TICK);

            Telemetry.addData("x", Traveled.x);
            Telemetry.addData("z", Traveled.z);

            boolean DoStrafe =  Direction.x != 0 && !(Math.abs(Math.floor(Traveled.x)) >= Math.abs(Math.floor(Direction.x)));
            boolean DoForward = Direction.z != 0 && !(Math.abs(Math.floor(Traveled.z)) >= Math.abs(Math.floor(Direction.z)));

            if (!(DoForward || DoStrafe)) {
                Drive.DriveFieldRelative(0, 0, 0);
                break;
            }

            Drive.DriveFieldRelative(DoForward ? -Forward * MAX_FORWARD_POWER : 0, DoStrafe ? -Strafe * MAX_STRAFE_POWER : 0, 0);
        }
    }

    public void DriveCentimeters(Vector2d Direction, double Forward, double Strafe) {
        Direction.x *= 0.393700787;
        Direction.z *= 0.393700787;

        this.DriveInches(Direction, Forward, Strafe);
    }

    public void Rotate(double Power) { Drive.DriveFieldRelative(0, 0, Power); }

    public void ResetIMU() {
        Drive.ResetIMU();
    }

    public void SetShooterPower(double Power) { Shooter.SetShooterPower(Power * MAX_SHOOTER_POWER); }

    public void SetServoPower(double Power) {
        Shooter.SetServoPower(Power * MAX_SERVO_POWER);
    }
}
