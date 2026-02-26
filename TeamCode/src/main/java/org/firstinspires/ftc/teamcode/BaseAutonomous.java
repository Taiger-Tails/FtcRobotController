package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BaseAutonomous {
    private Drive Drive = new Drive();
    private Shooter Shooter = new Shooter();
    final private Constants Constants = new Constants();

    private Telemetry Telemetry;

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

        while (true) {
            double FrontRightTicks = Drive.FrontRight.getCurrentPosition() - initialPositionFrontRight;
            double FrontLeftTicks = Drive.FrontLeft.getCurrentPosition() - initialPositionFrontLeft;
            double BackRightTicks = Drive.BackRight.getCurrentPosition() - initialPositionBackRight;
            double BackLeftTicks = Drive.BackLeft.getCurrentPosition() - initialPositionBackLeft;

            Vector2d Traveled = new Vector2d(
                    ((BackLeftTicks + BackRightTicks + FrontLeftTicks + FrontRightTicks) / 4) * Constants.INCHES_PER_TICK,
                    ((BackLeftTicks - BackRightTicks + FrontLeftTicks - FrontRightTicks) / 4) * Constants.INCHES_PER_TICK);

            Telemetry.addData("x", Traveled.x);
            Telemetry.addData("z", Traveled.z);

            boolean DoStrafe = Math.abs(Math.floor(Traveled.x)) != Math.floor(Direction.x);
            boolean DoForward = Math.abs(Math.floor(Traveled.z)) != Math.floor(Direction.z);

            if (!(DoForward || DoStrafe)) {
                Drive.DriveFieldRelative(0, 0, 0);
                break;
            }

            Drive.DriveFieldRelative(DoForward ? -Forward : 0, DoStrafe ? -Strafe : 0, 0);
        }
    }

    public void DriveCentimeters(Vector2d Direction, double Forward, double Strafe) {
        Direction.x *= 0.393700787;
        Direction.z *= 0.393700787;

        this.DriveInches(Direction, Forward, Strafe);
    }

    public void RotateToDegree(double Degree, double Power) {
        while (Math.floor( Drive.Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) ) != Math.floor(Degree)) {
            Drive.DriveFieldRelative(0, 0, Power);
        }

        Drive.DriveFieldRelative(0, 0, 0);
    }

    public void ResetIMU() {
        Drive.ResetIMU();
    }
}
