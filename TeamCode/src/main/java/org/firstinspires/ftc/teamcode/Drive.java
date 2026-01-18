package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    // Make motor and imu variables

    private DcMotor FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor;
    private IMU Imu;
    final private Constants Constants = new Constants();

    public void Init(HardwareMap HwMap) {
        // Set variables

        FrontRightMotor = HwMap.get(DcMotor.class, Constants.FRONT_RIGHT_WHEEL_DEVICE_NAME);
        FrontLeftMotor = HwMap.get(DcMotor.class, Constants.FRONT_LEFT_WHEEL_DEVICE_NAME);
        BackRightMotor = HwMap.get(DcMotor.class, Constants.BACK_LEFT_WHEEL_DEVICE_NAME);
        BackLeftMotor = HwMap.get(DcMotor.class, Constants.BACK_RIGHT_WHEEL_DEVICE_NAME);

        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Imu = HwMap.get(IMU.class, Constants.IMU_DEVICE_NAME);

        RevHubOrientationOnRobot RobotOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        Imu.initialize(new IMU.Parameters(RobotOrientation));

        this.ResetIMU();
    }

    public void MoveMotors(double Forward, double Strafe, double Rotate) {
        // Math stuff (don't touch)

        double FrontLeftPower = Forward + Strafe + Rotate;
        double BackLeftPower = Forward - Strafe + Rotate;
        double FrontRightPower = Forward - Strafe - Rotate;
        double BackRightPower = Forward + Strafe - Rotate;

        double MaxPower = 1.0;

        MaxPower = Math.max(MaxPower, Math.abs(FrontLeftPower));
        MaxPower = Math.max(MaxPower, Math.abs(BackLeftPower));
        MaxPower = Math.max(MaxPower, Math.abs(FrontRightPower));
        MaxPower = Math.max(MaxPower, Math.abs(BackRightPower));

        FrontLeftMotor.setPower(Constants.MAX_DRIVE_SPEED * (FrontLeftPower / MaxPower));
        FrontRightMotor.setPower(Constants.MAX_DRIVE_SPEED * (FrontRightPower / MaxPower));
        BackRightMotor.setPower(Constants.MAX_DRIVE_SPEED * (BackRightPower / MaxPower));
        BackLeftMotor.setPower(Constants.MAX_DRIVE_SPEED * (BackLeftPower / MaxPower));
    }

    public void DriveFieldRelative(double Forward, double Strafe, double Rotate) {
        // More math stuff (also don't touch)

        double Theta = AngleUnit.normalizeRadians(Math.atan2(Forward, Strafe) - Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double R = Math.hypot(Strafe, Forward);

        double NewForward = R * Math.sin(Theta);
        double NewStrafe = R * Math.cos(Theta);

        this.MoveMotors(NewForward, NewStrafe, Rotate);
    }
    public void ResetIMU() {
        Imu.resetYaw();
        //Imu.resetDeviceConfigurationForOpMode();
        // ^ I wonder if this is why the drive didn't work
    }
}