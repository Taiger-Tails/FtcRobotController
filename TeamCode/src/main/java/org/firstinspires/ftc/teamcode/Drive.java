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

    public void Init(HardwareMap hwMap) {
        // Set variables

        FrontRightMotor = hwMap.get(DcMotor.class, "FR");
        FrontLeftMotor = hwMap.get(DcMotor.class, "FL");
        BackRightMotor = hwMap.get(DcMotor.class, "BR");
        BackLeftMotor = hwMap.get(DcMotor.class, "BL");

        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Imu = hwMap.get(IMU.class, "IMU");

        RevHubOrientationOnRobot RobotOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        Imu.initialize(new IMU.Parameters(RobotOrientation));
    }

    public void MoveMotors(double forward, double strafe, double rotate) {
        // Math stuff (don't touch)

        double FrontLeftPower = forward + strafe + rotate;
        double BackLeftPower = forward - strafe + rotate;
        double FrontRightPower = forward - strafe - rotate;
        double BackRightPower = forward + strafe - rotate;

        double MaxPower = 1.0;
        double MaxSpeed = 1.0;

        MaxPower = Math.max(MaxPower, Math.abs(FrontLeftPower));
        MaxPower = Math.max(MaxPower, Math.abs(BackLeftPower));
        MaxPower = Math.max(MaxPower, Math.abs(FrontRightPower));
        MaxPower = Math.max(MaxPower, Math.abs(BackRightPower));

        FrontLeftMotor.setPower(MaxSpeed * (FrontLeftPower / MaxPower));
        FrontRightMotor.setPower(MaxSpeed * (FrontRightPower / MaxPower));
        BackRightMotor.setPower(MaxSpeed * (BackRightPower / MaxPower));
        BackLeftMotor.setPower(MaxSpeed * (BackLeftPower / MaxPower));
    }

    public void DriveFieldRelative(double forward, double strafe, double rotate) {
        // More math stuff (also don't touch)

        double Theta = AngleUnit.normalizeRadians(Math.atan2(forward, strafe) - Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double R = Math.hypot(strafe, forward);

        double NewForward = R * Math.sin(Theta);
        double NewStrafe = R * Math.cos(Theta);

        this.MoveMotors(NewForward, NewStrafe, rotate);
    }
    public void ResetIMU() {
        Imu.resetYaw();
        Imu.resetDeviceConfigurationForOpMode();
    }
}