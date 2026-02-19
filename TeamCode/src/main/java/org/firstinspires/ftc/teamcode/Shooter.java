/package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    // Make variables
    final private Constants Constants = new Constants();

    public DcMotorEx ShooterMotor, ShooterMotor2;
    private CRServo ServoLeft, ServoRight;

    private double PreviousMotorRevs = 0;
    public double ShooterSpeed = 0;
    public double MaxShooterPower = Constants.MAX_SHOOTER_POWER;

    public void Init(HardwareMap HwMap) {
        // Set variables
        PIDFCoefficients PIDFCoefficients = new PIDFCoefficients(15, 1.0102, 12, 0);

        ShooterMotor = HwMap.get(DcMotorEx.class, Constants.SHOOTER_WHEEL_NAME);
        ServoLeft = HwMap.get(CRServo.class, Constants.SERVO_LEFT_NAME);
        ServoRight = HwMap.get(CRServo.class, Constants.SERVO_RIGHT_NAME);
        ShooterMotor2 = HwMap.get(DcMotorEx.class, Constants.SECOND_SHOOTER_WHEEL_NAME);

        ShooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        ShooterMotor2.setPIDFCoefficients(Constants.WHEEL_RUN_MODE, PIDFCoefficients);
        ShooterMotor.setPIDFCoefficients(Constants.WHEEL_RUN_MODE, PIDFCoefficients);

        // Set it to break because when testing it wastes time by not coming to a complete stop
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetShooterPower(final double Power) {
        final double motorRevs = ShooterMotor.getCurrentPosition() / ShooterMotor.getMotorType().getTicksPerRev();

        ShooterMotor.setPower(Power * MaxShooterPower);
        ShooterMotor2.setPower(Power * MaxShooterPower);
        ShooterSpeed = motorRevs - PreviousMotorRevs;
        PreviousMotorRevs = motorRevs;
    }

    public void SetServoPower(final double Power) {
        if (Math.abs(ShooterSpeed) > Constants.MIN_SPEED_TO_ENABLE_SERVOS - (Constants.MAX_SHOOTER_POWER - MaxShooterPower / 1000)) {
            ServoLeft.setPower(-Power);
            ServoRight.setPower(Power);
        } else {
            ServoLeft.setPower(0);
            ServoRight.setPower(0);
        }
    }
}
