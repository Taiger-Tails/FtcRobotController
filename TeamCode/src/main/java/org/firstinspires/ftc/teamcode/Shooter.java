package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    // Make variables
    final private Constants Constants = new Constants();

    private DcMotor ShooterMotor, ShooterMotor2;
    private CRServo ServoLeft, ServoRight;

    private double PreviousMotorRevs = 0;
    public double ShooterSpeed = 0;
    public double MaxShooterPower = Constants.MAX_SHOOTER_POWER;

    public boolean ServoOverride = false;

    public void Init(HardwareMap HwMap) {
        // Set variables
        ShooterMotor = HwMap.get(DcMotor.class, Constants.SHOOTER_WHEEL_NAME);
        ServoLeft = HwMap.get(CRServo.class, Constants.SERVO_LEFT_NAME);
        ServoRight = HwMap.get(CRServo.class, Constants.SERVO_RIGHT_NAME);
        ShooterMotor2 = HwMap.get(DcMotor.class, "Shooter2");

        ShooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

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
