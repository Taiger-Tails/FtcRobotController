package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@TeleOp
public class FlywheelTuner extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(FlywheelTuner.class);
    private final Constants Constants = new Constants();
    public DcMotorEx FlywheelMotor;

    public double HighVelocity = 1500;
    public double LowVelocity = 900;

    public double CurTargetVelocity = HighVelocity;

    double F = 0;
    double P = 0;

    double[] StepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int StepIndex = 1;

    Shooter Shooter = new Shooter();

    @Override
    public void init() {
        FlywheelMotor = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_WHEEL_NAME);
        Shooter.Init(hardwareMap);
        FlywheelMotor.setMode(Constants.WHEEL_RUN_MODE);

        PIDFCoefficients pidfCoeffcients = new PIDFCoefficients(P, 0, 0, F);
        FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffcients);


    }

    @Override
    public void loop() {
        CurTargetVelocity = gamepad1.yWasPressed() ?
                CurTargetVelocity == HighVelocity ? LowVelocity :
                        CurTargetVelocity == LowVelocity ? HighVelocity :
                                CurTargetVelocity : CurTargetVelocity;

        if (gamepad1.bWasPressed()) {
            StepIndex = (StepIndex + 1) % StepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= StepSizes[StepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += StepSizes[StepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += StepSizes[StepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= StepSizes[StepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0,F);
        FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        FlywheelMotor.setVelocity(CurTargetVelocity);

        double CurVelocity = FlywheelMotor.getVelocity();
        double Error = CurTargetVelocity - CurVelocity;

        telemetry.addData("Target Velocity", CurTargetVelocity);
        telemetry.addData("Current Velocity","%.2f", CurVelocity);
        telemetry.addData("Error","%.2f", Error);
        telemetry.addLine("---------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", StepSizes[StepIndex]);

    }
}