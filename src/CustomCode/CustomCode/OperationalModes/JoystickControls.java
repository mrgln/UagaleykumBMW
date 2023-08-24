package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class JoystickControls extends OpMode {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    Servo steer;

    @Override
    public void init()
    {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotorEx.class,   "right_drive");

        steer = hardwareMap.get(Servo.class, "steer");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive = new SampleTankDrive(hardwareMap);
    }

    //    public static double steerMulti = -0.11;
    public static double steerMulti = 0.4;
    public static double speed = 1;
    public static double zeroSteer = 0.55;

    SampleTankDrive drive;

    @Override
    public void loop()
    {
        double forwardPower = Math.pow(gamepad1.right_trigger, 3) - Math.pow(gamepad1.left_trigger, 3);
        double rotationalPower = Math.pow(gamepad1.left_stick_x, 2) * (gamepad1.left_stick_x > 0 ? 1.0 : -1.0);

        drive.setDrivePower(new Pose2d(forwardPower, 0, rotationalPower));
        telemetry.addData("turn", drive.outputable);
        telemetry.update();
    }
}
