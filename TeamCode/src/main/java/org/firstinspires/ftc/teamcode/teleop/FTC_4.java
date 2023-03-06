package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * OpMode princiapl para as competições.
 * O foco é sempre buscar o máximo de performance
 * e estabilidade por aqui
 */
@TeleOp(name = "FTC_4", group = "TeleOps")
public class FTC_4 extends OpMode {
    private static final PwmControl.PwmRange MAX_REV_SERVO_RANGE = new PwmControl.PwmRange(500, 2500, 18000);
    private Drivetrain drivetrain;
    private Intake intake;

    private DcMotor leftExtension;
    private DcMotor rightExtension;

    private DcMotor leftOuttake;
    private DcMotor rightOuttake;

    private ServoImplEx garra;
    private CRServoImplEx garrarotate;

    private ElapsedTime timer;
    private ElapsedTime garraTimer;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        intake =  new Intake(hardwareMap);

        leftExtension = hardwareMap.get(DcMotor.class, "leftExtension");
        rightExtension = hardwareMap.get(DcMotor.class, "rightExtension");

        leftOuttake = hardwareMap.get(DcMotor.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotor.class, "rightOuttake");

        garra = (ServoImplEx) hardwareMap.get(Servo.class,"garra");
        garrarotate = (CRServoImplEx) hardwareMap.get(CRServo.class,"garrarotate");

        garra.setPwmRange(MAX_REV_SERVO_RANGE);
        garrarotate.setPwmRange(MAX_REV_SERVO_RANGE);

        leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
        garraTimer = new ElapsedTime();
        timer.reset();
        garraTimer.reset();
    }

    @Override
    public void loop() {

        // rotação da garra
        if(garraTimer.seconds() > 0.01) {
            garrarotate.setPower(0.0);
        }

        if(gamepad2.left_trigger > 0.1) {
            garrarotate.setPower(1.0);
            garraTimer.reset();
        }

        if(gamepad2.left_bumper) {
            garrarotate.setPower(-1.0);
            garraTimer.reset();
        }

        // Fourbar
        if(timer.seconds() > 0.01) {
            intake.desligaservo();
        }

        if(gamepad2.x) {
            intake.moveFourbar(1.0);
            timer.reset();
        }

        if(gamepad2.y) {
            intake.moveFourbar(-1.0);
            timer.reset();
        }

        // Garra
        if(gamepad2.right_trigger > 0.1) {
            garra.setPosition(0.0);
        }
        if(gamepad2.right_bumper) {
            garra.setPosition(0.3);
        }

        // controle da base
        drivetrain.standardMecanumController(gamepad1);

        // controle do intake
        leftExtension.setPower(gamepad2.left_stick_y/ 2);
        rightExtension.setPower(gamepad2.left_stick_y/ 2);

        // controle do outtake
        leftOuttake.setPower(gamepad2.right_stick_y * 0.60);
        rightOuttake.setPower(gamepad2.right_stick_y * 0.60);

    }
}