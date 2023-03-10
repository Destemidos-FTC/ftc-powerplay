package org.firstinspires.ftc.teamcode.teleop;

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


    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {


        // controle da base
        drivetrain.standardMecanumController(gamepad1);

    }
}