package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;


@TeleOp(name="Soares Torquato Maressa", group="Linear Opmode")

public class TesteGarra extends LinearOpMode {
    private GamepadEx player2;

    private ElapsedTime runtime = new ElapsedTime();

    private CRServoImplEx servo_E;
    private CRServoImplEx servo_D;

    private DestemidosBot robot;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new DestemidosBot(hardwareMap);

        player2 = new GamepadEx(gamepad2);

        servo_E = (CRServoImplEx) hardwareMap.get(CRServo.class, "servo_e");
        servo_D = (CRServoImplEx) hardwareMap.get(CRServo.class, "servo_d");

        servo_E.setDirection(CRServo.Direction.REVERSE);
        servo_D.setDirection(CRServo.Direction.FORWARD);

        servo_E.setPwmRange(RobotConstants.MAX_SERVO_RANGE);
        servo_D.setPwmRange(RobotConstants.MAX_SERVO_RANGE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            player2.getGamepadButton(GamepadKeys.Button.A)
                    .whenActive(new InstantCommand(() -> servo_E.setPower(0.5)))
                    .whenActive(new InstantCommand(() -> servo_D.setPower(0.5)));

            player2.getGamepadButton(GamepadKeys.Button.X)
                    .whenActive(new InstantCommand(() -> servo_E.setPower(-0.5)))
                    .whenActive(new InstantCommand(() -> servo_D.setPower(-0.5)));

        }
    }
}
