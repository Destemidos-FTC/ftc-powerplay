package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

@TeleOp(name = "TesteMovementSystem", group = "Test")
public class TesteMovementSystem extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // setup
        final DestemidosBot robot = new DestemidosBot(hardwareMap);
        final GamepadEx driver = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // leitores dos botões, que controlam a troca dos modos
        ToggleButtonReader toggleButtonY = new ToggleButtonReader(driver, GamepadKeys.Button.Y);
        ToggleButtonReader toggleButtonX = new ToggleButtonReader(driver, GamepadKeys.Button.X);
        ToggleButtonReader toggleButtonA = new ToggleButtonReader(driver, GamepadKeys.Button.A);
        ToggleButtonReader toggleButtonB = new ToggleButtonReader(driver, GamepadKeys.Button.B);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            driver.readButtons();

            // inputs para o controle avançado
            double theta        = Math.atan2(driver.getLeftX(), driver.getLeftY());
            double direction    = Math.hypot(driver.getLeftY(), driver.getLeftX());
            double turn         = driver.getRightX();

            /* controle dos modos:
            *
            * X = MecanumAvançado
            * Y = MecanumAlterntivo
            * A = Field Oriented
            * B = Tank
            */

            if(toggleButtonX.getState()) {
                //movementSystem.controleMecanumAvançado(theta, direction, turn);
            }
            else if (toggleButtonY.getState()) {
                robot.drivetrain.driveAlternativeMecanum(gamepad1);
            }
            else if (toggleButtonA.getState()) {
                robot.drivetrain.driveFieldOriented(gamepad1);
            }
            else if (toggleButtonB.getState()) {
                robot.drivetrain.driveTank(gamepad1);
            }
            else {
                // caso algo dê errado:
                robot.drivetrain.driveStandardMecanum(gamepad1);
            }


        }
    }
}
