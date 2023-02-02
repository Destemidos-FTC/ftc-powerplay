package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;

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
                MovementSystem.controleMecanumAvançado(theta, direction, turn, robot);
            }
            else if (toggleButtonY.getState()) {
                MovementSystem.controleMecanumAlternativo(gamepad1, robot);
            }
            else if (toggleButtonA.getState()) {
                MovementSystem.controleFieldOriented(gamepad1, robot);
            }
            else if (toggleButtonB.getState()) {
                MovementSystem.controleTank(gamepad1, robot);
            }
            else {
                // caso algo dê errado:
                MovementSystem.controleOmnidirecionalClassico(gamepad1, robot);
            }


        }
    }
}
