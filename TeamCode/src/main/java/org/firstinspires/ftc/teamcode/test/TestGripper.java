package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Gripper;

/**
 * Teste focado em utilizar o sistema de commandos
 * junto dos recursos do GamepadEx, providos pela FTCLib
 */
@TeleOp(name = "TestGripper", group = "Test")
public class TestGripper extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final Gripper gripper   = new Gripper(hardwareMap);
        final GamepadEx player2 = new GamepadEx(gamepad2);

        // definimos as ações de cada botão aqui!

        // quando apertado uma vez, fecha a garra
        // na segunda, ele abre a garra
        player2.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(
                        new InstantCommand(gripper::closeGrip),
                        new InstantCommand(gripper::releaseGrip),
                        true
                );

        // quando pressioando uma vez, gira a garra em 180 graus (vira pra base)
        // na segunda vez, ela retorna a sua posição incial
        player2.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(
                        new InstantCommand(gripper::rotateGripper, gripper),
                        new InstantCommand(gripper::returnToCollectPostion, gripper),
                        true
                );

        waitForStart();
        while (opModeIsActive()) {
            player2.readButtons();

            // nunca esqueça de chamar essa função!
            CommandScheduler.getInstance().run();

            telemetry.addData("gripper position", gripper.gripper.getPosition());
            telemetry.addData("gripper angle", gripper.gripper.getAngle());
            telemetry.addData("rotator position", gripper.rotator.getPosition());
            telemetry.addData("rotator angle", gripper.rotator.getAngle());
            telemetry.update();
        }

    }
}
