package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Gripper;

/**
 * Teste focado em utilizar o sistema de commandos
 * junto dos recursos do GamepadEx, providos pela FTCLib
 */
@TeleOp(name = "TestGripperCommand", group = "Test")
@Disabled
public class TestGripperCommand extends CommandOpMode {
    private Gripper gripper;
    private GamepadEx player2;

    @Override
    public void initialize() {
        gripper = new Gripper(hardwareMap);
        player2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();
        register(gripper);

        // quando apertado uma vez, fecha a garra
        // na segunda, ele abre a garra
        player2.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new InstantCommand(gripper::closeGrip),
                        new InstantCommand(gripper::openGrip),
                        true
                );

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(new InstantCommand(() -> gripper.moveWrist(1)), true)
                .whenInactive(new InstantCommand(() -> gripper.moveWrist(0)), true);

        player2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new InstantCommand(() -> gripper.moveWrist(- 1)), true)
                .whenInactive(new InstantCommand(() -> gripper.moveWrist(0)), true);
     }

    @Override
    public void run() {

        player2.readButtons();

        CommandScheduler.getInstance().run();

        telemetry.addData("gripper position", gripper.gripper.getCurrentPosition());
        telemetry.addData("wrist A power", gripper.wristServoA.getPower());
        telemetry.addData("wrist B power", gripper.wristServoB.getPower());
        telemetry.addData("is wristA enabled?: ", gripper.wristServoA.isPwmEnabled());
        telemetry.addData("is wristB enabled?: ", gripper.wristServoB.isPwmEnabled());
        telemetry.addData("is gripper enabled?: ", gripper.gripper.isMotorEnabled());
        telemetry.update();
    }
}
