package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

public class AliançaVermelha extends CommandOpMode {
    private DestemidosBot robot;
    private GamepadEx player2;

    @Override
    public void initialize() {
        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player2 = new GamepadEx(gamepad2);
        CommandScheduler.getInstance().reset();
        register(robot.gripper, robot.armSystem, robot.forearmSystem);

        player2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(new InstantCommand(() -> robot.gripper.moveWrist(1)))
                .whenInactive(new InstantCommand(() -> robot.gripper.moveWrist(0)));

        player2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(new InstantCommand(() -> robot.gripper.moveWrist(-1)))
                .whenInactive(new InstantCommand(() -> robot.gripper.moveWrist(0)));

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(robot.gripper::closeGrip),
                        new InstantCommand(robot.gripper::openGrip),
                        true
                );

        player2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new InstantCommand(() -> robot.armSystem.setArmPosition(ArmSystem.ArmStage.CLOSED))
                );

        player2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new InstantCommand(() -> robot.armSystem.setArmPosition(ArmSystem.ArmStage.LOW))
                );

        player2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        new InstantCommand(() -> robot.armSystem.setArmPosition(ArmSystem.ArmStage.MEDIUM))
                );

        player2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> robot.armSystem.setArmPosition(ArmSystem.ArmStage.HIGH))
                );
    }


    @Override
    public void run() {
        double energy = robot.voltageSensor.getVoltage();

        CommandScheduler.getInstance().run();

        player2.readButtons();

        robot.armSystem.setVoltage(energy);
        robot.forearmSystem.moveForearmManually(gamepad2.right_stick_y);

        robot.drivetrain.updateVoltage(energy);
        robot.drivetrain.fieldOrientedController(gamepad1, robot.localizationSystem.getRobotHeading());
    }
}
