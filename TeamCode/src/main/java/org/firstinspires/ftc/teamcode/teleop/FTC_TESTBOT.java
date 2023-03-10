package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmToGround;
import org.firstinspires.ftc.teamcode.commands.ArmToHighJunction;
import org.firstinspires.ftc.teamcode.commands.ArmToLowJunction;
import org.firstinspires.ftc.teamcode.commands.ArmToMediumJunction;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

/**
 * OpMode focado em experimentação de novas idéias
 * e de medição das informações do robô
 */
@TeleOp(name = "TESTBOT", group = "Test")
public class FTC_TESTBOT extends CommandOpMode {
    private DestemidosBot robot;
    private GamepadEx player2;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player2 = new GamepadEx(gamepad2);

        //CommandScheduler.getInstance().reset();
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
                .whenPressed(new ArmToGround(robot));

        player2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ArmToLowJunction(robot));

        player2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ArmToMediumJunction(robot));

        player2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ArmToHighJunction(robot));
    }

    @Override
    public void run() {

        double antes = System.currentTimeMillis();

        double energy = robot.voltageSensor.getVoltage();

        CommandScheduler.getInstance().run();

        player2.readButtons();

        robot.armSystem.setVoltage(energy);
        robot.forearmSystem.setVoltage(energy);

        robot.drivetrain.updateVoltage(energy);
        robot.drivetrain.standardMecanumController(gamepad1);

        double depois = System.currentTimeMillis();

        telemetry.addData("voltagem do controlhub", robot.voltageSensor.getVoltage());
        telemetry.addData("arm position", robot.armSystem.armA.getCurrentPosition());
        telemetry.addData("arm feedforward:", robot.armSystem.getArmFeedforwardPower());
        telemetry.addData("arm pid:", robot.armSystem.getArmPidPower());
        telemetry.addData("tempo de loop (ms)", depois - antes);
        telemetry.update();
    }
}