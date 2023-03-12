package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.ArmToGround;
import org.firstinspires.ftc.teamcode.commands.ArmToHighJunction;
import org.firstinspires.ftc.teamcode.commands.ArmToLowJunction;
import org.firstinspires.ftc.teamcode.commands.ArmToMediumJunction;
import org.firstinspires.ftc.teamcode.commands.CloseArm;
import org.firstinspires.ftc.teamcode.subsystems.ArmSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * OpMode focado em experimentação de novas idéias
 * e de medição das informações do robô
 */
@TeleOp(name = "TESTBOT", group = "Test")
public class FTC_TESTBOT extends CommandOpMode {
    private DestemidosBot robot;
    private GamepadEx player2;
    private Trigger garraTrigger;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player2 = new GamepadEx(gamepad2);

        register(robot.gripper, robot.armSystem);

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(new InstantCommand(() -> robot.gripper.moveWrist(1)))
                .whenInactive(new InstantCommand(() -> robot.gripper.moveWrist(0)));

        player2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new InstantCommand(() -> robot.gripper.moveWrist(-1)))
                .whenInactive(new InstantCommand(() -> robot.gripper.moveWrist(0)));

        player2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ArmToGround(robot));

        player2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ArmToLowJunction(robot));

        player2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ArmToMediumJunction(robot));

        player2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ArmToHighJunction(robot));

        player2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new CloseArm(robot));

        garraTrigger = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0)
                .toggleWhenActive(
                        new InstantCommand(robot.gripper::openGrip),
                        new InstantCommand(robot.gripper::closeGrip),
                        true
                );
    }

    @Override
    public void run() {
        double antes = System.currentTimeMillis();

        double energy = robot.voltageSensor.getVoltage();

        player2.readButtons();
        CommandScheduler.getInstance().run();

        robot.armSystem.setVoltage(energy);

        robot.drivetrain.updateVoltage(energy);
        robot.drivetrain.standardMecanumController(gamepad1);

        double depois = System.currentTimeMillis();

        telemetry.addData("arm position", robot.armSystem.armA.getCurrentPosition());
        telemetry.addData("arm feedforward:", robot.armSystem.getArmFeedforwardPower());
        telemetry.addData("arm pid:", robot.armSystem.getArmPID());

        telemetry.addData("forearm position", robot.armSystem.forearmMotor.getCurrentPosition());
        telemetry.addData("forearm feedforward:", robot.armSystem.getForearmFeedforwardPower());
        telemetry.addData("forearm pid:", robot.armSystem.getForearmPID());

        telemetry.addData("voltagem do controlhub", robot.voltageSensor.getVoltage());
        telemetry.addData("tempo de loop (ms)", depois - antes);
        telemetry.update();
    }
}