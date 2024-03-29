package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;

/*
 * Constants shared between multiple drive types.
 *
 * Constants generated by LearnRoadRunner.com/drive-constants
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * Motor Constraints
     */
    public static final double GEARBOX = 15.2;
    public static final double TICKS_PER_REV = 28.0 * GEARBOX;
    public static final double MAX_RPM = 6000.0 / GEARBOX;
    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;

    /*
     * Multiplicadores
     */
    public static double LATERAL_MULTIPLIER = 1;

    /*
     * Coeficientes de PID
     */
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(40, 0, 10, 2.28);
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0,0,0);
    public static PIDCoefficients LATERAL_PID = new PIDCoefficients(0,0,0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0,0,0);

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.4763; // in
    public static double GEAR_RATIO = 1;       // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 9.3; // in
    public static double WHEEL_BASE = 11.64567; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0.0;
    public static double kStatic = 0;

    // essas constantes são medidas com os testes: MaxVelocityTuner e MaxAngularVeloTuner
    // usamos sempre os 85% da medidas "máximas"
    public static double MAX_VEL = 264.90;
    public static double MAX_ACCEL = 40;
    public static double MAX_ANG_VEL = 60;
    public static double MAX_ANG_ACCEL = 60;

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
      // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
      return 32767 / ticksPerSecond;
    }

    public static TrajectoryVelocityConstraint setVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint setAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}