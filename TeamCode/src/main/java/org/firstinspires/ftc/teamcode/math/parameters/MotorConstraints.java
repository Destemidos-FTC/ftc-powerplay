package org.firstinspires.ftc.teamcode.math.parameters;

/**
 *
 */
public class MotorConstraints {

    /**
     *
     */
    double TICKS_PER_REV;

    /**
     *
     */
    double MAX_RPM;

    /**
     * https://docs.revrobotics.com/ultraplanetary/ultraplanetary-gearbox/cartridge-details
     */
    double GEARBOX;

    public MotorConstraints(double ticksPerRev, double maxRpm, double gearbox) {
        this.TICKS_PER_REV = ticksPerRev;
        this.MAX_RPM = maxRpm;
        this.GEARBOX = gearbox;
    }
}
