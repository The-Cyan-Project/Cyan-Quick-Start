package org.firstinspires.ftc.teamcode.disabledSamples;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Debuggers;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Point;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.localization.TankKinematics;
import com.github.bouyio.cyancore.pathing.PathFollower;
import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * <p>
 *     This class demonstrates <strong>single</strong> smart point following in CyanFTC.
 *     The advantage of using smart points instead of normal ones is the distance unit
 *     certainty and safety.
 * </p>
 * */
@Disabled
@TeleOp()
public class SmartPointFollowing extends OpMode {

    // TODO: Set constants to match the ones of the robot
    // NOTE: The distance unit of the track width must be
    // the same as the distance unit of the converted motor
    // rotations.
    final double TRACK_WIDTH = 0000000000;
    final double WHEEL_RADIUS = 000000000;
    final double ENCODER_COUNT_PER_REVOLUTION = 000000000;
    final double TICKS_TO_LINEAR_DISTANCE = 2 * Math.PI * WHEEL_RADIUS / ENCODER_COUNT_PER_REVOLUTION;

    /**The localization system.*/
    TankKinematics odometry;

    /**The point following system.*/
    PathFollower follower;

    // The motors we will be using.
    DcMotor leftMotor;
    DcMotor rightMotor;

    /**The logging system.*/
    Logger logger;

    /**The smart point used for point following demonstration.*/
    SmartPoint point;

    @Override
    public void init() {
        // Initializing hardware.
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializing localization system.
        TankKinematics.TankKinematicsMeasurementProvider measurementProvider = new TankKinematics.TankKinematicsMeasurementProvider(
                leftMotor::getCurrentPosition,
                rightMotor::getCurrentPosition,
                TICKS_TO_LINEAR_DISTANCE
        );

        odometry = new TankKinematics(TRACK_WIDTH, Distance.DistanceUnit.CM, measurementProvider);

        // Initializing the point following system.

        // Initializing the steering PID controller coefficients.
        // TODO: Tune the controller and set the actual coefficient.
        PIDCoefficients coefficients = new PIDCoefficients();
        coefficients.kP = 0000000;
        coefficients.kD = 0000000;
        coefficients.kI = 0000000;

        // Initializing the point follower itself.
        follower = new PathFollower(odometry);

        // Configuring the follower.
        // TODO: Set the preferred distance unit.
        follower.setDistanceUnitOfMeasurement(Distance.DistanceUnit.CM);

        // Initializing the point.
        // TODO: Set the actual coordinates and distance unit of the point.
        point = new SmartPoint(Distance.DistanceUnit.CM, 000000, 000000);

        // Initializing localization system logging.
        logger = Debuggers.getGlobalLogger();
        odometry.attachLogger(logger);
        follower.attachLogger(logger);

    }

    @Override
    public void loop() {

        // Point following code.

        // Setting the target.
        follower.followSmartPoint(point);

        // Getting the movement instruction from the follower.
        double[] movementPowerInstructions = follower.getCalculatedPowers();

        // Converting them to movement powers.
        double leftMotorPower = movementPowerInstructions[0] + movementPowerInstructions[1];
        double rightMotorPower = movementPowerInstructions[0] - movementPowerInstructions[1];

        // Normalizing motor powers.
        double max = Math.max(leftMotorPower, rightMotorPower);
        leftMotorPower /= max;
        rightMotorPower /= max;

        // Applying the motor powers.
        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

        // Updating the debug information.
        odometry.debug();
        follower.debug();

        // Retrieving and displaying the debug information.
        DebugPacket[] loggerPackets = logger.dump();

        for (DebugPacket packet : loggerPackets) {
            telemetry.addData(packet.getHeader().getIdentifier(), packet.getValue());
        }
    }
}
