package org.firstinspires.ftc.teamcode.disabledSamples;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Debuggers;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Point;
import com.github.bouyio.cyancore.localization.TankKinematics;
import com.github.bouyio.cyancore.pathing.Path;
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
 *     This class demonstrates path following in CyanFTC.
 *     The way path following differs from point sequencing is that it much smoother.
 *     This is achieved by dynamically adjusting the targeted point.
 * </p>
 * */
@Disabled
@TeleOp()
public class PathFollowing extends OpMode {

    // TODO: Set the path following configurations.
    // NOTE: Tuning look ahead distance is very simple.
    // The greater the value is smoother the following gets
    // at the expense of some accuracy.
    final double LOOK_AHEAD_DISTANCE = 000000000;

    // NOTE: Once again the greater the value of admissible
    // point error is the smoother the following gets at the
    // expense of some accuracy.
    final double ADMISSIBLE_POINT_ERROR = 000000000;

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

    /**The sequence following system.*/
    PathFollower follower;

    // The motors we will be using.
    DcMotor leftMotor;
    DcMotor rightMotor;

    /**The logging system.*/
    Logger logger;

    /**The sequence used for point following demonstration.*/
    Path path;

    @Override
    public void init() {
        // Initializing hardware.
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializing localization system.
        TankKinematics.MeasurementProvider measurementProvider = new TankKinematics.MeasurementProvider(
                leftMotor::getCurrentPosition,
                rightMotor::getCurrentPosition,
                TICKS_TO_LINEAR_DISTANCE
        );

        odometry = new TankKinematics(TRACK_WIDTH, Distance.DistanceUnit.CM, measurementProvider);

        // Initializing the sequence following system.

        // Initializing the steering PID controller coefficients.
        // TODO: Tune the controller and set the actual coefficient.
        PIDCoefficients coefficients = new PIDCoefficients();
        coefficients.kP = 0000000;
        coefficients.kD = 0000000;
        coefficients.kI = 0000000;

        // Initializing the sequence follower itself.
        follower = new PathFollower(odometry);

        // Configuring follower.
        // TODO: Set the preferred distance unit.
        follower.purePursuitSetUp(LOOK_AHEAD_DISTANCE, ADMISSIBLE_POINT_ERROR);
        follower.setDistanceUnitOfMeasurement(Distance.DistanceUnit.CM);

        // Initializing the sequence.
        // TODO: Set the actual coordinates of the points.
        // TODO: Use as many point as it is necessary.
        path = new Path(
                new Point(0000000, 0000000),
                new Point(0000000, 0000000),
                new Point(0000000, 0000000),
                new Point(0000000, 0000000),
                new Point(0000000, 0000000)
        );

        // Initializing localization system logging.
        logger = Debuggers.getGlobalLogger();
        odometry.attachLogger(logger);
        follower.attachLogger(logger);

    }

    @Override
    public void loop() {

        // Point following code.

        // Setting the target.
        follower.followPath(path);

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
