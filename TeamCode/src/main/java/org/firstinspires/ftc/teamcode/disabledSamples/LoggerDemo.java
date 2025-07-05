package org.firstinspires.ftc.teamcode.disabledSamples;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.debugger.Debuggers;
import com.github.bouyio.cyancore.debugger.formating.MessageLevel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * <p>This class is a demonstration of the logging functionality of CyanFTC.<p/>
 * @see com.github.bouyio.cyancore.debugger.Logger
 * */
@Disabled
@TeleOp()
public class LoggerDemo extends OpMode {

    Logger logger;

    @Override
    public void init() {
        // Initializing the default logger.
        logger = Debuggers.getGlobalLogger();
    }

    @Override
    public void loop() {
        // Logging messages and values.
        logger.logMessage("Log a simple debug message");
        logger.logMessage(MessageLevel.WARNING, "Log a message with a flag");
        logger.logValue("Button", gamepad1.a); // Log a value with a specified header.

        // Retrieving logger data.
        DebugPacket[] loggerPackets = logger.dump();

        // Outputting logger data to the telemetry system.
        for (DebugPacket packet : loggerPackets) {
            telemetry.addData(packet.getHeader().getIdentifier(), packet.getValue());
        }
    }
}
