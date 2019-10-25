package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.ftc9974.thorcore.OpModeEnhanced;

import java.util.Iterator;
import java.util.Locale;
import java.util.Map;

@TeleOp(name = "Debugger")
public class Debugger extends OpModeEnhanced {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addLine("=== Motors ===");
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            telemetry.addData("Name", entry.getKey());
            DcMotor value = entry.getValue();
            telemetry.addData("Position", value.getCurrentPosition());
            telemetry.addData("Port", value.getPortNumber());
            telemetry.addData("Type", value.getMotorType().toString());
            telemetry.addData("Controller Manufacturer", value.getController().getManufacturer());
        }
        telemetry.addLine("=== Servos ===");
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            telemetry.addData("Name", entry.getKey());
            Servo value = entry.getValue();
            telemetry.addData("Position", value.getPosition());
            telemetry.addData("Port", value.getPortNumber());
        }
        telemetry.addLine("=== Touch Sensors ===");
        for (Map.Entry<String, TouchSensor> entry : hardwareMap.touchSensor.entrySet()) {
            telemetry.addData("Name", entry.getKey());
            telemetry.addData("Is Pressed", entry.getValue().isPressed() ? "yes" : "no");
        }
        telemetry.addLine("=== Distance Sensors ===");
        for (DistanceSensor distanceSensor : hardwareMap.getAll(DistanceSensor.class)) {
            String name = "No name found";
            Iterator<String> nameIterator = hardwareMap.getNamesOf(distanceSensor).iterator();
            if (nameIterator.hasNext()) {
                name = nameIterator.next();
            }
            telemetry.addData("Name", name);
            telemetry.addData("Distance (mm)", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance (m)", distanceSensor.getDistance(DistanceUnit.METER));
            telemetry.addData("Distance (in)", distanceSensor.getDistance(DistanceUnit.INCH));
        }
        telemetry.addLine("=== Analog Sensors ===");
        for (AnalogInput analogInput : hardwareMap.getAll(AnalogInput.class)) {
            String name = "No name found";
            Iterator<String> nameIterator = hardwareMap.getNamesOf(analogInput).iterator();
            if (nameIterator.hasNext()) {
                name = nameIterator.next();
            }
            telemetry.addData("Name", name);
            telemetry.addData("Voltage", analogInput.getVoltage());
            telemetry.addData("Manufacturer", analogInput.getManufacturer());
        }
        telemetry.addLine("=== All devices ===");
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping : hardwareMap.allDeviceMappings) {
            telemetry.addLine(String.format(Locale.getDefault(), "=== %s ===", deviceMapping.getDeviceTypeClass().getSimpleName()));
            for (Map.Entry<String, ? extends HardwareDevice> entry : deviceMapping.entrySet()) {
                telemetry.addData("Name", entry.getKey());
                HardwareDevice device = entry.getValue();
                telemetry.addData("Connection info", device.getConnectionInfo());
                telemetry.addData("Manufacturer", device.getManufacturer());
                telemetry.addData("Version", device.getVersion());
            }
        }
    }
}
