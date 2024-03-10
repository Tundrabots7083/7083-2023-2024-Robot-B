package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.test.ProgrammingBoard;
import org.firstinspires.ftc.teamcode.test.Test;

import java.util.List;

@TeleOp(name = "Test Programming Board", group = "test")
public class TestProgrammingBoard extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    List<Test> tests;
    boolean wasDown, wasUp;
    int testNum;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        board.init(hardwareMap);
        tests = board.getTests();
    }

    @Override
    public void loop() {
        // move up in the list of test
        if (gamepad1.dpad_up && !wasUp) {
            testNum--;
            if (testNum < 0) {
                testNum = tests.size() - 1;
            }
        }
        wasUp = gamepad1.dpad_up;

        // move down in the list of tests
        if (gamepad1.dpad_down && !wasDown) {
            testNum++;
            if (testNum >= tests.size()) {
                testNum = 0;
            }
        }
        wasDown = gamepad1.dpad_down;

        // Put instructions on the telemetry
        telemetry.addLine("Use Up and Down on D-pad to cycle through choices");
        telemetry.addLine("Use the Gamepad to run test");
        // put the test on the telemetry
        Test currTest = tests.get(testNum);
        telemetry.addData("Test:", currTest.getDescription());
        // run or don’t run based on the gamepad settings
        currTest.run(gamepad1, telemetry);
    }
}
