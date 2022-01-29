package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele

import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.barcode.BarcodeConstants

class TuneOpenCv : BaseOpMode() {
    override fun setupLoop() {
        val (leftThreshold, middleThreshold) = robot.barcode.analysis
        BarcodeConstants.leftBox.threshold = leftThreshold
        BarcodeConstants.middleBox.threshold = middleThreshold
    }
}