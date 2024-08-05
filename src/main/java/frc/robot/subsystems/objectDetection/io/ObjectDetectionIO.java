package frc.robot.subsystems.objectDetection.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class ObjectDetectionIO extends IOBase {
    public final Supplier<PhotonPipelineResult> result = fields.addObject("result", this::getResult);
    public final BooleanSupplier hasTarget = fields.addBoolean("hasTarget", this::hasTarget);
    public final Supplier<PhotonTrackedTarget> bestTarget = fields.addObject("bestTarget", this::getBestTarget); // להעביר
    public final DoubleSupplier yawFromTarget = fields.addDouble("getYawFromTarget", this::getYawFromTarget); // לpose3d

    public ObjectDetectionIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs

    protected abstract PhotonPipelineResult getResult();

    protected abstract boolean hasTarget();

    protected abstract PhotonTrackedTarget getBestTarget();

    protected abstract double getYawFromTarget();

}
