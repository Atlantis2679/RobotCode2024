package frc.robot.subsystems.objectDetection.io;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.objectDetection.ObjectDetection;

public class ObjectDetectionIOCamera extends ObjectDetectionIO {
    private final PhotonCamera camera;

    public ObjectDetectionIOCamera(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        camera = new PhotonCamera("photonvision");
    }

    @Override
    protected PhotonPipelineResult getResult() {
        return camera.getLatestResult();
    }

    @Override
    protected boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }

    @Override
    protected PhotonTrackedTarget getBestTarget() {
        return camera.getLatestResult().getBestTarget();

    }

}
