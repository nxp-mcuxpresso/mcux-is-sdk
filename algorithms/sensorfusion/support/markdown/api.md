Sensor Fusion Core APIs         {#apis}
==================
Setup
-----
Function | Description 
---------|------------
[initializeControlPort] (@ref initializeControlPort()) |Initialize hardware resources associated with the control port
[initializeStatusSubsystem] (@ref initializeStatusSubsystem()) | Initialize hardware resources for the Status Subsystem
[initSensorFusionGlobals] (@ref initSensorFusionGlobals()) | Initialize structures for sensor fusion

Public Interface
----------------
Function | Description 
---------|------------
[sfg.applyPerturbation](@ref ApplyPerturbation(SensorFusionGlobals *sfg)) | Optional unit test function
[sfg.conditionSensorReadings](@ref conditionSensorReadings(SensorFusionGlobals *sfg)) | Process most recent sensor readings prior to running sensor fusion
[sfg.installSensor] (@ref installSensor( SensorFusionGlobals *sfg, PhysicalSensor *pSensor, uint16_t addr, uint16_t schedule, void *bus_driver, initializeSensor_t *initialize, readSensor_t *read)) | Install a sensor driver (init and read functions) for later use
[sfg.initializeFusionEngine] (@ref initializeFusionEngine(SensorFusionGlobals* sfg)) | Sensor Fusion startup routine
[sfg.readSensors](@ref readSensors(SensorFusionGlobals *sfg, uint16_t read_loop_counter)) | Take sensor readings
[sfg.runFusion](@ref runFusion(SensorFusionGlobals *sfg)) | Run sensor fusion code
[sfg.queueStatus](@ref queueStatus()) | Queue up a status change for next regularly scheduled update
[sfg.setStatus](@ref setStatus()) | Make immediate status change
[sfg.updateStatus](@ref updateStatus()) | Update status to equal queued status

Other
-----

Function | Description 
---------|------------
[sfg.pControlSubsystem->stream] (@ref CreateAndSendPackets()) | Stream samples and fusion results to the Sensor Fusion Toolbox. The control system streaming function has not been abstracted up to the sfg level, as it is application specific.
