var calibrate_depth = new ROSLIB.Service({
    ros : ros,
    name : '/nav/CalibrateSurfaceDepth',
    messageType : '/peripherals/avg_data'
});

$("#depth_calibrate").click(function(){
    var request = new ROSLIB.ServiceRequest({
        acq_rate:10,
        acq_count:100
    });

    calibrate_depth.callService(request);
});

