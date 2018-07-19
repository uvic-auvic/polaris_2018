var imu_reset = new ROSLIB.Service({
    ros : ros,
    name : '/imu/set_velocity',
    messageType : '/peripherals/set_vel'
});

$("#imu_reset").click(function(){
    var request = new ROSLIB.ServiceRequest({
        velocity:{
            x:0.0,
            y:0.0,
            z:0.0
        }
    });

    imu_reset.callService(request);
});

