var imu_reset = new ROSLIB.Service({
    ros : ros,
    name : '/power_board/PowerEnable',
    messageType : '/peripherals/power_enable'
});

$("#imu_reset").click(function(){
    var request = new ROSLIB.ServiceRequest({
        vector:{
            x:0,
            y:0,
            z:0
        }
    });

    imu_reset.callService(request);
});

