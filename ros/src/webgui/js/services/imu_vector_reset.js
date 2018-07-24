$("#imu_reset").click(function(){
    var imu_reset = new ROSLIB.Service({
        ros : ros,
        name : '/imu/set_velocity',
        messageType : '/peripherals/set_vel'
    });

    $("#imu_x_vel_opt").val("");
    $("#imu_y_vel_opt").val("");
    $("#imu_z_vel_opt").val("");

    var request = new ROSLIB.ServiceRequest({
        velocity:{
            x:0,
            y:0,
            z:0
        }
    });

    imu_reset.callService(request);
});

$("#imu_set").click(function(){
    var imu_reset = new ROSLIB.Service({
        ros : ros,
        name : '/imu/set_velocity',
        messageType : '/peripherals/set_vel'
    });

    var x_v = Number($("#imu_x_vel_opt").val());
    var y_v = Number($("#imu_y_vel_opt").val());
    var z_v = Number($("#imu_z_vel_opt").val());

    if(x_v != NaN && y_v != NaN && z_v != NaN){
        var request = new ROSLIB.ServiceRequest({
            velocity:{
                x:x_v,
                y:y_v,
                z:z_v
            }
        });
    
        imu_reset.callService(request);
    }

    
});