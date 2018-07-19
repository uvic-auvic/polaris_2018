var ctrl_sys_en = new ROSLIB.Service({
    ros : ros,
    name : '/control_system/ControlSysEnable',
    messageType : '/navigation/control_en'
});

$("#control_system_settings").click(function(){
    console.log("clicked");
    var b_vel_x_enable = document.getElementById('vel_x_enable').checked;
    var b_vel_y_enable = document.getElementById('vel_y_enable').checked;
    var b_vel_z_enable = document.getElementById('vel_z_enable').checked;
    var b_pitch_enable = document.getElementById('pitch_enable').checked;
    var b_roll_enable = document.getElementById('roll_enable').checked;
    var b_yaw_enable = document.getElementById('yaw_enable').checked;

    var request = new ROSLIB.ServiceRequest({
        vel_x_enable:b_vel_x_enable,
        vel_y_enable:b_vel_y_enable,
        vel_z_enable:b_vel_z_enable,
        pitch_enable:b_pitch_enable,
        roll_enable:b_roll_enable,
        yaw_enable:b_yaw_enable
    });

    ctrl_sys_en.callService(request);
});

