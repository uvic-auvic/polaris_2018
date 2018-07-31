var mc = new ROSLIB.Service({
    ros : ros,
    name : '/power_board/PowerEnable',
    messageType : '/peripherals/power_enable'
});

$("#power_send").click(function(){
    console.log("clicked");
    var motor_pwr_enable = document.getElementById('pb_mc_power').checked;
    var rail_5V_pwr_enable = document.getElementById('pb_5v').checked;
    var rail_12V_9V_pwr_enable = document.getElementById('pb9_12_v').checked;
    var parallel_batteries_enable = document.getElementById('pb_bridge_batteries').checked;

    var request = new ROSLIB.ServiceRequest({
        motor_pwr_enable:motor_pwr_enable,
        rail_5V_pwr_enable:rail_5V_pwr_enable,
        rail_12V_9V_pwr_enable:rail_12V_9V_pwr_enable,
        parallel_batteries_enable:parallel_batteries_enable
    });

    mc.callService(request);
});

