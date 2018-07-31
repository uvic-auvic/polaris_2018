motor_lookup = {};
motor_rpms_act = {};
motor_rpms_set = {};
motors_pwms = {};

function recv_motor_eunms(message){
    $("#rpm-table-body").text("");
    

    for(key in message.motors)
    {
        var motor_name = key;
        if(motor_name.slice(-4) == "_idx")
        {
            motor_name = motor_name.substr(0, (motor_name.length - 4));
        }
        motor_lookup[message.motors[key]] = motor_name;
        motor_name = motor_name.replace(/_/g, ' ');
        
        $("#rpm-table-body").append("<tr><td>" + motor_name + "</td><td><span id='rpm-act-"+message.motors[key]+"'></span> RPM</td><td><span id='rpm-set-"+message.motors[key]+"'></span> RPM</td></tr>");
        $("#pwm-table-body").append("<tr><td>" + motor_name + "</td><td><span id='pwm-"+message.motors[key]+"'></span> Hz</td></tr>");  
    }
}

function get_motor_enums(){
    var motor_eunums = new ROSLIB.Service({
        ros : ros,
        name : '/motor_controller/getMotorEnums',
        messageType : '/peripherals/get_motor_enums'
    });

    var request = new ROSLIB.ServiceRequest({});


    motor_eunums.callService(request, function(message){
        recv_motor_eunms(message);
    });
}

get_motor_enums();

var rpms_act = new ROSLIB.Topic({
    ros : ros,
    name : '/motor_controller/MotorsRPMs/',
    messageType : '/peripherals/rpms/'
});

rpms_act.subscribe(function(message) {
  //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
  for(var k=0; k<message.rpms.length;k++)
  {
    $("#rpm-act-"+k).text(message.rpms[k]);
    motor_rpms_act[k] = message.rpms[k];
  }
});

var rpms_set = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/rpms',
    messageType : '/peripherals/rpms/'
});

rpms_set.subscribe(function(message) {
  //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
  for(var k=0; k<message.rpms.length;k++)
  {
    $("#rpm-set-"+k).text(message.rpms[k]);
    motor_rpms_set[k] = message.rpms[k];
  }
});

var pwm_out = new ROSLIB.Topic({
    ros : ros,
    name : '/rpm_control/pwms/',
    messageType : '/navigation/thrusts/'
});

pwm_out.subscribe(function(message) {
  //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
  for(var k=0; k<message.thruster_pwms.length;k++)
  {
    $("#pwm-"+k).text(message.thruster_pwms[k]);
    motors_pwms[k] = message.thruster_pwms[k];
  }
});

selector_option = $("#select-graphic").children(":selected")[0].id;

function update_thrust_visual(){
    
    $("#select-graphic").change(function(){
        selector_option = $(this).children(":selected")[0].id;
    });
    if(selector_option == "rpm-actual")
    {
        for(key in motor_lookup){
            var rpm = 0;
            if(motor_rpms_set[key] > 0){
                rpm = 1;
            }else{
                rpm = -1
            }
    
            if(motor_rpms_act[key] == null){
                return;
            }
            set_thruster_color(motor_lookup[key], motor_rpms_act[key] * rpm, 2000);
        }
    }
    else if(selector_option == "pwm-current")
    {
        for(key in motors_pwms)
        {
            set_thruster_color(motor_lookup[key], motors_pwms[key], 100);
        }
    }
    else if(selector_option == "rpm-set-point")
    {
        for(key in motor_rpms_set)
        {
            set_thruster_color(motor_lookup[key], motor_rpms_set[key], 2000);
        }
    }
    else
    {
        console.log("unknown selector option");
    }
}

setInterval(update_thrust_visual, 100);