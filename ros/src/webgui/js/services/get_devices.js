var mc = new ROSLIB.Service({
    ros : ros,
    name : '/serial_manager/GetAllDevices',
    messageType : '/monitor/serial_manager'
});

$("#reset_device_list").click(function(){
    update_device_list();
});

 function update_device_list(){

    var request = new ROSLIB.ServiceRequest({});

    mc.callService(request, function(message){
        success(message);
    });
 }

function success(message){
    $("#device_list").empty();
    for(var c = 0; c < message.devices.length; c++){
        var item = message.devices[c];
        $("#device_list").append("<tr><td>" + item.name + "</td><td>" + item.port + "</td></tr>");
    }
}

update_device_list();
