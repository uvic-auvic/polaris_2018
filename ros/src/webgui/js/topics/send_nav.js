var send_nav = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/navigation',
    messageType : '/navigation/nav_request'
});

function reset_nav(){
    message = {
        depth:0,
        yaw_rate:0,
        forwards_velocity:0,
        sideways_velocity:0,
    }
    send_nav.publish(message);
}

$("#reset_nav").click(function(){
    reset_nav();
})

$("#send_manual_nav").click(function(){
    
    message = {
        depth:Number($("#depth_nav").val()),
        yaw_rate:Number($("#yaw_nav").val()),
        forwards_velocity:Number($("#x_nav").val()),
        sideways_velocity:Number($("#y_nav").val()),
    }
    send_nav.publish(message);
})



