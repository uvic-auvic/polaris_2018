var depth = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/depth_control_info/',
    messageType : '/navigation/depth_info/'
  });

depth.subscribe(function(message){
    $("#desired_depth").text(message.desired_depth);
    $("#current_depth").text(message.current_depth);
})