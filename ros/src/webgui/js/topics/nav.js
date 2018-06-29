var nav = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/navigation',
    messageType : '/navigation/nav_request'
  });

  nav.subscribe(function(message) {
    //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
    $('#nav_depth').text(message.depth);
    $('#yaw_rate').text(message.yaw_rate);
    $('#forward_velocity').text(message.forwards_velocity);
    $('#sideways_velocity').text(message.sideways_velocity);
    });