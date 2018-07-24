var nav = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/navigation',
    messageType : '/navigation/nav_request'
  });

nav.subscribe(function(message) {
  //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
  $('#nav_depth').text(message.depth.toFixed(2));
  $('#yaw_rate').text(message.yaw_rate.toFixed(2));
  $('#forward_velocity').text(message.forwards_velocity.toFixed(2));
  $('#sideways_velocity').text(message.sideways_velocity.toFixed(2));
});