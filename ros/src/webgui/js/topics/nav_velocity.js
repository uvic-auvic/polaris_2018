var nav = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/velocity_vectors',
    messageType : '/navigation/nav'
  });

nav.subscribe(function(message) {
  //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
  $('#cs_velocity_x').text(message.direction.x.toFixed(2));
  $('#cs_velocity_y').text(message.direction.y.toFixed(2));
  $('#cs_velocity_z').text(message.direction.z.toFixed(2));
  $('#cs_ang_vel_roll').text(message.orientation.roll.toFixed(2));
  $('#cs_ang_vel_pitch').text(message.orientation.pitch.toFixed(2));
  $('#cs_ang_vel_yaw').text(message.orientation.yaw.toFixed(2));
});