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
  }
});