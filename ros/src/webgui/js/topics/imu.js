var imu = new ROSLIB.Topic({
    ros : ros,
    name : '/imu/imu_sensor',
    messageType : 'peripherals/imu'
  });

  imu.subscribe(function(message) {
    //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
    $('#stabilised_magnetic_field_x').text(message.stabilised_magnetic_field.x.toFixed(2));
    $('#stabilised_magnetic_field_y').text(message.stabilised_magnetic_field.y.toFixed(2));
    $('#stabilised_magnetic_field_z').text(message.stabilised_magnetic_field.z.toFixed(2));

    $('#stabilised_acceleration_x').text(message.stabilised_acceleration.x.toFixed(2));
    $('#stabilised_acceleration_y').text(message.stabilised_acceleration.y.toFixed(2));
    $('#stabilised_acceleration_z').text(message.stabilised_acceleration.z.toFixed(2));

    $('#compensated_angular_rate_x').text(message.compensated_angular_rate.x.toFixed(2));
    $('#compensated_angular_rate_y').text(message.compensated_angular_rate.y.toFixed(2));
    $('#compensated_angular_rate_z').text(message.compensated_angular_rate.z.toFixed(2));

    $('#magnetic_field_x').text(message.magnetic_field.x.toFixed(2));
    $('#magnetic_field_y').text(message.magnetic_field.y.toFixed(2));
    $('#magnetic_field_z').text(message.magnetic_field.z.toFixed(2));
    
    $('#acceleration_x').text(message.acceleration.x.toFixed(2));
    $('#acceleration_y').text(message.acceleration.y.toFixed(2));
    $('#acceleration_z').text(message.acceleration.z.toFixed(2));

    $('#angular_rate_x').text(message.angular_rate.x.toFixed(2));
    $('#angular_rate_y').text(message.angular_rate.y.toFixed(2));
    $('#angular_rate_z').text(message.angular_rate.z.toFixed(2));

    $('#euler_angles_roll').text(message.euler_angles.roll.toFixed(2));
    $('#euler_angles_pitch').text(message.euler_angles.pitch.toFixed(2));
    $('#euler_angles_yaw').text(message.euler_angles.yaw.toFixed(2));

    $('#imu_temp').text(Math.round(message.temperature));
  });
