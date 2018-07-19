var imu = new ROSLIB.Topic({
    ros : ros,
    name : '/imu/imu_sensor',
    messageType : 'peripherals/imu'
  });

  imu.subscribe(function(message) {
    //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
    $('#stabilised_magnetic_field_x').text(message.stabilised_magnetic_field.x);
    $('#stabilised_magnetic_field_y').text(message.stabilised_magnetic_field.y);
    $('#stabilised_magnetic_field_z').text(message.stabilised_magnetic_field.z);

    $('#stabilised_acceleration_x').text(message.stabilised_acceleration.x);
    $('#stabilised_acceleration_y').text(message.stabilised_acceleration.y);
    $('#stabilised_acceleration_z').text(message.stabilised_acceleration.z);

    $('#compensated_angular_rate_x').text(message.compensated_angular_rate.x);
    $('#compensated_angular_rate_y').text(message.compensated_angular_rate.y);
    $('#compensated_angular_rate_z').text(message.compensated_angular_rate.z);

    $('#magnetic_field_x').text(message.magnetic_field.x);
    $('#magnetic_field_y').text(message.magnetic_field.y);
    $('#magnetic_field_z').text(message.magnetic_field.z);
    
    $('#acceleration_x').text(message.acceleration.x);
    $('#acceleration_y').text(message.acceleration.y);
    $('#acceleration_z').text(message.acceleration.z);

    $('#angular_rate_x').text(message.angular_rate.x);
    $('#angular_rate_y').text(message.angular_rate.y);
    $('#angular_rate_z').text(message.angular_rate.z);

    $('#velocity_x').text(message.velocity.x);
    $('#velocity_y').text(message.velocity.y);
    $('#velocity_z').text(message.velocity.z);

    $('#euler_angles_x').text(message.euler_angles.roll);
    $('#euler_angles_y').text(message.euler_angles.pitch);
    $('#euler_angles_z').text(message.euler_angles.yaw);

    $('#imu_temp').text(Math.round(message.temperature));
  });
