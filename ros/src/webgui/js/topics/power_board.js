var power_board = new ROSLIB.Topic({
    ros : ros,
    name : '/power_board/power_board_data',
    messageType : 'peripherals/powerboard'
  });

  power_board.subscribe(function(message) {
    //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
    $('#pb_batt_1_volt').text((message.voltage_battery_1)/1000.0);
    $('#pb_batt_2_volt').text((message.voltage_battery_2)/1000.0);
    $('#pb_motor_curr').text(message.current_motors);
    $('#pb_system_curr').text(message.current_system);
    $('#pb_temperature').text(Math.round(message.temperature));
    $('#pb_rh').text(message.humidity);
    $('#pb_internal_pressure').text(Math.round(message.internal_pressure));
    $('#pb_external_pressure').text(Math.round(message.external_pressure));
  });