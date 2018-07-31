var power_board = new ROSLIB.Topic({
    ros : ros,
    name : '/power_board/power_board_data',
    messageType : 'peripherals/powerboard'
  });

function set_battery_voltage_background(id, volt)
{
  if(volt  < 18)
  {
    $(id).css('background-color', 'red');
  } 
  else if(volt < 22)
  {
    $(id).css('background-color', 'yellow');
  }
  else
  {
    $(id).css('background-color', '');
  }
}

  power_board.subscribe(function(message) {
    //console.log('Received message on ' + depth_node.name + ': ' + message.temperature);
    var volt1 = ((message.voltage_battery_1)/1000.0);
    var volt2 = ((message.voltage_battery_2)/1000.0);
    $('#pb_batt_1_volt').text(volt1);
    set_battery_voltage_background('#pb_batt_1_volt_row', volt1);
    $('#pb_batt_2_volt').text(volt2);
    set_battery_voltage_background('#pb_batt_2_volt_row', volt2);
    $('#pb_motor_curr').text(message.current_motors);
    $('#pb_system_curr').text(message.current_system);
    $('#pb_temperature').text(Math.round(message.temperature));
    $('#pb_rh').text(message.humidity);
    $('#pb_internal_pressure').text(Math.round(message.internal_pressure));
    $('#pb_external_pressure').text(message.external_pressure.toFixed(3));
  });