
joystick_node = null;

window.addEventListener("gamepadconnected", function(e) {
  gp = navigator.getGamepads()[e.gamepad.index];
  console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    gp.index, gp.id,
    gp.buttons.length, gp.axes.length);

  // Publishing a Topic
  // ------------------
  joystick = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/joystick',
    messageType : 'navigation/joystick'
  });
});

window.addEventListener("gamepaddisconnected", function(e) {
  console.log("Gamepad disconnected from index %d: %s",
    e.gamepad.index, e.gamepad.id);
});

function convert_gameController_to_json(gp){
  var buttons = [];
  for (i = 0; i < gp.buttons.length; i++){
    var val = gp.buttons[i];
    var pressed = val == 1.0;
    if (typeof(val) == "object") {
      pressed = val.pressed;
      val = val.value; 
    }
    buttons.push(pressed);
  }
  var axes = [];
  for (i = 0; i < gp.axes.length; i++) {
    var val = gp.axes[i].toFixed(4);
    axes.push(parseInt(val*100));
  }
  return {
    buttons:buttons,
    axes:axes,
    id:gp.id,
  }
}

function controller_update_loop() {
  if(document.getElementById('controller_enable').checked){

    var gps = navigator.getGamepads();
    for(var c = 0; c < gps.length; gps++){
      if(gps[c].id.localeCompare("Logitech WingMan Attack 2 (Vendor: 046d Product: c20d)") < 20){
        var info = new ROSLIB.Message(convert_gameController_to_json(gps[c]));
        //console.log(info);
        joystick.publish(info);
      }
    }
  }
}   

setInterval(controller_update_loop, 100);
