keyboard = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/keyboard',
    messageType : 'navigation/keyboard'
});

var keysDown = {

}

function send_keyboard_msg(keys){
    var info = new ROSLIB.Message({
        W_pressed:keys[0],
        A_pressed:keys[1],
        S_pressed:keys[2],
        D_pressed:keys[3]
    });

    keyboard.publish(info);
}

$(document).keydown(function(event){
    keysDown[event.originalEvent.keyCode] = true;
});

$(document).keyup(function(event){
    if(keysDown.hasOwnProperty(event.originalEvent.keyCode)){
        delete keysDown[event.originalEvent.keyCode];
    }
});

function keyboard_update_loop(){
    if(document.getElementById('keypress_enable').checked){
        send_keyboard_msg([ keysDown[87],   // W 
                            keysDown[65],   // A
                            keysDown[83],   // S
                            keysDown[68]]); // D
    }else{
        keysDown = {};
    }
}

setInterval(keyboard_update_loop, 100);
