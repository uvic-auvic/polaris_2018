keyboard = new ROSLIB.Topic({
    ros : ros,
    name : '/keyboard',
    messageType : 'navigation/keyboard'
});

var keysDown = {

}

var ALLOWED_KEY_CODES = [65, 68, 83, 87];

function send_keyboard_msg(keys){
    var info = new ROSLIB.Message({
        pressed_key_codes:keys,
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
        var keys = [];
        for(i in keysDown){
            i = parseInt(i);
            if(keysDown[i] == true && ALLOWED_KEY_CODES.indexOf(i) != -1){
                keys.push(i);
            }
        }
    }else{
        keysDown = {};
    }
    send_keyboard_msg(keys);
}

setInterval(keyboard_update_loop, 100);
