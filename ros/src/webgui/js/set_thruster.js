function set_thruster_color(thruster, rpm, base){
    var color;
    if(rpm > 0){
        color = '#008000';
    }else{
        color = '#FF0000';
    }
    rpm = Math.abs(rpm);
    //255 is the max arguement of transparency
    var transparency = Math.round(Math.min((rpm * 255) / base, 255));
    transparency = ("00"+transparency.toString(16)).slice(-2);
    color = color + transparency;
    $('#'+thruster).css("background-color", color);
}