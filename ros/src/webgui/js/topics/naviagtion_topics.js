var power_board = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/joystick',
    messageType : 'peripherals/powerboard'
});

var nav = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/navigation',
    messageType : '/navigation/nav_request'
});

var nav = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/thrust',
    messageType : '/navigation/nav_request'
});

var nav = new ROSLIB.Topic({
    ros : ros,
    name : '/nav/rpms',
    messageType : '/peripherals/rpms'
});