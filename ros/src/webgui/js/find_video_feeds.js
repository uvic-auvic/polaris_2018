ros = new ROSLIB.Ros();
ros.connect("ws://localhost:9090");

function get_topic_success(message){ 
    console.log("success");
    for(i=0; i < message.length; i++){
        $("#main-video-selector").append("<div class='radio'><label id=" + message[i] + "><input id=" + message[i] + " type='radio' name='optradio'>" + message[i] + "</label></div>");
    }

    $("#main-video-selector div label").click(function(event){
        var id = event.target.id
        var url = "http://localhost:8080/stream?topic=" + id + "&type=ros_compressed";
        $("#main-video-stream").attr('src', url);
    });
}

function get_topic_fail(message){ 
    console.log("fail: " + message);
}

ros.getTopicsForType("sensor_msgs/Image", get_topic_success, get_topic_fail);

