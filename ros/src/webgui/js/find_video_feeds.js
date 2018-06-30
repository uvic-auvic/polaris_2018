ros = new ROSLIB.Ros();
ros.connect("ws://"+ window.location.hostname + ":9090");

function get_topic_success(message){ 
    $("#main-video-selector").append("<option id='' selected> None </option>");
    for(i=0; i < message.length; i++){
        $("#main-video-selector").append("<option id='http://" + window.location.hostname + ":8080/stream?topic=" + message[i] + "&type=ros_compressed'>" + message[i] + "</option>");
    }

    /*
    .click() does not work for option elements in chrome. 
    Used .change() and monitored the whole dropdown seeing if :selected moved
    */
    $("#main-video-selector").change(function(){
        var url = $(this).children(":selected")[0].id;
        $("#main-video-stream").attr('src', url);

        console.log($(this).children(":selected").html());
    });

}

function get_topic_fail(message){ 
    console.log("fail: " + message);
}

ros.getTopicsForType("sensor_msgs/Image", get_topic_success, get_topic_fail);

