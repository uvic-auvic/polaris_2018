var ros_out = new ROSLIB.Topic({
    ros : ros,
    name : '/rosout',
    messageType : 'rosgraph_msgs/Log'
});

compare_height = 0;
max_scroll_height = 0;
scroll_pos_from_top = 0;

ros_out.subscribe(function(message) {
    var para = document.createElement("p");
    para.innerHTML = message.msg;
    var node = $('#ros_out_node').append(para);

    //console Auto scrolling section

    compare_height = node.height();
    max_scroll_height = max_scroll_height + $(para).outerHeight();

    scroll_pos_from_top = node.scrollTop();
    var diff = (max_scroll_height - scroll_pos_from_top);
    if( diff > compare_height && diff < 500){
        node.scrollTop(max_scroll_height);
    }
}); 