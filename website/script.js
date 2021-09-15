var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function () {
    console.log("Connected");
});

ros.on('error', function (error) {
    console.log("Error");
});

ros.on('close', function () {
    console.log("Closed");
});


var cmdvel = new ROSLIB.Topic({
    ros: ros,
    name: "/cmd_vel",
    messageType: "geometry_msgs/Twist"
});


var twist = new ROSLIB.Message({
    linear: {
        x: 0.5,
        y: 0.0,
        z: 0.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: 0.5
    }
});

var stop = new ROSLIB.Message({
    linear: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    },
    angular: {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
});


$("#b1").click(function () {
    console.log("publishing start");
    cmdvel.publish(twist);
});

$("#b2").click(function () {
    console.log("publishing stop");
    cmdvel.publish(stop);
});


