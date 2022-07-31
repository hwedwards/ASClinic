// Construct the URL to the host
// i.e., to the computer that runs the robot
// -----------------------------------------
const url_to_host = new URL(window.location.href);
url_to_host.protocol = "ws";
url_to_host.port = 8080;
url_to_host.pathname = "/";
// Hard-code a URL for development purposes
//url_to_host.href = "ws://192.168.1.26:9090"
// Log the URL being used:
console.log("url.href = ",url_to_host.href);

//document.getElementById("urlOfHostLabel").innerHTML = url_to_host.href;


// Heartbeat timer
// ---------------
const heartbeart_interval_in_milliseconds = 1000;
var heartbeat_interval_id = setInterval(publishHeartbeat, heartbeart_interval_in_milliseconds);
clearInterval(heartbeat_interval_id);


// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros(
{
	//url : "ws://localhost:9090"
	url : url_to_host.href
});

ros.on("connection", function()
{
	console.log("Connected to websocket server.");
	updateWebSocketStatusLabel("connected");
	// Start the heartbeat publishing
	heartbeat_interval_id = setInterval(publishHeartbeat, heartbeart_interval_in_milliseconds);
});

ros.on("error", function(error)
{
	console.log("Error connecting to websocket server: ", error);
	updateWebSocketStatusLabel("experienced an error while connecting, error text: " + error);
});

ros.on("close", function()
{
	console.log("Connection to websocket server closed.");
	updateWebSocketStatusLabel("connection is closed");
	// Stop the heartbeat
	clearInterval(heartbeat_interval_id);
});

function updateWebSocketStatusLabel(status_as_string)
{
	document.getElementById("webSocketStatusLabel").innerHTML = "<b>" + status_as_string + "</b>";

	document.getElementById("urlOfHostLabel").innerHTML = url_to_host.href;
}


// ROS Bridge connect and disconnect
// ---------------------------------
function rosbridgeConnect()
{
	ros.connect(url_to_host.href);
}

function rosbridgeClose()
{
	ros.close();
}


// Base namespace
// --------------
const base_namespace = "/asc";


// Heartbeat Publisher
// -------------------
var heartbeat = new ROSLIB.Topic(
{
	ros : ros,
	name : base_namespace + "/" + "web_interface_heartbeat",
	messageType : "std_msgs/Bool"
});

function publishHeartbeat()
{
	var heartbeat_message = new ROSLIB.Message(
	{
		data : true,
	});
	heartbeat.publish(heartbeat_message);
	console.log("Heartbeat message published.");
}


// Publisher variables
// -------------------
var inc_motor_duty_cycle = new ROSLIB.Topic(
{
	ros : ros,
	name : base_namespace + "/" + "increment_motor_duty_cycle_from_web_interface",
	messageType : "geometry_msgs/Vector3"
});

var set_motor_duty_cycle = new ROSLIB.Topic(
{
	ros : ros,
	name : base_namespace + "/" + "set_motor_duty_cycle_from_web_interface",
	messageType : "geometry_msgs/Vector3"
});

// Publish functions
// -----------------
function sendIncrementSpeedMessage(inc_left,inc_right)
{
	var vector3_message = new ROSLIB.Message(
	{
		x : inc_left,
		y : inc_right,
		z : 0.0
	});
	inc_motor_duty_cycle.publish(vector3_message);
	console.log("Increment duty cycle message published with (left,right) increment = ( " + inc_left + " , " + inc_right + " )" );
}

function sendSpeedMessage(speed_left,speed_right)
{

	var vector3_message = new ROSLIB.Message(
	{
		x : speed_left,
		y : speed_right,
		z : 0.0
	});
	set_motor_duty_cycle.publish(vector3_message);
	console.log("Set duty cycle message published with (left,right) percentage = ( " + speed_left + " , " + speed_right + " )" );
}


// Subscriber variables
// --------------------
var motor_duty_cycle_subscriber = new ROSLIB.Topic({
	ros : ros,
	name : base_namespace + "/" + "current_motor_duty_cycle_for_web_interface",
	messageType : "geometry_msgs/Vector3"
});

// Subscriber callback functions
// -----------------------------
motor_duty_cycle_subscriber.subscribe(function(message)
{
	console.log("Received message on " + motor_duty_cycle_subscriber.name + ", with data (left,right) = ( " + message.x + " , " + message.y + " )" );
	//listener.unsubscribe();

	document.getElementById("leftWheelCurrentDutyCycle").innerHTML = message.x;
	document.getElementById("rightWheelCurrentDutyCycle").innerHTML = message.y;
});

