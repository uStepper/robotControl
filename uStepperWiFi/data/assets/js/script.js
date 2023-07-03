// Websocket object to be used
var websocket;

var lastG1Command = [];
var joystickHandlerCnt = 0;

// Position returned from device
var pos = {
	x: 0.0, 
	y: 0.0,
	z: 0.0
};

var tempPos = {
	x: 0.0, 
	y: 0.0,
	z: 0.0
}

var recording = false;

// Flag to check if the last sent command was completed
var commandAck = false;

var xyJoystickActive = false;
var zJoystickActive = false;
var joystickActive = false;

// Keeping track of commands sent
var commandsSend = 0;

// Min interval in ms between each command 
var wsInterval = 100;

// Maximum position change (mm) per command
var stepSize = 20; 


// Element references to modify DOM
var statusBar 		= document.getElementById("comm-status");
var logElement 		= document.getElementById("log");
var servoElement 	= document.getElementById('servo')
var servoMin 		= document.getElementById('minServoValue');
var servoMax 		= document.getElementById('maxServoValue');
var playBackSpeed	= document.getElementById('playBackSpeed');
var pauseBtn		= document.getElementById('pause');
var playBtn			= document.getElementById('play');
var stopBtn			= document.getElementById('stop');
var recordBtn		= document.getElementById('record');
var recordLineBtn	= document.getElementById('recordLine');
var accelerationInput 		= document.getElementById('acceleration');
var brakeSelect = document.getElementById('brake'); 

var baseSensitivityInput 		= document.getElementById('sensitivityBase');
var elbowSensitivityInput 		= document.getElementById('sensitivityElbow');
var shoulderSensitivityInput 		= document.getElementById('sensitivityShoulder');

var baseHomingSpeedInput = document.getElementById('HomingSpeedBase'); 
var elbowHomingSpeedInput = document.getElementById('HomingSpeedElbow'); 
var shoulderHomingSpeedInput = document.getElementById('HomingSpeedShoulder'); 

var maxFeedrate = document.getElementById('maxFeedrate');
var minFeedrate = document.getElementById('minFeedrate');

var xDisplay		= document.getElementById('x-pos');
var yDisplay		= document.getElementById('y-pos');
var zDisplay		= document.getElementById('z-pos');
var homeBtn			= document.getElementById('homeBtn');
var uploadBtn		= document.getElementById('uploadBtn');
var uploadFile		= document.getElementById('uploadFile');
var uploadForm 		= document.getElementById('uploadForm');
var pumpBtn			= document.getElementById('pumpBtn');

var emergencyBtn	= document.getElementById('emergencyBtn');
var linesElement	= document.getElementById('lines');

var recordingElement = document.getElementById('recording');

var initPosition = false;
var getRecording = false;

var websocketCnt = 0;

var pumpState = false;
var silenceWebsocket = 0;

var fileUploadURL = "/upload";


xhttp = new XMLHttpRequest();

// Read contents of recording.txt every 3000 second or when prompted by record
xhttp.onreadystatechange = function() {
	
	if (this.readyState == 4 && this.status == 200) {

		lines = this.responseText.split('\n');
		if( lines.length == 0 ){
			linesElement.innerHTML = "<span class='line'>No lines recorded</span>";
		}

		linesElement.innerHTML = '';
		for(var i = 0;i < lines.length-1;i++){
		    linesElement.innerHTML += "<span class='linenum'>" + (i+1) + ":</span><span class='line'>" + lines[i] + "</span>";
		}

		document.getElementById('record-len').innerHTML = "("+ (lines.length-1) +")";
	}
};

function requestRecording(){
	xhttp.open("GET", "/recording.txt", true);
	if (xhttp.readyState){
		xhttp.send();
	}
}



uploadBtn.addEventListener('click', function(event){
	event.preventDefault()
	// Open up uploadFile 
	uploadFile.value = null;
	uploadFile.click();
});

// When a file is selected, UPLOAD
uploadFile.onchange = function(){

	var file = this.files[0];

	// files types allowed
	var allowed_types = [ 'text/plain' ];
	if(allowed_types.indexOf(file.type) == -1) {
		alert('File: Incorrect type');
		return;
	}

	// Max 2 MB allowed
	var max_size_allowed = 2*1024*1024
	if(file.size > max_size_allowed) {
		alert('File: Exceeded size 2MB');
		return;
	}

	// Rename file
	var fileData = new FormData();
	fileData.append('file', file, 'recording.txt');

	// Send file
	var request = new XMLHttpRequest();
	request.open("POST", fileUploadURL, true);
	request.send(fileData);

	request.onload = function() {
		// Load the recording into the GUI
		requestRecording();
	}
};

baseHomingSpeedInput.onchange = function(){
	var homingSpeed = parseInt(baseHomingSpeedInput.value);

	if(homingSpeed < 10)
	{
		homingSpeed = 10;
		baseHomingSpeedInput.value = homingSpeed;
	}

	if(homingSpeed > 100)
	{
		homingSpeed = 100;
		baseHomingSpeedInput.value = homingSpeed;
	}

	var command = [
		{name: "S", value: homingSpeed },
	];
	sendCommand("M21", command);
}

maxFeedrate.onchange = function(){

	if(parseInt(maxFeedrate.value) > 1000)
	{
		maxFeedrate.value = 1000;
	}
	else if(parseInt(maxFeedrate.value) < parseInt(minFeedrate.value))
	{
		maxFeedrate.value = parseInt(minFeedrate.value);
	}
}

minFeedrate.onchange = function(){

	if(parseInt(minFeedrate.value) < 10.0)
	{
		minFeedrate.value = 10.0;
	}
	else if(parseInt(minFeedrate.value) > parseInt(maxFeedrate.value))
	{
		minFeedrate.value = parseInt(maxFeedrate.value);
	}
}

elbowHomingSpeedInput.onchange = function(){
	var homingSpeed = parseInt(elbowHomingSpeedInput.value);

	if(homingSpeed < 10)
	{
		homingSpeed = 10;
		elbowHomingSpeedInput.value = homingSpeed;
	}

	if(homingSpeed > 100)
	{
		homingSpeed = 100;
		elbowHomingSpeedInput.value = homingSpeed;
	}

	var command = [
		{name: "S", value: homingSpeed },
	];
	sendCommand("M22", command);
}

shoulderHomingSpeedInput.onchange = function(){
	var homingSpeed = parseInt(shoulderHomingSpeedInput.value);

	if(homingSpeed < 10)
	{
		homingSpeed = 10;
		shoulderHomingSpeedInput.value = homingSpeed;
	}

	if(homingSpeed > 100)
	{
		homingSpeed = 100;
		shoulderHomingSpeedInput.value = homingSpeed;
	}

	var command = [
		{name: "S", value: homingSpeed },
	];
	sendCommand("M23", command);
}


// Add events for brake method select
brakeSelect.onchange = function(event) {
	var value = event.target.value;

	switch( value ){
		case "free":
			sendCommand("M14");
		break;

		case "cool":
			sendCommand("M15");
		break;

		case "hard":
			sendCommand("M16");
		break;
	}
};

servoElement.oninput = function()
{
	var range = parseInt(servoMax.value,10) - parseInt(servoMin.value,10);
	var angle = parseInt(servoMin.value,10) + ((range/100) * parseInt(servoElement.value,10));

	var command = [
		{name: "S", value: angle.toFixed(0) },
	];
	sendCommand("M4", command);
}

baseSensitivityInput.oninput = function()
{
	var sensitivity = parseInt(baseSensitivityInput.value);

	if(sensitivity < -63)
	{
		sensitivity = -63;
		baseSensitivityInput.value = sensitivity;
	}

	if(sensitivity > 64)
	{
		sensitivity = 64;
		baseSensitivityInput.value = sensitivity;
	}

	var command = [
		{name: "S", value: sensitivity },
	];
	sendCommand("M18", command);
}

elbowSensitivityInput.oninput = function()
{
	var sensitivity = parseInt(elbowSensitivityInput.value);

	if(sensitivity < -63)
	{
		sensitivity = -63;
		elbowSensitivityInput.value = sensitivity;
	}

	if(sensitivity > 64)
	{
		sensitivity = 64;
		elbowSensitivityInput.value = sensitivity;
	}

	var command = [
		{name: "S", value: sensitivity },
	];
	sendCommand("M19", command);
}

shoulderSensitivityInput.oninput = function()
{
	var sensitivity = parseInt(shoulderSensitivityInput.value);

	if(sensitivity < -63)
	{
		sensitivity = -63;
		shoulderSensitivityInput.value = sensitivity;
	}

	if(sensitivity > 64)
	{
		sensitivity = 64;
		shoulderSensitivityInput.value = sensitivity;
	}

	var command = [
		{name: "S", value: sensitivity },
	];
	sendCommand("M20", command);
}

accelerationInput.oninput = function()
{
	console.log(accelerationInput.value);
	var accel = parseFloat(accelerationInput.value).toFixed(1);

	if(accel < 1.0)
	{
		accel = 1.0;
		accelerationInput.value = accel;
	}

	if(accel > 10000.0)
	{
		accel = 10000.0;
		accelerationInput.value = accel;
	}
	
	var command = [
		{name: "A", value: accel },
	];
	sendCommand("M17", command);
}

pumpBtn.onclick = function(){

	if(pumpState ){
		sendCommand( "M6" );	//turn off pump
		pumpBtn.innerHTML = "Vacuum on";
	}
	else
	{
		sendCommand( "M5" );	//turn on pump
		pumpBtn.innerHTML = "Vacuum off";
		
	}
	pumpState = !pumpState;
	console.log("Pump state changed !");
};

playBtn.onclick  = function(){
	var command = [
		{name: "F", value: parseInt(playBackSpeed.value,10) },
	];
	sendCommand("M11",command);
}

pauseBtn.onclick  = function(){
	sendCommand("M12");
}

recordLineBtn.onclick = function(){
	sendCommand("M13");

	// Request recording (update the gui)
	requestRecording();
}

homeBtn.onclick = function(){

	if(recording ){
		sendCommand( "M3" );
		recordBtn.style="color:white";
		//recordBtn.innerHTML = "Record";
		recording = !recording;
	}
	sendCommand("G28");
	console.log("Going home !");
	silenceWebsocket = 1;
};

emergencyBtn.onclick = function(){

	if(recording ){
		sendCommand( "M3" );
		recordBtn.style="color:white";
		//recordBtn.innerHTML = "Record";
		recording = !recording;
	}
	//sendCommand("G28");
	sendCommand("M0");
	console.log("Stop Motion !");
};



window.onload = function() {
	// Initiate the websocket connection
	initWebSocket();

	// Load previous made recording
	requestRecording();

	// Always try to reinitiate the Websocket connection
	setInterval(function() {

		if( websocket.readyState != 1 ){
			initWebSocket();

		}

		if( ! initPosition ){
			// Request position
			sendCommand("M9");
		}

		requestRecording();

	}, 3000)
}



recordBtn.onclick = function(){
	if(! recording ){
		sendCommand( "M2" );
		recordBtn.style="color:red";
		//recordBtn.innerHTML = "Stop recording";
	}else{
		sendCommand( "M3" );
		recordBtn.style="color:white";
		//recordBtn.innerHTML = "Record";
	}
	
	recording = !recording;

	recordLineBtn.classList.toggle('d-none');

};

// Keep the websocket connection alive 
var websocketInterval = function() {
	if(websocket.readyState == 1){
		if( initPosition ){
			if(silenceWebsocket === 0)
			{
				if(websocketCnt === 0)
				{
					if( commandAck ) {
						commandAck = false;
						joystickControl();
					}
					websocketCnt = 1;
				}
				else if(websocketCnt === 1)
				{
					sendCommand("M9");
					websocketCnt = 0;
				}
			}
		}

	}

	// Set next timeout
    setTimeout(websocketInterval, wsInterval);
}

function joystickControl(){
	var feedrateX = 0;
	var feedrateY = 0;
	var feedrateZ = 0;
	
	joystickHandlerCnt++;

	joystickMinValue = 10.0;
	joystickMaxValue = 100.0;
	joystickActiveValueSpan = joystickMaxValue - joystickMinValue;		//only values between 100 and 10 are considered. values below 10 are set to 0, to make it easier to control
	feedrateSpan = parseFloat(maxFeedrate.value - minFeedrate.value);

	scaleFactor = feedrateSpan/joystickActiveValueSpan;
	
	if(xyjoystick.isActive())
	{
		var xy = xyjoystick.getPosition();
		console.log(xy);
		xyJoystickActive = true;
		
		if(Math.abs(xy.y) < joystickMinValue)
		{
			feedrateY = 0.0;
		}
		else
		{
			feedrateY = Math.abs(xy.y) - joystickMinValue;
			feedrateY *= scaleFactor;
			feedrateY += joystickMinValue;
			if(xy.y < 0.0) feedrateY *= -1.0;
			//feedrateY = (xy.y/Math.abs(xy.y))*(parseFloat(minFeedrate.value) + (xy.y - joystickMinValue)*scaleFactor);
		}
		if(Math.abs(xy.x) < joystickMinValue)
		{
			feedrateX = 0.0;
		}
		else
		{
			feedrateX = Math.abs(xy.x) - joystickMinValue;
			feedrateX *= scaleFactor;
			feedrateX += joystickMinValue;
			if(xy.x < 0.0) feedrateX *= -1.0;
		}
	}
	else
	{
		xyJoystickActive = false;
		feedrateX = 0;
		feedratey = 0;
	}

	if(zjoystick.isActive())
	{
		zJoystickActive = true;
		var z = zjoystick.getPosition();

		if(Math.abs(z.y) < joystickMinValue)
		{
			feedrateZ = 0.0;
		}
		else
		{
			feedrateZ = Math.abs(z.y) - joystickMinValue;
			feedrateZ *= scaleFactor;
			feedrateZ += joystickMinValue;
			if(z.y > 0.0) feedrateZ *= -1.0;
		}
	}
	else
	{
		zJoystickActive = false;
		feedrateZ = 0;
	}

	if(zJoystickActive === false && xyJoystickActive === false)
	{
		var command = [
			{name: "X", value: 0.0 },
			{name: "Y", value: 0.0 },
			{name: "Z", value: 0.0 },
		];
		if(JSON.stringify(command) != JSON.stringify(lastG1Command))
		{
			lastG1Command = command;
			sendCommand( "M10", lastG1Command);
			console.log(command);
		}
		else if(joystickHandlerCnt > 10)
		{
			sendCommand( "M10", lastG1Command);
			console.log(command);
			joystickHandlerCnt = 0;
		}
		
		return;
	}
	if(pos.x < 0.0)
	{
		feedrateY *= -1.0;
	}

	var command = [
		{name: "Y", value: feedrateX.toFixed(1) },
		{name: "X", value: feedrateY.toFixed(1) },
		{name: "Z", value: feedrateZ.toFixed(1) },
	];

	if(JSON.stringify(command) != JSON.stringify(lastG1Command))
	{
		lastG1Command = command;
		sendCommand( "M10", lastG1Command);
		console.log(command);
	}
	else if(joystickHandlerCnt > 10)
	{
		sendCommand( "M10", lastG1Command);
		console.log(command);
		joystickHandlerCnt = 0;
	}
}




function initWebSocket()
{

	if( websocket ) websocket.close();
	
	websocket = new WebSocket('ws://192.168.4.1:81/');

	addToLog("Connecting...");
	setStatus("Connecting...", "primary")

	websocket.onopen = function(event) { onWsOpen(event) };
	websocket.onclose = function(event) { onWsClose(event) };
	websocket.onmessage = function(event) { onWsMessage(event) };

	window.addEventListener('beforeunload', function() {
		websocket.close();
	});

	setTimeout(websocketInterval, wsInterval);

}

function sendCommand( command, param = [] ){
	
	var gcode = command + " ";

	if( param.length > 0 ){
		var parameters = param.map(e => e.name + e.value ).join(' ');
		gcode += " " + parameters;
	}

	if(websocket.readyState == 1){
		console.log( "Sending: " + gcode);
		websocket.send( gcode );
		commandsSend++;
	}
}

function onWsOpen(event) {
	addToLog("Websocket connection established");
	setStatus("Connected", "success");

	// When connection first is opened, request current position
	sendCommand("M9");
}

function onWsClose(event) {
	addToLog("Websocket connection lost");
	setStatus("No connection", "danger")
}

// Whenever a message is received from the ESP
function onWsMessage(event) {

	var data = event.data;
	
	if( data.includes("OK")) {
		// Please send new command 
		commandAck = true;

	} else if( data.includes("RDY") ){

		commandAck = true;

	} else if( data.includes("POS") ){

		var positions = [];
		// Read data about position and update current positions
		var items = data.split(" ");
		items.shift(); // Remove "G1"

		for (var i in items) {
	    	positions[i] = items[i].substring(1, items[i].length);
		}

		pos.x = parseFloat(positions[0]);
		pos.y = parseFloat(positions[1]);
		pos.z = parseFloat(positions[2]);

		if(!isFinite(pos.x) || !isFinite(pos.y) || !isFinite(pos.z))
		{
			return;
		}

		if(initPosition != true)
		{
			addToLog( "Position received" );
		}
		

		initPosition = true;
		xDisplay.value = pos.x.toFixed(0);
		yDisplay.value = pos.y.toFixed(0);
		zDisplay.value = pos.z.toFixed(0);
	}

	else if(data.includes("HOMINGDONE"))
	{
		silenceWebsocket = 0;
	}
	else {

		console.log( "Unknown: " +  data );
	}
}

function addToLog( data ){

	var options = { };
	var now = new Date();

	if(logElement.value != ""){
		logElement.value += "\n";
	}

	logElement.value += now.toLocaleTimeString('en-GB', options) + ": " + data;
	logElement.scrollTop = logElement.scrollHeight;

}

function setStatus( text, type ){
	statusBar.className = type;

	textArea = statusBar.getElementsByTagName('span')[0];
	textArea.innerHTML = text;
}

// Map function from Arduino documentation
function map(x, in_min, in_max, out_min, out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Joystick

var xyjoystick = joystick( document.getElementById('joystick-wrapper-xy'), 'xy' );
var zjoystick = joystick( document.getElementById('joystick-wrapper-z'), 'z' );

// setInterval(() => console.log(zjoystick.getPosition()), 50);

function joystick(parent, axis) {

	// Max lenght from center
	const maxDiff = 100; 

	const stick = document.createElement('div');
	stick.classList.add('joystick');

	stick.addEventListener('mousedown', handleMouseDown, { passive: true });
	document.addEventListener('mousemove', handleMouseMove);
	document.addEventListener('mouseup', handleMouseUp);
	stick.addEventListener('touchstart', handleMouseDown, { passive: true });
	document.addEventListener('touchmove', handleMouseMove);
	document.addEventListener('touchend', handleMouseUp);

	let dragStart = null;
	let active = false;
	let currentPos = { x: 0, y: 0 };

	function handleMouseDown(event) {
		stick.style.transition = '0s';
		if (event.changedTouches) {
			dragStart = {
				x: event.changedTouches[0].clientX,
				y: event.changedTouches[0].clientY,
			};
			return;
		}

		dragStart = {
			x: event.clientX,
			y: event.clientY,
		};

		active = true;
	}

	function handleMouseMove(event) {
		if (dragStart === null) return;
		event.preventDefault(); // Prevent scroll on mobile touch of joystick

		if (event.changedTouches) {
		 	event.clientX = event.changedTouches[0].clientX;
			event.clientY = event.changedTouches[0].clientY;
		}

		const xDiff = Math.round(event.clientX - dragStart.x);
		const yDiff = Math.round(event.clientY - dragStart.y);
		const angle = Math.atan2(yDiff, xDiff);

		const distance = Math.min(maxDiff, Math.hypot(xDiff, yDiff));

		var xNew = distance * Math.cos(angle);
		var yNew = distance * Math.sin(angle);

		if(axis == "z"){
			xNew = 0;
		}

		stick.style.transform = `translate3d(${xNew}px, ${yNew}px, 0px)`;
		currentPos = { x: xNew, y: yNew };

		active = true;
		
	}

	function handleMouseUp(event)
	{
		if (dragStart === null) return;

		stick.style.transition = '.2s';
		stick.style.transform = `translate3d(0px, 0px, 0px)`;
		dragStart = null;
		active = false;

		currentPos = { x: 0, y: 0 };

	}

	parent.appendChild(stick);

	return {
		getPosition: () => currentPos,
		isActive: () => active,
	};
}
