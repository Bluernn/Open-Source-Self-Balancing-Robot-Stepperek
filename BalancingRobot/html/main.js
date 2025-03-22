var websocket = new WebSocket('ws://' + location.hostname + '/');
var robotStarted = false;
var currentMode = "balanceMode";
var dataBuffer = [];

let isDragging = false;
let centerX, centerY;
let maxVelocity = 0.4;
let maxTurnFactor = 0.25;
let isDevMode = false;

// Mode switching and view update
function switchMode(mode) {
    currentMode = mode;

    // Hides all sections
    document.getElementById("positionModeFields").style.display = "none";
    document.getElementById("testModeFields").style.display = "none";
    document.getElementById("sensorModeFields").style.display = "none";
    document.getElementById("remoteModeBox").style.display = "none";
    document.getElementById("ctrlFields").style.display = "none";

    // Displays the appropriate section for the selected mode
    if (mode === 'positionMode') {
        document.getElementById("positionModeFields").style.display = "block";
    } else if (mode === 'sensorMode') {
        document.getElementById("sensorModeFields").style.display = "block";
    } else if (mode === 'remoteMode') {
        document.getElementById("remoteModeBox").style.display = "block";
    } else if (mode === 'ctrlMode') {
        document.getElementById("ctrlFields").style.display = "block";
    } else if (mode === 'testMode') {
        document.getElementById("testModeFields").style.display = "block";
    }

    // Updated active mode display
    let modeText = '';
    if (mode === 'balanceMode') {
        modeText = 'Balance Mode';
    } else if (mode === 'positionMode') {
        modeText = 'Position Mode';
    } else if (mode === 'sensorMode') {
        modeText = 'Ultrasonic Mode';
    } else if (mode === 'remoteMode') {
        modeText = 'Remote Control';
    } else if (mode === 'quadrantSequenceMode') {
        modeText = 'Quadrant Driving Mode';
    } else if (mode === 'ctrlMode'){
        modeText = 'Changing Controller Gains Mode';
    } else if (mode === 'testMode') {
        modeText = 'Pos./Vel. Set Mode';
    }

    document.getElementById("currentModeDisplay").textContent = `Active mode: ${modeText}`;
}

// Robot START/STOP function
function toggleRobotStartStop() {
    robotStarted = !robotStarted;
    document.getElementById("startStopRobotBtn").textContent = robotStarted ? "STOP" : "START";

    if (robotStarted) {
        sendRobotData();
    } else {
        stopRobot();
    }

    document.querySelectorAll(".modeBtn").forEach(btn => btn.disabled = robotStarted);
}

// Sends data depending on the selected mode
function sendRobotData() {
    let data = { id: currentMode };

    // Gather values based on the selected mode
    if (currentMode === "positionMode") {
        let position = document.getElementById("positionInput");

        data.position = position ? parseFloat(position.value) : 0;
    } else if (currentMode === "sensorMode") {
        let sensorPosition = document.getElementById("sensorPositionInput");

        data.sensorPosition = sensorPosition ? parseFloat(sensorPosition.value) : 0;
    } else if (currentMode === "ctrlMode") {
        let KpInput = document.getElementById("KpInput");
        let KdInput = document.getElementById("KdInput");

        data.KpAngle = KpInput ? parseFloat(KpInput.value) : 0;
        data.KdAngle = KdInput ? parseFloat(KdInput.value) : 0;
    } else if (currentMode === "testMode") {
        let tmp = document.getElementById("testInput");

        data.tmp = tmp ? parseFloat(tmp.value) : 0;
    }
    
    // Add robot enabled status
    data.robotEnable = robotStarted ? 1 : 0;

    // Convert data to JSON and display it
    let jsonData = JSON.stringify(data, null, 2); // Pretty print JSON with indentation
    document.getElementById("jsonContent").textContent = jsonData;

    // Send JSON data through WebSocket
    websocket.send(jsonData);
}

// Stop the robot
function stopRobot() {
    let jsonData = JSON.stringify({ id: "stop", robotEnable: 0 });
    websocket.send(jsonData);
    document.getElementById("jsonContent").textContent = jsonData;
}

// Updates the displayed tilt angle
function updateTiltAngleDisplay(angle) {
    document.getElementById("tiltAngleValue").textContent = angle + "°";
}

// WebSocket initialization
websocket.onopen = function(evt) {
    console.log('WebSocket connection opened');
    websocket.send(JSON.stringify({ id: "init" }));
};

websocket.onmessage = function(evt) {
	var msg = evt.data;
	//console.log("msg=" + msg);
	var values = msg.split('\4'); // \4 is EOT
	//console.log("values=" + values);
	if (values[0] == 'DATA') {
		var p_ref = parseFloat(values[1]);
		var p = parseFloat(values[2]);
		var v_ref = parseFloat(values[3]);
		var v = parseFloat(values[4]);
		var O_ref = parseFloat(values[5]);
		var O = parseFloat(values[6]);
		var a = parseFloat(values[7]);

        updateTiltAngleDisplay(O.toFixed(2));

        if (isDevMode == true)
        {
            var logEntry = `p_ref=${p_ref.toFixed(6)} p=${p.toFixed(6)} v_ref=${v_ref.toFixed(6)} v=${v.toFixed(6)} O_ref=${O_ref.toFixed(6)} O=${O.toFixed(6)} a=${a.toFixed(6)}\n`;
            dataBuffer.push(logEntry);
        }
	}
}

// Save data to file
function saveDataToFile() {
	// Blob object with the contents of the buffer
	let blob = new Blob(dataBuffer, { type: "text/plain" });
	let url = URL.createObjectURL(blob);

	// Create link to download the file
	let a = document.createElement("a");
	a.href = url;
	a.download = "esp_data_log.txt";  // Nazwa pliku
	document.body.appendChild(a);
	a.click();

	// After downloading, remove the link
	document.body.removeChild(a);
	URL.revokeObjectURL(url);
}

// Clean the buffer
function clearBuffer() {
	dataBuffer = [];
	console.log("Buffer cleared");
}

document.getElementById("remoteModeBtn").onclick = () => {
    switchMode('remoteMode');
    const rect = document.getElementById("joystickContainer").getBoundingClientRect();
    centerX = rect.left + rect.width / 2;
    centerY = rect.top + rect.height / 2;
};

function startDragging(event) {
    isDragging = true;
    event.preventDefault();
}

function stopDragging(event) {
    isDragging = false;
    document.getElementById("joystick").style.transform = 'translate(-50%, -50%)';
    sendJoystickData(0, 0);  // Resets values after stopping
    event.preventDefault();
}

let lastUpdate = Date.now();  // Store the last update time

function dragJoystick(event) {
    if (!isDragging || currentMode !== 'remoteMode') return;

    // Handle touch or mouse
    const touch = event.touches ? event.touches[0] : event;
    const deltaX = touch.clientX - centerX;
    const deltaY = touch.clientY - centerY;
    const maxRadius = 160;  // Increased maximum radius for larger joystick
    const distance = Math.min(maxRadius, Math.hypot(deltaX, deltaY));
    const angle = Math.atan2(deltaY, deltaX);

    const x = distance * Math.cos(angle);
    const y = distance * Math.sin(angle);

    document.getElementById("joystick").style.transform = `translate(${x - 75}%, ${y - 75}%)`; // Adjusted for new size

    // Calculate velocity and turnFactor
    const velocityBase = (distance / maxRadius * maxVelocity);
    let velocity = y > 0 ? velocityBase : -velocityBase;
    const turnFactor = (x / maxRadius * maxTurnFactor);

    // Only send data if enough time has passed (e.g., 50ms)
    if (Date.now() - lastUpdate > 50) {
        sendJoystickData(velocity.toFixed(2), turnFactor.toFixed(2));
        lastUpdate = Date.now();  // Update last update time
    }

    event.preventDefault();
}

function sendJoystickData(velocity, turnFactor) {
    if (currentMode !== 'remoteMode') return;

    const data = {
        id: "remoteMode",
        velocity: parseFloat(velocity),
        turnFactor: parseFloat(turnFactor),
        robotEnable: robotStarted ? 1 : 0
    };

    const jsonData = JSON.stringify(data, null, 2);
    document.getElementById("jsonContent").textContent = jsonData;
    websocket.send(jsonData);
}

function devMode() {
    isDevMode = !isDevMode;
    if (isDevMode === true)
    {
        document.getElementById("saveBtn").style.display = "block";
        document.getElementById("clearBufferBtn").style.display = "block";
        document.getElementById("jsonDisplay").style.display = "block";
        document.getElementById("ctrlModeBtn").style.display = "block";
        document.getElementById("testModeBtn").style.display = "block";
    }
    else
    {
        document.getElementById("saveBtn").style.display = "none";
        document.getElementById("clearBufferBtn").style.display = "none";
        document.getElementById("jsonDisplay").style.display = "none"; 
        document.getElementById("ctrlModeBtn").style.display = "none";
        document.getElementById("testModeBtn").style.display = "none";
    }
}

// Add mouse and touch events to joystick
document.getElementById("joystick").addEventListener('mousedown', startDragging);
document.getElementById("joystick").addEventListener('touchstart', startDragging, { passive: true });
document.addEventListener('mouseup', stopDragging);
document.addEventListener('touchend', stopDragging, { passive: true });
document.addEventListener('mousemove', dragJoystick);
document.addEventListener('touchmove', dragJoystick, { passive: true });

// Operation of mode buttons
document.getElementById("balanceModeBtn").onclick = () => switchMode('balanceMode');
document.getElementById("positionModeBtn").onclick = () => switchMode('positionMode');
document.getElementById("sensorModeBtn").onclick = () => switchMode('sensorMode');
document.getElementById("quadrantSequenceMode").onclick = () => switchMode('quadrantSequenceMode');
document.getElementById("ctrlModeBtn").onclick = () => switchMode('ctrlMode');
document.getElementById("testModeBtn").onclick = () => switchMode('testMode');

// Button operation
document.getElementById("saveBtn").addEventListener("click", saveDataToFile);
document.getElementById("clearBufferBtn").addEventListener("click", clearBuffer);
document.getElementById("startStopRobotBtn").addEventListener("click", toggleRobotStartStop);
document.getElementById("sendBtn").addEventListener("click", sendRobotData);
document.getElementById("devModeBtn").addEventListener("click", devMode);
