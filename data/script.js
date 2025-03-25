const canvas = document.getElementById("gameCanvas");
const ctx = canvas.getContext("2d");

canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

// คำนวณตำแหน่งกึ่งกลางของหน้าต่าง
const centerX = canvas.width / 2;
const centerY = canvas.height / 2;

const gridSize = 50; // 1 ช่อง = 50px = 0.5m
const snakeSize = (22 / 50) * gridSize; // ขนาดงู = 22cm แปลงเป็น pixel
const wheelDistance = 0.3 * gridSize; // ระยะห่างระหว่างล้อซ้าย-ขวา (30cm)
const rotationAngle = 30 * (Math.PI / 180); // 30 degrees in radians
const initialSnake = [{ x: centerX, y: centerY, angle: 0 }];
let snake = [{ x: centerX, y: centerY, angle: 0 }]; // Add angle property
let speed = 5;
let target = null;
let trajectory = [];
let fadeEffect = null;
let direction = { x: 0, y: 0 };
let moving = false;
let currentParams = {}; // Store current parameters
let currentV = 0;
let currentOmega = 0;
let currentVL = 0;
let currentVR = 0;

const velocityDisplay = document.createElement("div");
velocityDisplay.style.position = "absolute";
velocityDisplay.style.top = "100px";
velocityDisplay.style.left = "100px";
velocityDisplay.style.background = "rgba(232, 161, 9, 0.9)";
velocityDisplay.style.color = "white";
velocityDisplay.style.padding = "10px";
velocityDisplay.style.borderRadius = "5px";
document.body.appendChild(velocityDisplay);

// ✅ ใช้ Bootstrap Modal API แทนการควบคุม `display`
const paramModal = new bootstrap.Modal(document.getElementById("paramModal"));

document.getElementById("openModal").addEventListener("click", () => {
    paramModal.show();
});

// ✅ Modal Form Validation
function validateModalForm() {
    let selectedSet = document.querySelector('input[name="paramType"]:checked');
    if (!selectedSet) return false;

    let inputs;
    if (selectedSet.value === "Velocity") { // เปลี่ยน "A" เป็น "Velocity"
        inputs = document.querySelectorAll("#Velocity input"); // เปลี่ยน "#setA" เป็น "#Velocity"
    } else {
        inputs = document.querySelectorAll("#Position input"); // เปลี่ยน "#setB" เป็น "#Position"
    }

    for (let input of inputs) {
        if (input.value === "" || input.value === null) {
            return false;
        }
    }
    return true;
}

document.getElementById("applyParams").addEventListener("click", () => {
    if (!validateModalForm()) {
        alert("Please fill in all parameter fields.");
        return;
    }

    let selectedSet = document.querySelector('input[name="paramType"]:checked');

    let params = {};
    if (selectedSet.value === "Velocity") { // เปลี่ยน "A" เป็น "Velocity"
        params = {
            v: parseFloat(document.getElementById("v").value), // เปลี่ยน "xA" เป็น "v"
            w: parseFloat(document.getElementById("w").value), // เปลี่ยน "yA" เป็น "w"
            time: parseFloat(document.getElementById("time").value) // เปลี่ยน "thetaA" เป็น "time" และเพิ่ม "timeA" เป็น "time"
        };
    } else {
        params = {
            distance: parseFloat(document.getElementById("distance").value), // เปลี่ยน "xB" เป็น "distance"
            theta: parseFloat(document.getElementById("theta").value) // เปลี่ยน "yB" เป็น "theta" และเปลี่ยน "thetaB" เป็น "theta"
        };
    }

    console.log("Applied Parameters:", params);
    currentParams = params; // Update current parameters
    sendParamsToESP32(params); // Send parameters to ESP32
    paramModal.hide();
});

// ✅ ปิดการใช้งานช่อง input ทั้งหมดเริ่มต้น
function disableAllInputs() {
    let inputsA = document.querySelectorAll("#Velocity input");
    let inputsB = document.querySelectorAll("#Position input");

    inputsA.forEach(input => input.disabled = true);
    inputsB.forEach(input => input.disabled = true);
}

// ✅ บังคับให้เลือก Set A หรือ B ก่อน
document.addEventListener("DOMContentLoaded", () => {
    disableAllInputs();
    document.getElementById("applyParams").disabled = true; // ปิดปุ่ม Apply
});

// ✅ เปิดช่องใส่ค่าตามเซตที่เลือก
function toggleInputs(type) {
    let inputsVelocity = document.querySelectorAll("#Velocity input"); // เปลี่ยน "#setA" เป็น "#Velocity"
    let inputsPosition = document.querySelectorAll("#Position input"); // เปลี่ยน "#setB" เป็น "#Position"

    if (type === "Velocity") { // เปลี่ยน "A" เป็น "Velocity"
        inputsVelocity.forEach(input => input.disabled = false); // เปลี่ยน "inputsA" เป็น "inputsVelocity"
        inputsPosition.forEach(input => input.disabled = true); // เปลี่ยน "inputsB" เป็น "inputsPosition"
    } else if (type === "Position") { // เปลี่ยน "B" เป็น "Position"
        inputsVelocity.forEach(input => input.disabled = true); // เปลี่ยน "inputsA" เป็น "inputsVelocity"
        inputsPosition.forEach(input => input.disabled = false); // เปลี่ยน "inputsB" เป็น "inputsPosition"
    }

    // ✅ Enable/disable Apply button based on form validity
    document.getElementById("applyParams").disabled = !validateModalForm();
}

// ✅ ตรวจจับการเลือกเซต
document.querySelectorAll('input[name="paramType"]').forEach(radio => {
    radio.addEventListener("change", (event) => {
        toggleInputs(event.target.value);
    });
});

// ✅ ตรวจจับการเปลี่ยนแปลงในช่อง input
document.querySelectorAll("#Velocity input, #Position input").forEach(input => { // เปลี่ยน "#setA input, #setB input" เป็น "#Velocity input, #Position input"
    input.addEventListener("input", () => {
        document.getElementById("applyParams").disabled = !validateModalForm();
    });
});


function drawAxes() {
    ctx.strokeStyle = "gray";
    ctx.lineWidth = 1;

    ctx.beginPath();
    ctx.moveTo(0, centerY);
    ctx.lineTo(canvas.width, centerY);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, canvas.height);
    ctx.stroke();
}

function drawTrajectory() {
    ctx.fillStyle = "red";
    for (let point of trajectory) {
        ctx.fillRect(point.x, point.y, 3, 3);
    }
}

function drawFadeEffect() {
    if (fadeEffect) {
        let { x, y, alpha } = fadeEffect;
        ctx.fillStyle = `rgba(255, 255, 0, ${alpha})`;
        ctx.beginPath();
        ctx.arc(x, y, 10, 0, Math.PI * 2);
        ctx.fill();
        fadeEffect.alpha -= 0.02;
        if (fadeEffect.alpha <= 0) fadeEffect = null;
    }
}

function drawDirectionArrow(x, y, angle) {
    ctx.save();
    ctx.translate(x, y); // Move origin to snake's center
    ctx.rotate(angle); // Rotate by the snake's angle

    // Draw the arrow
    ctx.fillStyle = "black";
    ctx.beginPath();
    ctx.moveTo(snakeSize / 2, 0);
    ctx.lineTo(-snakeSize / 3, -snakeSize / 4);
    ctx.lineTo(-snakeSize / 3, snakeSize / 4);
    ctx.closePath();
    ctx.fill();

    ctx.restore(); // Restore original state
}


function drawSnake() {
    ctx.fillStyle = "lime";
    let head = snake[0];

    ctx.beginPath();
    ctx.arc(head.x, head.y, snakeSize / 2, 0, Math.PI * 2);
    ctx.fill();

    drawDirectionArrow(head.x, head.y, head.angle); // Draw arrow with angle
}

function updateVelocityDisplay(v, omega, vL, vR, x, y, angle, speed) {
    velocityDisplay.innerHTML = `
        <b>Robot Data</b><br>
        Linear Velocity (v): ${v.toFixed(2)} px/frame<br>
        Angular Velocity (ω): ${omega.toFixed(2)} rad/frame<br>
        Left Wheel Speed (vL): ${vL.toFixed(2)} px/frame<br>
        Right Wheel Speed (vR): ${vR.toFixed(2)} px/frame<br>
        <br>
        <b>Real-Time Data</b><br>
        Speed: ${speed.toFixed(2)} px/s<br>
        X: ${x.toFixed(2)}<br>
        Y: ${y.toFixed(2)}<br>
        Angle: ${angle.toFixed(2)}
    `;
}

function updateSnakeWithControls() {
    if (moving) {
        let head = snake[0];
        let newHead = { x: head.x, y: head.y, angle: head.angle }; // Create new head object

        if (direction.x === 0) {
            // Move forward/backward
            newHead.x += speed * Math.cos(head.angle) * -direction.y; // Update x based on angle and direction
            newHead.y += speed * Math.sin(head.angle) * -direction.y; // Update y based on angle and direction
        } else {
            // Rotate left/right
            newHead.angle += direction.x * rotationAngle;
        }

        snake.unshift(newHead);
        snake.pop();
        trajectory.push(newHead);
        if (trajectory.length > 100) trajectory.shift(); // Limit trajectory length

        if (direction.x === 0) {
            currentVL = speed * -direction.y;
            currentVR = speed * -direction.y;
            currentV = (currentVL + currentVR) / 2;
            currentOmega = (currentVR - currentVL) / wheelDistance;
        } else {
            currentVL = 0;
            currentVR = 0;
            currentV = 0;
            currentOmega = direction.x * rotationAngle;
        }
        updateVelocityDisplay(currentV, currentOmega, currentVL, currentVR, 0, 0, 0, 0);
        sendDirectionToESP32(direction, currentV, currentOmega, currentVL, currentVR, head.angle); // Send direction and velocity to ESP32
    }
}

// ✅ ฟังก์ชันเคลื่อนที่งูแบบ Smooth (กดปุ่มแล้วค้างไว้)
function moveSnake(dx, dy) {
    direction = { x: dx, y: dy };
    moving = true;
}

// ✅ หยุดเคลื่อนที่เมื่อปล่อยปุ่ม
function stopSnake() {
    moving = false;
    sendDirectionToESP32({x:0,y:0},0,0,0,0,snake[0].angle)
}

// ✅ ปุ่มควบคุมการเคลื่อนที่
const moveAmount = 1;

document.getElementById("up").addEventListener("mousedown", () => moveSnake(0, -moveAmount));
document.getElementById("down").addEventListener("mousedown", () => moveSnake(0, moveAmount));
document.getElementById("left").addEventListener("mousedown", () => moveSnake(-moveAmount, 0));
document.getElementById("right").addEventListener("mousedown", () => moveSnake(moveAmount, 0));
document.addEventListener("mouseup", stopSnake);

// ✅ คีย์บอร์ดควบคุมแบบ Smooth
document.addEventListener("keydown", (event) => {
    if (event.key === "ArrowUp") moveSnake(0, -1);
    if (event.key === "ArrowDown") moveSnake(0, 1);
    if (event.key === "ArrowLeft") moveSnake(-1, 0);
    if (event.key === "ArrowRight") moveSnake(1, 0);
});

document.addEventListener("keyup", stopSnake);

// ✅ ฟังก์ชันหลักของเกม
function gameLoop() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawAxes();
    drawTrajectory();
    drawFadeEffect();
    drawSnake();
    updateSnakeWithControls();  // ใช้ปุ่มควบคุม
    requestAnimationFrame(gameLoop);
}

function resetSnake() {
    snake = [...initialSnake]; // คืนค่าตำแหน่งเริ่มต้น
    trajectory = []; // ล้างเส้นทางการเคลื่อนที่
    direction = { x: 0, y: 0 }; // หยุดการเคลื่อนที่
    moving = false; // หยุดการเคลื่อนที่
    sendDirectionToESP32({x:0,y:0},0,0,0,0,snake[0].angle) // ส่งคำสั่งหยุดไปที่ ESP32
}

document.getElementById("reset").addEventListener("click", resetSnake);

gameLoop();

// --- WebSocket Communication ---
let websocket;

function initWebSocket() {
    websocket = new WebSocket('ws://' + window.location.hostname + ':81/');
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
    websocket.onerror = onError;
}

function onOpen(event) {
    console.log('WebSocket connection opened');
}

function onClose(event) {
    console.log('WebSocket connection closed');
    setTimeout(initWebSocket, 2000);
}

function onMessage(event) {
    console.log('WebSocket message received:', event.data);
    try {
        const data = JSON.parse(event.data);
        if (data.type === "realtime") {
            // อัปเดตค่าใน velocityDisplay
            // realTimeDisplay.innerHTML = `
            //     <b>Real-Time Data</b><br>
            //     Speed: ${data.speed.toFixed(2)} px/s<br>
            //     X: ${data.x.toFixed(2)}<br>
            //     Y: ${data.y.toFixed(2)}<br>
            //     Angle: ${data.angle.toFixed(2)}
            // `;
            updateVelocityDisplay(currentV, currentOmega, currentVL, currentVR, data.x, data.y, data.angle, data.speed);
        }
    } catch (error) {
        console.error("Error parsing JSON:", error);
    }
}

function onError(event) {
    console.error('WebSocket error:', event);
}

function sendDirectionToESP32(direction, v, omega, vL, vR, angle) {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
        let dirString;
        if (direction.x === 0 && direction.y === -1) {
            dirString = "Forward";
        } else if (direction.x === 0 && direction.y === 1) {
            dirString = "Backward";
        } else if (direction.x === -1 && direction.y === 0) {
            dirString = "Turn Left";
        } else if (direction.x === 1 && direction.y === 0) {
            dirString = "Turn Right";
        } else if (direction.x === 0 && direction.y === 0) {
            dirString = "Stop";
        } else if (direction.x === 1 && direction.y === -1) {
            dirString = "Forward Right";
        } else if (direction.x === -1 && direction.y === -1) {
            dirString = "Forward Left";
        } else if (direction.x === 1 && direction.y === 1) {
            dirString = "Backward Right";
        } else if (direction.x === -1 && direction.y === 1) {
            dirString = "Backward Left";
        }
        websocket.send(JSON.stringify({ type: "direction", data: dirString, v: v, omega: omega, vL: vL, vR: vR, angle: angle }));
    }
}

function sendParamsToESP32(params) {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(JSON.stringify({ type: "params", data: params }));
    }
}

initWebSocket();
