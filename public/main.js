const fcanvas = document.getElementById("flight");
const mcanvas = document.getElementById("map");
const pcanvas = document.getElementById("poi");
const fctx = fcanvas.getContext("2d");
const mctx = mcanvas.getContext("2d");
const pctx = pcanvas.getContext("2d");

const spread_chance = 0.3;
const spread_radius = 20;

// set canvas dimensions
fcanvas.width = fcanvas.offsetWidth; fcanvas.height = fcanvas.offsetHeight;
mcanvas.width = mcanvas.offsetWidth; mcanvas.height = mcanvas.offsetHeight;
pcanvas.width = pcanvas.offsetWidth; pcanvas.height = pcanvas.offsetHeight;

// brush state
let currentTool = null;
let isDrawing = false;
const fireBrush = document.getElementById('fire-brush');
const poiBrush = document.getElementById('poi-brush');
const firePoints = [];
const poiPoints = [];
const discoveredPOIs = [];
let fireGrid = [];
let POIY = 28;

// brush settings
const FIRE_BRUSH_SIZE = 15;
const FIRE_COLOR = 'rgba(255, 83, 73, 0.3)';
const POI_SIZE = 5;
const POI_COLOR = 'rgba(0, 255, 255, 0.8)';

// Brush selection
fireBrush.addEventListener('click', () => {
    currentTool = 'fire';
    fireBrush.classList.add('active');
    poiBrush.classList.remove('active');
});

poiBrush.addEventListener('click', () => {
    currentTool = 'poi';
    poiBrush.classList.add('active');
    fireBrush.classList.remove('active');
});

// Mouse interaction handlers
fcanvas.addEventListener('mousedown', startDrawing);
fcanvas.addEventListener('mousemove', draw);
fcanvas.addEventListener('mouseup', () => {isDrawing = false;});
fcanvas.addEventListener('mouseleave', () => {isDrawing = false;});
fcanvas.addEventListener('click', handleClick);

function mousePos(canvas, evt) {
  const rect = canvas.getBoundingClientRect();
  return { x: evt.clientX - rect.left, y: evt.clientY - rect.top };
}

function startDrawing(e) {
    if (currentTool === 'fire') { isDrawing = true; draw(e); }
}

function draw(e) {
  if (!isDrawing || currentTool !== 'fire') return;
  
  const pos = mousePos(fcanvas, e);
  fireGrid.push(`${Math.round(pos.x)},${Math.round(pos.y)}`);
  firePoints.push(pos);
  
  fctx.fillStyle = `rgba(255, 165, 0, 0.3)`;
  fctx.beginPath();
  fctx.arc(pos.x, pos.y, FIRE_BRUSH_SIZE, 0, Math.PI * 2);
  fctx.fill();
}

function handleClick(e) {
  if (currentTool === 'poi') {
    const pos = mousePos(fcanvas, e);
    poiPoints.push(pos);
        
    fctx.fillStyle = POI_COLOR;
    fctx.beginPath();
    fctx.arc(pos.x, pos.y, POI_SIZE, 0, Math.PI * 2);
    fctx.fill();
  }
}

function spreadFire() {
    const newFirePoints = [];
    for (const coordStr of fireGrid) {
        newFirePoints.push(coordStr);
        const coords = coordStr.split(',');

        if (Math.random() > spread_chance) continue;
        
        const angle = Math.random() * Math.PI * 2;
        const newX = Math.round(parseFloat(coords[0]) + Math.cos(angle) * spread_radius);
        const newY = Math.round(parseFloat(coords[1]) + Math.sin(angle) * spread_radius);
            
        // Check if new position is within canvas bounds
        if (newX < 0 || newX > fcanvas.width || newY < 0 || newY > fcanvas.height) {
            continue;
        }

        if (!newFirePoints.includes(`${newX},${newY}`)) {
            newFirePoints.push(`${newX},${newY}`);
        }
    }
    
    fireGrid = newFirePoints;
}

function redrawFireAndPOI() {
    for (const coordStr of fireGrid) { // redraw fire
        const coords = coordStr.split(',');
        fctx.fillStyle = `rgba(255, 165, 0, ${0.3}`;
        fctx.beginPath();
        fctx.arc(parseFloat(coords[0]), parseFloat(coords[1]), FIRE_BRUSH_SIZE, 0, Math.PI * 2);
        fctx.fill();
    }
    
    fctx.fillStyle = POI_COLOR;
    poiPoints.forEach(point => { // redraw POI
        fctx.beginPath();
        fctx.arc(point.x, point.y, POI_SIZE, 0, Math.PI * 2);
        fctx.fill();
    });
}

function checkFireInFOV(position) {
    for (const coordStr of fireGrid) {
        const coords = coordStr.split(',');
        const dx = parseFloat(coords[0]) - position.x;
        const dy = parseFloat(coords[1]) - position.y;
        const distance = Math.sqrt(dx*dx + dy*dy);
        
        if (distance <= 50) { // fire in FOV
            const scaledPoint = scalePoint({x: parseFloat(coords[0]), y: parseFloat(coords[1])}, 'p');

            pctx.fillStyle = 'rgba(255, 0, 0, 1.0)';
            pctx.beginPath();
            pctx.arc(scaledPoint.x, scaledPoint.y, 2, 0, Math.PI * 2);
            pctx.fill();
        }
    }
}

function checkPOIInFOV(position) {
    for (const point of poiPoints) {
        const dx = point.x - position.x;
        const dy = point.y - position.y;
        const distance = Math.sqrt(dx*dx + dy*dy);
        
        if (distance <= 50) { // POI in FOV
            const scaledPoint = scalePoint(point, 'p');
            if (!discoveredPOIs.includes(point)) {
                pctx.fillStyle = "black";
                pctx.beginPath();
                pctx.arc(scaledPoint.x, scaledPoint.y, 5, 0, Math.PI * 2);
                pctx.fill();
                pctx.font = '17px Trebuchet MS';
                pctx.fillStyle = 'black';
                const lineHeight = 24;
                const padding = 28;
                discoveredPOIs.push(point);
                pctx.fillText(`POI #${discoveredPOIs.indexOf(point)+1} at (${point.x.toFixed(1)}, ${point.y.toFixed(1)})`, padding + 10, POIY);
                POIY += lineHeight;
            }
        }
    }
}

const segments = [];
const path = [];
const cols = 8;
const spacing = fcanvas.width/cols;

let totalLength = 0;
let direction = -1;

for (let i = 0; i < cols; i++) {
    const startX = spacing * (i + 0.5);
    const startY = direction === 1 ? 50 : fcanvas.height - 50;
    const endY = direction === 1 ? fcanvas.height - 50 : 50;
    path.push({ x: startX, y: startY });
    path.push({ x: startX, y: endY });
    direction *= -1;
}

for (let i = 0; i < path.length - 1; i++) {
    const d = [path[i + 1].x - path[i].x, path[i + 1].y - path[i].y];
    const length = Math.sqrt(d[0]*d[0] + d[1]*d[1]);
    segments.push({
        start: totalLength,
        length: length,
        startPoint: path[i],
        endPoint: path[i + 1]
    });
    totalLength += length;
}

const aspect = fcanvas.width / fcanvas.height;
const canvasMap = {
    m: {
        height: mcanvas.height,
        width: mcanvas.height * aspect,
        x: (mcanvas.width - (mcanvas.height * aspect))
    }, p: {
        height: pcanvas.height,
        width: pcanvas.height * aspect,
        x: (pcanvas.width - (pcanvas.height * aspect))
    }
};

function scalePoint(point, canvas='m') {
    const map = canvasMap[canvas];
    const scaleX = map.width / fcanvas.width;
    const scaleY = map.height / fcanvas.height;
    return {
        x: map.x + point.x * scaleX,
        y: point.y * scaleY
    };
}

function drawPath(ctx, isMap=false) {
    ctx.beginPath();
    const startPoint = isMap ? scalePoint(path[0]) : path[0];
    ctx.moveTo(startPoint.x, startPoint.y);
    
    for (let i = 1; i < path.length; i++) {
        const point = isMap ? scalePoint(path[i]) : path[i];
        ctx.lineTo(point.x, point.y);
    }
    
    ctx.strokeStyle = isMap ? "black" : "rgba(0,0,0,0)";
    ctx.lineWidth = 2;
    ctx.stroke();
}

function drawPlane(position, ctx, isMap = false) {
    const pos = isMap ? scalePoint(position) : position;
    
    if (isMap) {
        ctx.fillStyle = "black";
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 5, 0, Math.PI * 2);
        ctx.fill();
    } else {
        ctx.fillStyle = "blue";
        ctx.fillRect(pos.x - 15, pos.y - 15, 30, 30);
    }
}

function drawFOV(position, ctx, isMap = false) {
    const pos = isMap ? scalePoint(position) : position;
    ctx.fillStyle = isMap ? "rgba(0, 255, 255, 0.3)" : "rgba(0,0,0,0)";

    ctx.beginPath();
    ctx.arc(pos.x, pos.y, (isMap ? 5 : 10)*5, 0, Math.PI * 2);
    ctx.fill();
}

function drawMetrics(position) {
    mctx.font = '17px Trebuchet MS';
    mctx.fillStyle = 'black';
    
    const padding = 28;
    const lineHeight = (mcanvas.height - (padding*2 + 17*4))/3;

    let y = 28;
    const addLine = (t) => {
        mctx.fillText(t, padding, y);
        y += lineHeight;
    };
  
    const completion = ((distance / totalLength) * 100).toFixed(1);
    const remaining = Math.round((totalLength - distance))-2;
    
    addLine(`Mission Progress: ${completion}%`);
    addLine(`Position: (${position.x.toFixed(2)}m, ${position.y.toFixed(2)}m)`);
    addLine(`Distance to completion: ${remaining}m`);
    addLine(`Total path length: ${totalLength.toFixed(1)}m`);
}

function getPosition(d) {
    let current = segments.find(s => (d >= s.start) && (d <= s.start + s.length));
    if (!current) return path[path.length - 1];

    let segmentProgress = (d - current.start) / current.length;
    return {
        x: current.startPoint.x + (current.endPoint.x - current.startPoint.x) * segmentProgress,
        y: current.startPoint.y + (current.endPoint.y - current.startPoint.y) * segmentProgress
    };
}

let distance = 0;
const speed = 2;

function update() {
    fctx.clearRect(0, 0, fcanvas.width, fcanvas.height);
    mctx.clearRect(0, 0, mcanvas.width, mcanvas.height);

    fctx.clearRect(0, 0, fcanvas.width, fcanvas.height);
    var bg = new Image(); bg.src = "pixelcamo.jpg";
    bg.onload = () => { fctx.drawImage(bg, 0, 0, (fcanvas.height*(bg.width/bg.height)), fcanvas.height); }
    
    const position = getPosition(distance);
    
    checkFireInFOV(position);
    checkPOIInFOV(position);
  
    drawPath(fctx);
    redrawFireAndPOI();
    drawFOV(position, fctx);
    drawPlane(position, fctx);
    
    drawPath(mctx, true);
    drawFOV(position, mctx, true);
    drawPlane(position, mctx, true);
    drawMetrics(position);

    distance += speed;
    if (distance < totalLength) {
        requestAnimationFrame(update);
    }
}

const FIRE_SPREAD_INTERVAL = 1500; // in ms
function start() {
    update();
    setInterval(spreadFire, FIRE_SPREAD_INTERVAL);
}

