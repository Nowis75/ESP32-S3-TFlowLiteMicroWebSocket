const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>ESP32-S3 Stream + FOMO Detection</title>
  <style>
    body {font-family: Arial, sans-serif; background-color: #f0f0f0; text-align: center; margin: 0;padding: 1rem;}
    .video-container {position: relative; display: inline-block;}
    canvas {position: absolute;top: 0;left: 0;}
    #videoCanvas {z-index: 2;}
    #videoImage {position: relative;z-index: 1;}
  </style>
</head>
<body>
  <h1>Live Stream with FOMO Detection</h1>
  <p>Model TensorFlow Lite Micro</p>
  <div class="video-container">
    <img id="videoImage" src="/stream" width="320" height="240" />
    <canvas id="videoCanvas" width="320" height="240"></canvas>
  </div>

  <script>
    const canvas = document.getElementById('videoCanvas');
    const ctx = canvas.getContext('2d');
    const ws = new WebSocket(`ws://${window.location.hostname}:81`);

    const video = new Image();
    video.crossOrigin = "anonymous";
    video.src = "/stream";

    // Parametry modelu a canvasu
    const GRID_SIZE = 12;
    const DIST_THRESHOLD = 1.5;  // max length in grid units / max. vzdálenost v gridových jednotkách
    const CONFIDENCE_THRESHOLD = 0.65;  // filter of light detection / filtr slabých detekcí
    const CELL_WIDTH = canvas.width / GRID_SIZE;
    const CELL_HEIGHT = canvas.height / GRID_SIZE;

    function groupDetections(detections) {
        const groups = [];

        detections.forEach(d => {
            let added = false;

            for (let group of groups) {
            for (let g of group) {
                const dx = d.x - g.x;
                const dy = d.y - g.y;
                const dist = Math.sqrt(dx * dx + dy * dy);
                if (dist <= DIST_THRESHOLD) {
                group.push(d);
                added = true;
                break;
                }
            }
            if (added) break;
            }

            if (!added) groups.push([d]);
        });

        // From every group calsulate sum fo centroid / Z každé skupiny spočítáme průměrný centroid
        const aggregated = groups.map(group => {
            const x = group.reduce((sum, d) => sum + d.x, 0) / group.length;
            const y = group.reduce((sum, d) => sum + d.y, 0) / group.length;
            const confidence = group.reduce((sum, d) => sum + d.confidence, 0) / group.length;
            return { x, y, confidence };
        });

        return aggregated;
    }

    // WebSocket - recieve detections / přijímání detekcí  
    let latestDetections = [];

    ws.onmessage = ({ data }) => {
    try {
        const detections = JSON.parse(data);
        if (Array.isArray(detections)) {
        // Filtrujeme podle confidence
        latestDetections = detections.filter(d => d.confidence >= CONFIDENCE_THRESHOLD);
        }
    } catch (err) {
        console.error("Error in parsing / Chyba při parsování JSON:", err);
    }
    };

    function renderFrame() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        if (video.complete && video.naturalWidth > 0) {
            ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
        }

        const groups = groupDetections(latestDetections);
        groups.forEach(({ x, y, confidence }) => {
            const centerX = (x + 0.5) * CELL_WIDTH;
            const centerY = (y + 0.5) * CELL_HEIGHT;
            const diameter = 8;

            // Centroid
            ctx.beginPath();
            ctx.arc(centerX, centerY, diameter, 0, 2 * Math.PI);
            ctx.fillStyle = "rgba(0, 255, 0, 0.6)";
            ctx.fill();
            ctx.strokeStyle = "#00FF00";
            ctx.stroke();

            // Popisek
            const labelText = `${(confidence * 100).toFixed(1)}%`;
            ctx.font = "12px Arial";
            const textWidth = ctx.measureText(labelText).width;
            const textHeight = 14;

            // Pozadí textu
            ctx.fillStyle = "rgba(0, 0, 0, 0.7)";
            ctx.fillRect(centerX - textWidth / 2 - 4, centerY - diameter - textHeight, textWidth + 8, textHeight);

            // Text
            ctx.fillStyle = "#fff";
            ctx.fillText(labelText, centerX - textWidth / 2, centerY - diameter - 2);
        });

        requestAnimationFrame(renderFrame);
    }


    // Start kreslení
    renderFrame();

  </script>
</body>
</html>
)rawliteral";
