# ESP32-S3-TFlowLiteMicroWebSocket

This project presents an extension and optimization of a system for visualizing object detection results on the ESP32-S3 microcontroller. It builds upon previous work in which the FOMO (Faster Objects, More Objects) model was optimized for embedded systems, and focuses on a complete end-to-end solution — from camera image acquisition, through inference of a quantized neural network model, to real-time visualization of detected objects in a web browser.

The project introduces a dual-threaded architecture utilizing both cores of the ESP32-S3, including a web server for video streaming (MJPEG) and a WebSocket server for real-time transmission of detections. Emphasis is placed on optimizing computational and memory efficiency. The inference runs on a dedicated core, external PSRAM is used for tensor allocation, and tasks are parallelized using FreeRTOS, with data access synchronized through mutexes and communication bandwidth minimized by sending only JSON centroid data.

Implementation details such as image downsampling, normalization for the quantized model, efficient video stream encoding, and thread synchronization are discussed in depth. The prototype achieves a smooth video stream (~30 FPS) with overlaid detection visualization and inference latency in the order of tens of milliseconds per evaluation. The results confirm that even on highly resource-constrained hardware, it is possible to implement a fully functional edge AI application for object detection with real-time visual feedback.

<p align="center">
  <img src="images/ascii_view.png" width="600" alt="Architecture Diagram">
</p>
<p align="center">
  <img src="images/real_view.png" width="600" alt="Architecture Diagram">
</p>
