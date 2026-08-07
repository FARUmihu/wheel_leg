import {
  DrawingUtils,
  FilesetResolver,
  GestureRecognizer,
} from "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.35";

const TASKS_VERSION = "0.10.35";
const WASM_ROOT = `https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@${TASKS_VERSION}/wasm`;
const MODEL_URL =
  "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task";
const GESTURE_THRESHOLDS = {
  victory: 0.55,
  openPalm: 0.42,
  closedFist: 0.50,
  thumb: 0.48,
};
const HAND_CONNECTIONS = [
  [0, 1], [1, 2], [2, 3], [3, 4],
  [0, 5], [5, 6], [6, 7], [7, 8],
  [5, 9], [9, 10], [10, 11], [11, 12],
  [9, 13], [13, 14], [14, 15], [15, 16],
  [13, 17], [0, 17], [17, 18], [18, 19], [19, 20],
];

class CameraController {
  constructor(videoElement) {
    this.videoElement = videoElement;
    this.stream = null;
  }

  async start() {
    this.stream = await navigator.mediaDevices.getUserMedia({
      video: {
        width: { ideal: 1280 },
        height: { ideal: 720 },
        facingMode: "user",
      },
      audio: false,
    });

    this.videoElement.srcObject = this.stream;
    await this.videoElement.play();
  }

  stop() {
    if (this.stream) {
      this.stream.getTracks().forEach((track) => track.stop());
    }

    this.stream = null;
    this.videoElement.srcObject = null;
  }
}

class GestureService {
  constructor() {
    this.recognizer = null;
  }

  async load() {
    if (this.recognizer) {
      return;
    }

    const vision = await FilesetResolver.forVisionTasks(WASM_ROOT);
    this.recognizer = await GestureRecognizer.createFromOptions(vision, {
      baseOptions: {
        modelAssetPath: MODEL_URL,
        delegate: "GPU",
      },
      runningMode: "VIDEO",
      numHands: 1,
    });
  }

  recognize(videoElement) {
    if (!this.recognizer) {
      return null;
    }

    return this.recognizer.recognizeForVideo(videoElement, performance.now());
  }
}

class CommandMapper {
  map(results) {
    const topGesture = results?.gestures?.[0]?.[0];
    const categoryName = topGesture?.categoryName ?? "None";
    const score = topGesture?.score ?? 0;
    const landmarkCommand = this.mapLandmarks(results?.landmarks?.[0]);

    if (categoryName === "Victory" && score >= GESTURE_THRESHOLDS.victory) {
      return {
        command: "前进",
        cssState: "forward",
        gesture: "V / 耶",
        mapping: "V 手势 -> 前进",
        score,
      };
    }

    if (categoryName === "Open_Palm" && score >= GESTURE_THRESHOLDS.openPalm) {
      return {
        command: "倒退",
        cssState: "backward",
        gesture: "五指张开",
        mapping: "五指张开 -> 倒退",
        score,
      };
    }

    if (categoryName === "Closed_Fist" && score >= GESTURE_THRESHOLDS.closedFist) {
      return {
        command: "停止",
        cssState: "stop",
        gesture: "完全握拳",
        mapping: "握拳 -> 停止移动",
        score,
      };
    }

    if (
      (categoryName === "Thumb_Up" || categoryName === "Thumb_Down") &&
      score >= GESTURE_THRESHOLDS.thumb
    ) {
      return {
        command: "右转",
        cssState: "right",
        gesture: "仅伸大拇指",
        mapping: "大拇指 -> 右转",
        score,
      };
    }

    if (landmarkCommand) {
      return landmarkCommand;
    }

    return {
      command: "待机",
      cssState: "idle",
      gesture: categoryName === "None" ? "--" : categoryName,
      mapping: "未触发",
      score,
    };
  }

  mapLandmarks(landmarks) {
    if (!landmarks) {
      return null;
    }

    const fingers = this.getFingerStates(landmarks);

    if (fingers.thumb && !fingers.index && !fingers.middle && !fingers.ring && !fingers.pinky) {
      return {
        command: "右转",
        cssState: "right",
        gesture: "仅伸大拇指",
        mapping: "大拇指 -> 右转",
        scoreLabel: "关键点",
        score: 0,
      };
    }

    if (fingers.pinky && !fingers.thumb && !fingers.index && !fingers.middle && !fingers.ring) {
      return {
        command: "左转",
        cssState: "left",
        gesture: "仅伸小拇指",
        mapping: "小拇指 -> 左转",
        scoreLabel: "关键点",
        score: 0,
      };
    }

    if (!fingers.thumb && !fingers.index && !fingers.middle && !fingers.ring && !fingers.pinky) {
      return {
        command: "停止",
        cssState: "stop",
        gesture: "完全握拳",
        mapping: "握拳 -> 停止移动",
        scoreLabel: "关键点",
        score: 0,
      };
    }

    return null;
  }

  getFingerStates(landmarks) {
    return {
      thumb: this.isThumbExtended(landmarks),
      index: this.isFingerExtended(landmarks, 8, 6, 5),
      middle: this.isFingerExtended(landmarks, 12, 10, 9),
      ring: this.isFingerExtended(landmarks, 16, 14, 13),
      pinky: this.isFingerExtended(landmarks, 20, 18, 17),
    };
  }

  isFingerExtended(landmarks, tipIndex, pipIndex, mcpIndex) {
    const wrist = landmarks[0];
    const tip = landmarks[tipIndex];
    const pip = landmarks[pipIndex];
    const mcp = landmarks[mcpIndex];
    const tipFromWrist = this.distance(tip, wrist);
    const pipFromWrist = this.distance(pip, wrist);
    const mcpFromWrist = this.distance(mcp, wrist);

    return tipFromWrist > pipFromWrist * 1.08 && tipFromWrist > mcpFromWrist * 1.18;
  }

  isThumbExtended(landmarks) {
    const wrist = landmarks[0];
    const thumbTip = landmarks[4];
    const thumbIp = landmarks[3];
    const thumbMcp = landmarks[2];
    const indexMcp = landmarks[5];
    const palmSize = this.distance(wrist, landmarks[9]);

    return (
      this.distance(thumbTip, wrist) > this.distance(thumbMcp, wrist) * 1.16 &&
      this.distance(thumbTip, thumbIp) > palmSize * 0.16 &&
      this.distance(thumbTip, indexMcp) > palmSize * 0.42
    );
  }

  distance(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }
}

class OverlayPainter {
  constructor(canvasElement) {
    this.canvasElement = canvasElement;
    this.context = canvasElement.getContext("2d");
    this.drawingUtils = new DrawingUtils(this.context);
  }

  resizeTo(videoElement) {
    const width = videoElement.videoWidth || 640;
    const height = videoElement.videoHeight || 480;

    if (this.canvasElement.width !== width || this.canvasElement.height !== height) {
      this.canvasElement.width = width;
      this.canvasElement.height = height;
    }
  }

  paint(videoElement, results) {
    this.resizeTo(videoElement);
    this.context.clearRect(0, 0, this.canvasElement.width, this.canvasElement.height);

    for (const landmarks of results?.landmarks ?? []) {
      this.drawingUtils.drawConnectors(
        landmarks,
        HAND_CONNECTIONS,
        { color: "#31d07f", lineWidth: 4 },
      );
      this.drawingUtils.drawLandmarks(landmarks, {
        color: "#edf2f4",
        fillColor: "#31d07f",
        lineWidth: 2,
        radius: 4,
      });
    }
  }

  clear() {
    this.context.clearRect(0, 0, this.canvasElement.width, this.canvasElement.height);
  }
}

class DemoView {
  constructor() {
    this.elements = {
      camera: document.querySelector("#camera"),
      overlay: document.querySelector("#overlay"),
      statusPill: document.querySelector("#statusPill"),
      commandCard: document.querySelector("#commandCard"),
      commandText: document.querySelector("#commandText"),
      gestureText: document.querySelector("#gestureText"),
      scoreText: document.querySelector("#scoreText"),
      mappingText: document.querySelector("#mappingText"),
      startButton: document.querySelector("#startButton"),
      stopButton: document.querySelector("#stopButton"),
    };
  }

  setStatus(text) {
    this.elements.statusPill.textContent = text;
  }

  setRunning(isRunning) {
    this.elements.startButton.disabled = isRunning;
    this.elements.stopButton.disabled = !isRunning;
  }

  showCommand(commandState) {
    this.elements.commandText.textContent = commandState.command;
    this.elements.gestureText.textContent = commandState.gesture;
    this.elements.scoreText.textContent = commandState.scoreLabel
      || (commandState.score > 0 ? `${Math.round(commandState.score * 100)}%` : "--");
    this.elements.mappingText.textContent = commandState.mapping;

    this.elements.commandCard.className = `command-readout ${commandState.cssState}`;
  }

  showError(message) {
    this.elements.commandText.textContent = message;
    this.elements.gestureText.textContent = "--";
    this.elements.scoreText.textContent = "--";
    this.elements.mappingText.textContent = "演示已停止";
    this.elements.commandCard.className = "command-readout error";
  }
}

class GestureDemoApp {
  constructor() {
    this.view = new DemoView();
    this.camera = new CameraController(this.view.elements.camera);
    this.gestureService = new GestureService();
    this.mapper = new CommandMapper();
    this.overlayPainter = new OverlayPainter(this.view.elements.overlay);
    this.isRunning = false;
    this.lastVideoTime = -1;
  }

  bindEvents() {
    this.view.elements.startButton.addEventListener("click", () => this.start());
    this.view.elements.stopButton.addEventListener("click", () => this.stop());
  }

  async start() {
    try {
      this.view.setStatus("加载模型");
      this.view.setRunning(true);
      await this.gestureService.load();

      this.view.setStatus("打开摄像头");
      await this.camera.start();

      this.isRunning = true;
      this.view.setStatus("识别中");
      this.loop();
    } catch (error) {
      this.stop();
      this.view.showError(error?.message || "启动失败");
      this.view.setStatus("错误");
    }
  }

  stop() {
    this.isRunning = false;
    this.camera.stop();
    this.overlayPainter.clear();
    this.view.setRunning(false);
    this.view.setStatus("未启动");
    this.view.showCommand({
      command: "待机",
      cssState: "idle",
      gesture: "--",
      mapping: "未触发",
      score: 0,
    });
  }

  loop() {
    if (!this.isRunning) {
      return;
    }

    const video = this.view.elements.camera;
    if (video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA) {
      if (video.currentTime !== this.lastVideoTime) {
        this.lastVideoTime = video.currentTime;
        const results = this.gestureService.recognize(video);
        this.overlayPainter.paint(video, results);
        this.view.showCommand(this.mapper.map(results));
      }
    }

    requestAnimationFrame(() => this.loop());
  }
}

const app = new GestureDemoApp();
app.bindEvents();
