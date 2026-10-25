// G メーター / 姿勢（ピッチ・ロール）メーター
// IMU の加速度を円グラフ上の点で、ジャイロ主体で求めた車体のピッチ・ロールを車のイラストで表示する。
// IMU は別タスク（コア 0）で 200Hz で読み、描画（loop, コア 1）とは切り離す。SD への記録も コア 0 の別タスク。
// 本体ボタン: A=記録の開始/停止, B=短押しでゼロ点・3 秒長押しで 6 面キャリブレーション, C=モード切替
// シリアル（115200bps）からも 1 文字コマンドで同じ操作ができる（リモートでの動作確認用）:
//   ' ': ゼロ点（停車中に）  c: キャリブレーション  m: モード切替  l: 記録の開始/停止  r: ピーク値のリセット
#include <M5Unified.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <cmath>

static constexpr int CX = 120, CY = 120, R = 110;  // メーター円
static constexpr float RANGE_G = 1.5f;             // 円の外周 = 1.5G
static constexpr float G_LPF_TAU = 0.07f;          // G 表示用ローパスの時定数（秒）
static constexpr int TRAIL = 40;

static constexpr float ATT_RANGE_DEG = 10.0f;  // 姿勢メーターの目盛り範囲（±）
static constexpr float VIS_GAIN = 3.0f;        // 車のイラストの傾きの強調倍率

static constexpr uint32_t IMU_PERIOD_MS = 5;  // IMU タスクの周期（200Hz）

static M5Canvas canvas(&M5.Display);
static Preferences prefs;

struct Vec3 {
  float x, y, z;
};
static Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static Vec3 normalize(Vec3 a) { return a * (1.0f / std::sqrt(dot(a, a))); }
static float len(Vec3 a) { return std::sqrt(dot(a, a)); }

// センサー各軸の補正値: 補正後 = (生値 - offset) / scale
static float calOffset[3] = {0, 0, 0};
static float calScale[3] = {1, 1, 1};
static bool calLoaded = false;

// ---- IMU タスク側の状態（IMU タスクだけが触る） ----
// ゼロ点時の姿勢を基準面とする。up/fwd/right = 基準面の座標軸（本体座標系）
// (right, fwd, up) は右手系。pitch: ノーズアップが正、roll: 右側が下がるのが正
static Vec3 g0 = {0, 0, 1};
static Vec3 up = {0, 0, 1}, fwd = {0, 1, 0}, right = {1, 0, 0};
static Vec3 gEst = {0, 0, 1};  // 推定した「上」方向（本体座標系の単位ベクトル）。ジャイロで回転させ、加速度で補正する
static Vec3 gyroBias = {0, 0, 0};
static Vec3 accMean = {0, 0, 1}, gyroMean = {0, 0, 0};
static float accVar = 1, gyroVar = 100;  // 起動直後は静止と判定しないよう大きめに置く
static uint32_t quietSince = 0;
static uint32_t peakSetAt = 0;
static constexpr uint32_t PEAK_HOLD_MS = 20000;  // PEAK G の保持時間
static uint32_t lastMicros = 0;

// ---- 描画側と共有するスナップショット（mux で保護） ----
struct Snapshot {
  Vec3 raw = {0, 0, 1};    // 加速度の生値
  Vec3 gyro = {0, 0, 0};   // ジャイロの生値（deg/s）
  Vec3 acc = {0, 0, 1};    // 補正後の加速度
  float fx = 0, fy = 0, fz = 0;  // 基準面上の動的加速度（左右 / 前後 / 上下）
  float peakG = 0;
  float pitch = 0, roll = 0;
  float accPitch = 0, accRoll = 0;  // 加速度だけから求めた角度（検証用）
  float pitchMin = 0, pitchMax = 0, rollMin = 0, rollMax = 0;
  float rateP = 0, rateR = 0;
  float accSd = 1, gyroSd = 10;
  Vec3 bias = {0, 0, 0};
  bool stationary = false;
  bool zeroing = false;
  int zeroAttempt = 0;       // ゼロ点の測定回数（1 から）
  float zeroProgress = 0;    // 今回の測定の進み具合 0..1
  bool zeroMoving = false;   // 前回の測定で揺れを検出した
  uint32_t zeroOkAt = 0, zeroFailAt = 0;
  uint32_t samples = 0;
};
static Snapshot shared;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool zeroRequest = false, peakResetRequest = false;

// ピッチ・ロールの履歴（推移グラフ用）。IMU タスクが 40Hz で書き込む（mux で保護）
static constexpr int HIST = 400;               // 40Hz × 10 秒
static constexpr int HIST_DECIM = 5;           // 200Hz を 1/5 に間引く
static float histP[HIST], histR[HIST];  // ピッチ・ロール（度）
static float histFB[HIST], histLR[HIST];  // 前後・左右の G（G メーターのボールの上・右が正）
static float histG[HIST];                 // 水平方向の合成 G（大きさ）
static int histHead = 0, histCount = 0;

// C ボタンでこの順に切り替わる
// ---- SD カードへの記録 ----
// IMU タスクが 200Hz のサンプルを LOG_DECIM 個ずつ平均して 50Hz の LogRec をリングバッファに積み、
// SD タスク（コア 0, 低優先度）が CSV にして書き出す。単純に間引くと 25Hz 超の振動が偽の遅い揺れとして
// 記録される（エイリアシング）ため、平均してから記録する。5Hz 以下の車体の動きは 98% 以上そのまま残る
// SD と液晶は SPI を共有しているので、SD アクセスと pushSprite は spiMutex で排他する
static constexpr int LOG_DECIM = 4;     // 200Hz → 50Hz（4 サンプル平均）
static constexpr int LOG_RATE_HZ = 200 / LOG_DECIM;
static constexpr int LOG_BUF = 800;     // 16 秒分
struct LogRec {
  uint32_t ms;
  float pitch, roll, fb, lr, ud;
  float gx, gy, gz;  // バイアス補正後の角速度（deg/s, 本体座標）
  float ax, ay, az;  // 補正後の加速度（G, 本体座標）
  uint16_t zeroId;
  bool still;
};
static LogRec logBuf[LOG_BUF];
static int logHead = 0, logTail = 0;     // mux で保護
static uint32_t logDropped = 0;          // バッファあふれで捨てた件数（mux で保護）
static volatile bool logging = false;    // true の間だけ IMU タスクが積む
static volatile bool logToggleRequest = false;
static uint16_t zeroId = 0;              // ゼロ点を取り直すたびに増える（ログで区間を見分ける）
static SemaphoreHandle_t spiMutex;

// SD_SAVING: 停止要求後、書き出し・クローズ中（抜いてはいけない） / SD_SAFE: アンマウント済み（抜いてよい）
enum SdState { SD_INIT, SD_NONE, SD_READY, SD_REC, SD_ERROR, SD_SAVING, SD_SAFE };
static uint32_t sdMsgAt = 0;  // SD 関連の通知パネルを出し始めた時刻
static volatile SdState sdState = SD_INIT;
static char logPath[24] = "";
static uint32_t logSession = 0;  // 起動ごとに 1 増える（SD 認識時に決める）
static uint32_t logSeq = 0;      // セッション内の記録番号
static uint32_t logStartMs = 0, logLines = 0, logMaxWriteMs = 0;
static int logMaxFill = 0;

enum Mode { MODE_ATTITUDE, MODE_TREND, MODE_GMETER, MODE_GTREND, MODE_DEBUG, MODE_COUNT };
static int mode = MODE_ATTITUDE;
static int trailX[TRAIL], trailY[TRAIL], trailHead = 0, trailCount = 0;

// 6 面キャリブレーションの状態。面の順番は +X, -X, +Y, -Y, +Z, -Z（その軸が上向き = +1G を読む向き）
static bool calMode = false;
static bool calDone[6];
static float calValue[6];
static const char* calMsg = "";
static uint32_t calMsgColor = TFT_WHITE;
static const char* FACE_NAMES[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

static Snapshot snapshot() {
  portENTER_CRITICAL(&mux);
  Snapshot s = shared;
  portEXIT_CRITICAL(&mux);
  return s;
}

// 加速度（生値, G）とジャイロ（deg/s）を同じサンプルから読む
static void readImu(Vec3& acc, Vec3& gyro) {
  float ax, ay, az, gx, gy, gz;
  M5.Imu.update();
  M5.Imu.getAccel(&ax, &ay, &az);
  M5.Imu.getGyro(&gx, &gy, &gz);
  acc = {ax, ay, az};
  gyro = {gx, gy, gz};
}

static Vec3 applyCal(Vec3 r) {
  return {(r.x - calOffset[0]) / calScale[0], (r.y - calOffset[1]) / calScale[1], (r.z - calOffset[2]) / calScale[2]};
}

static void loadCal() {
  prefs.begin("imucal", true);
  calLoaded = prefs.isKey("off");
  if (calLoaded) {
    prefs.getBytes("off", calOffset, sizeof(calOffset));
    prefs.getBytes("scl", calScale, sizeof(calScale));
  }
  prefs.end();
  Serial.printf("cal %s: offset=(%.3f %.3f %.3f) scale=(%.3f %.3f %.3f)\n", calLoaded ? "loaded" : "none",
                calOffset[0], calOffset[1], calOffset[2], calScale[0], calScale[1], calScale[2]);
}

static void saveCal() {
  prefs.begin("imucal", false);
  prefs.putBytes("off", calOffset, sizeof(calOffset));
  prefs.putBytes("scl", calScale, sizeof(calScale));
  prefs.end();
  calLoaded = true;
}

static float pitchOf(Vec3 g) { return std::atan2(dot(g, fwd), std::sqrt(dot(g, right) * dot(g, right) + dot(g, up) * dot(g, up))) * RAD_TO_DEG; }
static float rollOf(Vec3 g) { return std::atan2(-dot(g, right), dot(g, up)) * RAD_TO_DEG; }

static void resetPeaksLocked(Snapshot& s) {
  s.peakG = 0;
  s.pitchMin = s.pitchMax = s.pitch;
  s.rollMin = s.rollMax = s.roll;
}

// ボタンを押した振動が収まるのを待ってから、0.5 秒平均して重力ベクトルとジャイロバイアスを求める。
// 揺れていたら測り直し、5 回やっても収まらなければゼロ点は更新しない（壊れたバイアスで確定させない）
static void doZero() {
  portENTER_CRITICAL(&mux);
  shared.zeroing = true;
  shared.zeroMoving = false;
  portEXIT_CRITICAL(&mux);

  constexpr int N = 100;
  Vec3 sa, sg;
  float gyroSd = 0;
  bool ok = false;
  for (int attempt = 0; attempt < 5 && !ok; attempt++) {
    portENTER_CRITICAL(&mux);
    shared.zeroAttempt = attempt + 1;
    shared.zeroProgress = 0;
    portEXIT_CRITICAL(&mux);
    vTaskDelay(pdMS_TO_TICKS(attempt == 0 ? 700 : 300));
    sa = {0, 0, 0};
    sg = {0, 0, 0};
    Vec3 sg2 = {0, 0, 0};
    for (int i = 0; i < N; i++) {
      Vec3 a, g;
      readImu(a, g);
      sa = sa + applyCal(a);
      sg = sg + g;
      sg2 = sg2 + Vec3{g.x * g.x, g.y * g.y, g.z * g.z};
      portENTER_CRITICAL(&mux);
      shared.zeroProgress = (i + 1) / (float)N;
      portEXIT_CRITICAL(&mux);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    Vec3 m = sg * (1.0f / N);
    Vec3 var = sg2 * (1.0f / N) - Vec3{m.x * m.x, m.y * m.y, m.z * m.z};
    gyroSd = std::sqrt(std::max(0.0f, var.x + var.y + var.z));
    ok = gyroSd < 0.5f;
    if (!ok) {
      Serial.printf("zero: moving (gyro sd %.2f), retry\n", gyroSd);
      portENTER_CRITICAL(&mux);
      shared.zeroMoving = true;
      portEXIT_CRITICAL(&mux);
    }
  }

  if (!ok) {
    Serial.println("zero: failed, keep previous");
    portENTER_CRITICAL(&mux);
    shared.zeroing = false;
    shared.zeroFailAt = millis();
    portEXIT_CRITICAL(&mux);
    lastMicros = micros();
    return;
  }

  g0 = sa * (1.0f / N);
  gyroBias = sg * (1.0f / N);

  // 重力の向きを鉛直とし、本体 X 軸（画面の左右）を水平面に投影したものを右方向とする。
  // 平置きでも縦置きでも画面の左右は水平に近いので、前方は up と right から決める。
  up = normalize(g0);
  Vec3 x = {1, 0, 0};
  if (std::fabs(dot(x, up)) < 0.95f) {
    right = normalize(x - up * dot(x, up));
    fwd = cross(up, right);
  } else {
    // X 軸がほぼ鉛直（横倒し）のときは Y 軸を前方の基準にする
    Vec3 y = {0, 1, 0};
    fwd = normalize(y - up * dot(y, up));
    right = cross(fwd, up);
  }
  gEst = up;
  accMean = g0;
  gyroMean = gyroBias;
  quietSince = millis();
  lastMicros = micros();
  Serial.printf("zero set: g0=(%.3f %.3f %.3f) fwd=(%.2f %.2f %.2f) right=(%.2f %.2f %.2f) gyroBias=(%.2f %.2f %.2f) sd=%.2f\n",
                g0.x, g0.y, g0.z, fwd.x, fwd.y, fwd.z, right.x, right.y, right.z, gyroBias.x, gyroBias.y,
                gyroBias.z, gyroSd);

  portENTER_CRITICAL(&mux);
  shared.fx = shared.fy = shared.fz = 0;
  shared.pitch = shared.roll = 0;
  shared.stationary = false;
  resetPeaksLocked(shared);
  shared.zeroing = false;
  shared.zeroOkAt = millis();
  zeroId++;
  histCount = 0;
  portEXIT_CRITICAL(&mux);
}

// 1 サンプル分の処理。重力方向 gEst をジャイロで回転させ、加速していないと判断できるときだけ加速度で補正する。
// ブレーキ中は加速度が傾きと区別できない（0.5G で約 27° 相当）ため、補正をかけない。
static void imuStep(float dt) {
  Vec3 raw, w;
  readImu(raw, w);
  Vec3 a = applyCal(raw);

  // 本体が角速度 ω で回ると、空間に固定されたベクトルは本体座標系で dv/dt = -ω × v で動く
  Vec3 wc = w - gyroBias;
  gEst = normalize(gEst - cross(wc * DEG_TO_RAD, gEst) * dt);

  // 加速度とジャイロのばらつき（約 0.5 秒の指数移動分散）が小さく、合成 G がほぼ 1 の状態が 1 秒続いたら
  // 静止（停車中・定速直進）とみなす。ジャイロは値そのものでなくばらつきで見るので、
  // バイアスがずれていても静止を検出でき、下のバイアス追従で修正される
  float alpha = std::min(1.0f, dt / 0.5f);
  Vec3 da = a - accMean, dw = w - gyroMean;
  accMean = accMean + da * alpha;
  gyroMean = gyroMean + dw * alpha;
  accVar += alpha * (dot(da, da) - accVar);
  gyroVar += alpha * (dot(dw, dw) - gyroVar);
  float an = len(a);
  float gErr = std::fabs(an - 1.0f);
  // 合成 G の 1G からのずれは温度で数 % 変わることがあるので、静止判定はばらつき主体にして G の条件は緩くする
  bool quiet = gErr < 0.1f && std::sqrt(accVar) < 0.015f && std::sqrt(gyroVar) < 1.0f;
  if (!quiet) quietSince = millis();
  bool stationary = millis() - quietSince > 1000;

  Vec3 aN = a * (1.0f / an);
  float diffDeg = std::acos(std::max(-1.0f, std::min(1.0f, dot(aN, gEst)))) * RAD_TO_DEG;
  float tau = 0;
  if (stationary) {
    tau = 0.5f;  // 静止中はしっかり補正してジャイロの積算誤差を消す
  } else if (gErr < 0.05f && diffDeg < 3.0f) {
    tau = 5.0f;  // 走行中は加速度とジャイロの向きが一致しているときだけゆっくり補正
  }
  if (tau > 0) gEst = normalize(gEst + (aN - gEst) * (dt / tau));
  // 静止中はジャイロバイアスにも追従させる（揺れ中に取ったゼロ点や温度変化の修正）
  if (stationary) gyroBias = gyroBias + (w - gyroBias) * (dt / 3.0f);

  float pitch = pitchOf(gEst), roll = rollOf(gEst);
  Vec3 d = a - g0;
  float k = std::min(1.0f, dt / G_LPF_TAU);

  portENTER_CRITICAL(&mux);
  Snapshot& s = shared;
  s.raw = raw;
  s.gyro = w;
  s.acc = a;
  // 重力分を引いた動的加速度を基準面の座標軸へ投影する（左右・前後は画面の向きに合わせて反転）
  s.fx += k * (-dot(d, right) - s.fx);
  s.fy += k * (-dot(d, fwd) - s.fy);
  s.fz += k * (dot(d, up) - s.fz);
  // PEAK G は最後に更新されてから PEAK_HOLD_MS たったら現在値にリセットする
  float gNow = std::sqrt(s.fx * s.fx + s.fy * s.fy);
  if (gNow > s.peakG) {
    s.peakG = gNow;
    peakSetAt = millis();
  } else if (millis() - peakSetAt > PEAK_HOLD_MS) {
    s.peakG = gNow;
    peakSetAt = millis();
  }
  s.pitch = pitch;
  s.roll = roll;
  s.accPitch = pitchOf(aN);
  s.accRoll = rollOf(aN);
  s.pitchMin = std::min(s.pitchMin, pitch);
  s.pitchMax = std::max(s.pitchMax, pitch);
  s.rollMin = std::min(s.rollMin, roll);
  s.rollMax = std::max(s.rollMax, roll);
  s.rateP = dot(wc, right);
  s.rateR = dot(wc, fwd);
  s.accSd = std::sqrt(accVar);
  s.gyroSd = std::sqrt(gyroVar);
  s.bias = gyroBias;
  s.stationary = stationary;
  s.samples++;
  if (s.samples % HIST_DECIM == 0) {
    histP[histHead] = pitch;
    histR[histHead] = roll;
    histFB[histHead] = s.fy;
    histLR[histHead] = s.fx;
    histG[histHead] = std::sqrt(s.fx * s.fx + s.fy * s.fy);
    histHead = (histHead + 1) % HIST;
    if (histCount < HIST) histCount++;
  }
  // ログ用: LOG_DECIM サンプル分を合計し、そろったら平均を積む（時刻は区間の中央、still / zeroId は最新値）
  static float sum[11];
  static uint32_t sumMs = 0;
  static int sumN = 0;
  if (!logging) {
    sumN = 0;
  } else {
    const float v[11] = {pitch, roll, s.fy, s.fx, s.fz, wc.x, wc.y, wc.z, a.x, a.y, a.z};
    uint32_t now = millis();
    if (sumN == 0) {
      for (float& x : sum) x = 0;
      sumMs = now;
    }
    for (int i = 0; i < 11; i++) sum[i] += v[i];
    if (++sumN == LOG_DECIM) {
      float k = 1.0f / LOG_DECIM;
      int next = (logHead + 1) % LOG_BUF;
      if (next == logTail) {
        logDropped++;
      } else {
        logBuf[logHead] = {sumMs + (now - sumMs) / 2, sum[0] * k, sum[1] * k, sum[2] * k, sum[3] * k, sum[4] * k,
                           sum[5] * k, sum[6] * k, sum[7] * k, sum[8] * k, sum[9] * k, sum[10] * k, zeroId,
                           stationary};
        logHead = next;
        logMaxFill = std::max(logMaxFill, (logHead - logTail + LOG_BUF) % LOG_BUF);
      }
      sumN = 0;
    }
  }
  portEXIT_CRITICAL(&mux);
}

// IMU の I2C アクセスはすべてこのタスクで行う（描画側とバスを取り合わないように）
static void imuTask(void*) {
  zeroRequest = true;  // 起動時にゼロ点を取る
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));

    if (zeroRequest) {
      zeroRequest = false;
      doZero();
      lastWake = xTaskGetTickCount();
      continue;
    }
    if (peakResetRequest) {
      peakResetRequest = false;
      portENTER_CRITICAL(&mux);
      resetPeaksLocked(shared);
      portEXIT_CRITICAL(&mux);
    }

    uint32_t now = micros();
    float dt = std::min(0.05f, (now - lastMicros) * 1e-6f);
    lastMicros = now;
    imuStep(dt);
  }
}

// セッション番号を決める。Preferences に前回の番号を覚え、その次から SD 上に未使用の番号を探す
static void startSession() {
  prefs.begin("log", false);
  uint32_t sess = prefs.getUInt("session", 0);
  char path[24];
  for (int tries = 0; tries < 1000; tries++) {
    sess = sess % 9999 + 1;
    snprintf(path, sizeof(path), "/log_%04u_01.csv", (unsigned)sess);
    if (!SD.exists(path)) break;
  }
  prefs.putUInt("session", sess);
  prefs.end();
  logSession = sess;
  logSeq = 0;
}

// ファイル名は /log_{セッション番号4桁}_{セッション内の記録番号2桁}.csv
static bool openNewLog(File& f) {
  do {
    logSeq++;
    snprintf(logPath, sizeof(logPath), "/log_%04u_%02u.csv", (unsigned)logSession, (unsigned)logSeq);
  } while (SD.exists(logPath) && logSeq < 99);
  f = SD.open(logPath, FILE_WRITE);
  if (!f) return false;
  f.printf("# session %u seq %u\n", (unsigned)logSession, (unsigned)logSeq);
  f.printf("# M5Stack G-meter log. %dHz (average of %d samples at 200Hz). cal offset=(%.4f %.4f %.4f) scale=(%.4f %.4f %.4f)\n", LOG_RATE_HZ, LOG_DECIM, calOffset[0],
           calOffset[1], calOffset[2], calScale[0], calScale[1], calScale[2]);
  f.print("# pitch: nose up +, roll: right down +, fb: ball up +, lr: ball right +, ud: up + (G, gravity removed)\n");
  f.print("t_ms,pitch_deg,roll_deg,g_fb,g_lr,g_ud,g_comb,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,still,zero_id\n");
  logStartMs = millis();
  logLines = 0;
  return true;
}

// SD をマウントする（液晶と SPI を共有しているので排他する）。速度を落としながら数回試す
// M5Stack Basic/Gray の microSD: CS=4, SCK=18, MISO=19, MOSI=23（液晶と同じ SPI バス）
static bool mountSd() {
  const uint32_t freqs[] = {20000000, 10000000, 4000000};
  for (uint32_t f : freqs) {
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    SPI.begin(18, 19, 23, 4);
    bool ok = SD.begin(GPIO_NUM_4, SPI, f);
    uint8_t type = ok ? SD.cardType() : CARD_NONE;
    if (!ok || type == CARD_NONE) SD.end();
    xSemaphoreGive(spiMutex);
    Serial.printf("sd: begin %luHz -> %s type=%u\n", (unsigned long)f, ok ? "ok" : "fail", type);
    if (ok && type != CARD_NONE) {
      Serial.printf("sd: card %lluMB\n", SD.cardSize() / (1024 * 1024));
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  return false;
}

static void unmountSd() {
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  SD.end();
  xSemaphoreGive(spiMutex);
}

static bool startRecording(File& file) {
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  if (logSession == 0) startSession();
  bool opened = openNewLog(file);
  xSemaphoreGive(spiMutex);
  if (!opened) return false;
  portENTER_CRITICAL(&mux);
  logHead = logTail = 0;
  logDropped = 0;
  portEXIT_CRITICAL(&mux);
  logging = true;
  sdState = SD_REC;
  Serial.printf("sd: recording %s (session %u)\n", logPath, (unsigned)logSession);
  return true;
}

static void sdTask(void*) {
  static LogRec recs[100];
  static char out[100 * 112];
  File file;
  uint32_t lastFlush = 0;

  // 起動時: カードがあればマウントして自動で記録開始
  if (mountSd()) {
    sdState = SD_READY;
    logToggleRequest = true;
  } else {
    sdState = SD_NONE;
    Serial.println("sd: no card");
  }

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));

    if (logToggleRequest) {
      logToggleRequest = false;
      SdState st = sdState;
      if (st == SD_REC) {
        // 停止: 残りを書き出して閉じ、アンマウントしてから SAFE にする（下の処理で行う）
        logging = false;
        sdState = SD_SAVING;
      } else if (st != SD_SAVING) {
        // 開始: 停止後・カード無し・エラー後はマウントからやり直す（カードの差し替えにも対応）
        bool mounted = (st == SD_READY) || mountSd();
        if (!mounted) {
          sdState = SD_NONE;
          sdMsgAt = millis();
          Serial.println("sd: no card");
        } else if (!startRecording(file)) {
          unmountSd();
          sdState = SD_ERROR;
          sdMsgAt = millis();
          Serial.println("sd: open failed");
        }
      }
    }

    // バッファから取り出して CSV にし、まとめて書く
    for (;;) {
      int n = 0;
      portENTER_CRITICAL(&mux);
      while (logTail != logHead && n < 100) {
        recs[n++] = logBuf[logTail];
        logTail = (logTail + 1) % LOG_BUF;
      }
      portEXIT_CRITICAL(&mux);
      if (n == 0 || !file) break;
      int len = 0;
      for (int i = 0; i < n; i++) {
        const LogRec& r = recs[i];
        len += snprintf(out + len, sizeof(out) - len,
                        "%lu,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%d,%u\n",
                        (unsigned long)(r.ms - logStartMs), r.pitch, r.roll, r.fb, r.lr, r.ud,
                        std::sqrt(r.fb * r.fb + r.lr * r.lr), r.gx, r.gy, r.gz, r.ax, r.ay, r.az, r.still ? 1 : 0,
                        (unsigned)r.zeroId);
      }
      uint32_t t0 = millis();
      xSemaphoreTake(spiMutex, portMAX_DELAY);
      size_t w = file.write((const uint8_t*)out, len);
      xSemaphoreGive(spiMutex);
      logMaxWriteMs = std::max<uint32_t>(logMaxWriteMs, millis() - t0);
      logLines += n;
      if (w != (size_t)len) {
        // 書き込み失敗（カードが抜かれた等）: 閉じてアンマウントし、エラー表示
        logging = false;
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        file.close();
        SD.end();
        xSemaphoreGive(spiMutex);
        sdState = SD_ERROR;
        sdMsgAt = millis();
        Serial.println("sd: write failed");
        break;
      }
    }

    // 電源断に備えて 1 秒ごとに確定させる。停止時は閉じてアンマウントし、抜いてよい状態にする
    if (file && logging && millis() - lastFlush > 1000) {
      lastFlush = millis();
      xSemaphoreTake(spiMutex, portMAX_DELAY);
      file.flush();
      xSemaphoreGive(spiMutex);
    } else if (sdState == SD_SAVING) {
      xSemaphoreTake(spiMutex, portMAX_DELAY);
      if (file) file.close();
      Serial.printf("sd: closed %s (%lu lines)\n", logPath, (unsigned long)logLines);
      // 確認用: 閉じたファイルを読み返し、サイズと先頭・末尾を出す
      File rf = SD.open(logPath, FILE_READ);
      if (rf) {
        size_t size = rf.size();
        Serial.printf("sd: verify size=%u\n", (unsigned)size);
        for (int i = 0; i < 5 && rf.available(); i++) Serial.printf("sd>  %s\n", rf.readStringUntil('\n').c_str());
        if (size > 300) {
          rf.seek(size - 300);
          rf.readStringUntil('\n');
          String last;
          while (rf.available()) last = rf.readStringUntil('\n');
          Serial.printf("sd>  ... %s\n", last.c_str());
        }
        rf.close();
      }
      SD.end();
      xSemaphoreGive(spiMutex);
      sdState = SD_SAFE;
      sdMsgAt = millis();
      Serial.println("sd: unmounted, safe to remove");
    }

    static uint32_t lastStat = 0;
    if (millis() - lastStat > 5000) {
      lastStat = millis();
      portENTER_CRITICAL(&mux);
      uint32_t dropped = logDropped;
      int fill = logMaxFill;
      portEXIT_CRITICAL(&mux);
      Serial.printf("sd: state=%d lines=%lu dropped=%lu maxFill=%d maxWrite=%lums\n", (int)sdState,
                    (unsigned long)logLines, (unsigned long)dropped, fill, (unsigned long)logMaxWriteMs);
    }
  }
}

static void startCal() {
  calMode = true;
  for (int i = 0; i < 6; i++) calDone[i] = false;
  calMsg = "Place a face down, press B";
  calMsgColor = TFT_WHITE;
}

// 静止中の生値を平均し、重力が最も強くかかっている軸の面として記録する
static void captureCalFace() {
  delay(500);  // ボタンを押した振動が収まるのを待つ
  constexpr int N = 100;
  float sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
  for (int i = 0; i < N; i++) {
    Vec3 r = snapshot().raw;
    float v[3] = {r.x, r.y, r.z};
    for (int k = 0; k < 3; k++) {
      sum[k] += v[k];
      sq[k] += v[k] * v[k];
    }
    delay(10);
  }
  float mean[3], maxSd = 0;
  for (int k = 0; k < 3; k++) {
    mean[k] = sum[k] / N;
    maxSd = std::max(maxSd, std::sqrt(std::max(0.0f, sq[k] / N - mean[k] * mean[k])));
  }
  if (maxSd > 0.02f) {
    calMsg = "Moving! Hold still, press B";
    calMsgColor = TFT_RED;
    return;
  }

  // 生値は Y 軸に 0.3G ほどずれがあるため、オフセットの影響を受けにくい「最も大きい軸」で判定する
  int axis = 0;
  for (int k = 1; k < 3; k++) {
    if (std::fabs(mean[k]) > std::fabs(mean[axis])) axis = k;
  }
  int face = axis * 2 + (mean[axis] > 0 ? 0 : 1);
  calDone[face] = true;
  calValue[face] = mean[axis];
  Serial.printf("cal face %s: %.4f (sd %.4f)\n", FACE_NAMES[face], mean[axis], maxSd);

  for (int i = 0; i < 6; i++) {
    if (!calDone[i]) {
      calMsg = "OK. Next face, press B";
      calMsgColor = TFT_GREEN;
      return;
    }
  }

  // 6 面そろったら各軸の +1G / -1G の値からオフセットとスケールを求める
  float off[3], scl[3];
  for (int k = 0; k < 3; k++) {
    float p = calValue[k * 2], n = calValue[k * 2 + 1];
    off[k] = (p + n) / 2;
    scl[k] = (p - n) / 2;
  }
  portENTER_CRITICAL(&mux);
  for (int k = 0; k < 3; k++) {
    calOffset[k] = off[k];
    calScale[k] = scl[k];
  }
  portEXIT_CRITICAL(&mux);
  saveCal();
  Serial.printf("cal saved: offset=(%.3f %.3f %.3f) scale=(%.3f %.3f %.3f)\n", calOffset[0], calOffset[1],
                calOffset[2], calScale[0], calScale[1], calScale[2]);
  calMode = false;
  zeroRequest = true;
}

static char modeLabel[12];
static const char* modeButtonLabel() {
  snprintf(modeLabel, sizeof(modeLabel), "MODE %d/%d", mode + 1, (int)MODE_COUNT);
  return modeLabel;
}

// 画面を液晶へ送る。SD と SPI を共有しているので排他する
static void pushCanvas() {
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  canvas.pushSprite(0, 0);
  xSemaphoreGive(spiMutex);
}

// 記録状態（左下）: 記録中は赤丸と経過時間
static void drawRecStatus() {
  canvas.setTextSize(1);
  canvas.setTextDatum(textdatum_t::bottom_left);
  SdState st = sdState;
  if (st == SD_REC) {
    uint32_t sec = (millis() - logStartMs) / 1000;
    if ((millis() / 500) % 2 == 0) canvas.fillCircle(6, 234, 4, TFT_RED);
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu:%02lu", (unsigned long)(sec / 60), (unsigned long)(sec % 60));
    canvas.setTextColor(TFT_RED);
    canvas.drawString(buf, 13, 239);
  } else if (st == SD_SAVING) {
    canvas.setTextColor(TFT_YELLOW);
    canvas.drawString("SAVING", 2, 239);
  } else if (st == SD_SAFE) {
    canvas.setTextColor(TFT_GREEN);
    canvas.drawString("SAFE", 2, 239);
  } else if (st == SD_NONE) {
    canvas.setTextColor(TFT_DARKGREY);
    canvas.drawString("NO SD", 2, 239);
  } else if (st == SD_ERROR) {
    canvas.setTextColor(TFT_RED);
    canvas.drawString("SD ERR", 2, 239);
  } else if (st == SD_READY) {
    canvas.setTextColor(TFT_LIGHTGREY);
    canvas.drawString("REC off", 2, 239);
  }
  canvas.setTextDatum(textdatum_t::top_left);
}

static float calHoldProgress = 0;  // B ボタン長押しの進み具合（0 なら非表示）
static constexpr uint32_t CAL_HOLD_MS = 3000;

static void drawButtonLabels(const char* a, const char* b, const char* c);

// 通常画面のボタン表示: A=記録, B=ゼロ点（長押しでキャリブレーション）, C=モード
static void drawMainButtons() {
  SdState st = sdState;
  const char* a = st == SD_REC ? "STOP" : (st == SD_SAVING ? "SAVING" : "REC");
  drawButtonLabels(a, "ZERO / CAL 3s", modeButtonLabel());
}

// 本体ボタン A/B/C の真上（画面最下段）に役割を表示する
static void drawButtonLabels(const char* a, const char* b, const char* c) {
  const int xs[3] = {68, 160, 252};
  const char* labels[3] = {a, b, c};
  canvas.setTextSize(1);
  canvas.setTextDatum(textdatum_t::bottom_center);
  for (int i = 0; i < 3; i++) {
    if (!labels[i][0]) continue;
    int w = canvas.textWidth(labels[i]) + 8;
    canvas.fillRoundRect(xs[i] - w / 2, 229, w, 11, 3, TFT_DARKGREY);
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(labels[i], xs[i], 239);
  }
  canvas.setTextDatum(textdatum_t::top_left);
  drawRecStatus();
}

static void drawCal(const Snapshot& s) {
  canvas.fillScreen(TFT_BLACK);
  canvas.setTextSize(2);
  canvas.setTextColor(TFT_YELLOW);
  canvas.setCursor(8, 8);
  canvas.print("6-FACE CALIBRATION");

  canvas.setTextColor(TFT_CYAN);
  canvas.setCursor(8, 40);
  canvas.printf("raw %+.2f %+.2f %+.2f", s.raw.x, s.raw.y, s.raw.z);

  // 各面の取得状況（その軸を上に向けて置く）
  for (int i = 0; i < 6; i++) {
    int col = i % 2, row = i / 2;
    canvas.setCursor(8 + col * 156, 76 + row * 30);
    canvas.setTextColor(calDone[i] ? TFT_GREEN : TFT_DARKGREY);
    if (calDone[i]) {
      canvas.printf("%s %+.3f", FACE_NAMES[i], calValue[i]);
    } else {
      canvas.printf("%s  ----", FACE_NAMES[i]);
    }
  }

  canvas.setTextColor(calMsgColor);
  canvas.setCursor(8, 176);
  canvas.print(calMsg);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_DARKGREY);
  canvas.setCursor(8, 212);
  canvas.print("Hold each face still ~1s while capturing.");
  drawButtonLabels("", "CAPTURE", "CANCEL");
  pushCanvas();
}

// 画面中央の大きな通知パネル
static void drawBanner(uint32_t bg, uint32_t fg, const char* line1, const char* line2) {
  canvas.fillRoundRect(20, 60, 280, 120, 10, bg);
  canvas.drawRoundRect(20, 60, 280, 120, 10, TFT_WHITE);
  canvas.setTextColor(fg);
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.setFont(&fonts::FreeSansBold18pt7b);
  canvas.drawString(line1, 160, 95);
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.drawString(line2, 160, 135);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(textdatum_t::top_left);
}

// ゼロ点取得中・成功・失敗の表示
static void drawZeroStatus(const Snapshot& s) {
  if (calHoldProgress > 0) {
    drawBanner(TFT_NAVY, TFT_WHITE, "Hold for CAL", "Release now = ZERO");
    canvas.drawRect(40, 158, 240, 12, TFT_WHITE);
    canvas.fillRect(42, 160, (int)(236 * calHoldProgress), 8, TFT_WHITE);
    return;
  }
  SdState sd = sdState;
  if (sd == SD_SAVING) {
    drawBanner(TFT_YELLOW, TFT_BLACK, "SAVING...", "Do not remove SD card");
    return;
  }
  if (millis() - sdMsgAt < 4000) {
    if (sd == SD_SAFE) {
      drawBanner(TFT_DARKGREEN, TFT_WHITE, "SAVED", "Safe to remove SD card");
      return;
    } else if (sd == SD_NONE && sdMsgAt) {
      drawBanner(TFT_RED, TFT_WHITE, "NO SD CARD", "Insert card and press REC");
      return;
    } else if (sd == SD_ERROR) {
      drawBanner(TFT_RED, TFT_WHITE, "SD ERROR", "Check card and press REC");
      return;
    }
  }
  if (s.zeroing) {
    drawBanner(TFT_ORANGE, TFT_BLACK, "ZEROING...", s.zeroMoving ? "Moving! Please hold still" : "Do not move");
    // 進み具合バーと測定回数
    canvas.drawRect(40, 158, 240, 12, TFT_BLACK);
    canvas.fillRect(42, 160, (int)(236 * s.zeroProgress), 8, TFT_BLACK);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_BLACK);
    canvas.setCursor(250, 66);
    canvas.printf("%d/5", s.zeroAttempt);
  } else if (s.zeroFailAt && millis() - s.zeroFailAt < 3000) {
    drawBanner(TFT_RED, TFT_WHITE, "ZERO FAILED", "Hold still and press B again");
  } else if (s.zeroOkAt && millis() - s.zeroOkAt < 1000) {
    drawBanner(TFT_DARKGREEN, TFT_WHITE, "ZERO OK", "");
  }
}

// 太い線（幅 2r）を三角形 2 枚で塗る。drawWideLine（アンチエイリアス付き）は 8bit キャンバスだと非常に重いため
static void thickLine(float x0, float y0, float x1, float y1, float r, uint32_t color) {
  float dx = x1 - x0, dy = y1 - y0;
  float l = std::sqrt(dx * dx + dy * dy);
  if (l < 0.5f) {
    canvas.fillCircle((int)x0, (int)y0, (int)r, color);
    return;
  }
  float nx = -dy / l * r, ny = dx / l * r;
  int ax = std::lround(x0 + nx), ay = std::lround(y0 + ny), bx = std::lround(x0 - nx), by = std::lround(y0 - ny);
  int cx = std::lround(x1 - nx), cy = std::lround(y1 - ny), ex = std::lround(x1 + nx), ey = std::lround(y1 + ny);
  canvas.fillTriangle(ax, ay, bx, by, cx, cy, color);
  canvas.fillTriangle(ax, ay, cx, cy, ex, ey, color);
}

// 右側の情報カード（角丸の枠・小さなラベル・大きな値）
// alert=true のときは背景を赤、文字を白にする（1G 超えの警告用）。値はカード幅に収まるよう必要なら縮める
static void drawCard(int x, int y, int w, int h, const char* label, const char* value, uint32_t valueColor,
                     const lgfx::IFont* valueFont, bool alert = false) {
  canvas.fillRoundRect(x, y, w, h, 6, alert ? TFT_RED : canvas.color565(36, 36, 36));
  canvas.drawRoundRect(x, y, w, h, 6, alert ? TFT_WHITE : canvas.color565(110, 110, 110));
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextColor(alert ? TFT_WHITE : TFT_LIGHTGREY);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.drawString(label, x + 6, y + 5);
  canvas.setFont(valueFont);
  canvas.setTextSize(std::min(1.0f, (float)(w - 10) / canvas.textWidth(value)));
  canvas.setTextColor(alert ? TFT_WHITE : valueColor);
  canvas.setTextDatum(textdatum_t::bottom_right);
  canvas.drawString(value, x + w - 5, y + h - 4);
  canvas.setTextSize(1);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setFont(&fonts::Font0);
}

// 向きを示す小さな三角（dir: 0=上, 1=下, 2=左, 3=右）
static void drawArrow(int x, int y, int dir, uint32_t color) {
  switch (dir) {
    case 0: canvas.fillTriangle(x, y - 5, x - 5, y + 4, x + 5, y + 4, color); break;
    case 1: canvas.fillTriangle(x, y + 5, x - 5, y - 4, x + 5, y - 4, color); break;
    case 2: canvas.fillTriangle(x - 5, y, x + 4, y - 5, x + 4, y + 5, color); break;
    default: canvas.fillTriangle(x + 5, y, x - 4, y - 5, x - 4, y + 5, color); break;
  }
}

static void drawMeter(const Snapshot& s) {
  canvas.fillScreen(TFT_BLACK);

  // 目盛り円（0.25G ごと）と十字線。0.5G ごとの円は白、それ以外と十字線はグレー
  const uint32_t grey = canvas.color565(100, 100, 100);
  const int rings = (int)std::lround(RANGE_G / 0.25f);
  canvas.fillRect(CX - R, CY - 1, R * 2, 2, grey);
  canvas.fillRect(CX - 1, CY - R, 2, R * 2, grey);
  for (int i = 1; i <= rings; i++) {
    int rr = R * i / rings;
    if (i % 2 == 0) {
      canvas.drawCircle(CX, CY, rr, TFT_WHITE);
      canvas.drawCircle(CX, CY, rr - 1, TFT_WHITE);
    } else {
      canvas.drawCircle(CX, CY, rr, grey);
    }
  }
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_WHITE);
  for (int i = 2; i <= rings; i += 2) {
    canvas.setCursor(CX + R * i / rings - 20, CY + 4);
    canvas.printf("%.1f", 0.25f * i);
  }

  // 合成 G の数値（円の中、中心の下）。ボールより先に描き、ボールが重なったらボールが上になる
  float g = std::sqrt(s.fx * s.fx + s.fy * s.fy);
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", g);
  // 1G を超えたら赤い背景に白文字で警告。チラつかないよう、1G 以下に戻っても 0.2 秒は赤を保つ（数値は常に現在値）
  static uint32_t overUntil = 0;
  if (g > 1.0f) overUntil = millis() + 200;
  bool over = millis() < overUntil;
  canvas.setFont(&fonts::FreeSansBold24pt7b);
  canvas.setTextSize(1.12f);
  int gy = CY + R / 2 + 6;
  int numW = canvas.textWidth(buf);
  if (over) {
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextSize(1);
    int unitW = canvas.textWidth("G");
    int left = CX - 8 - numW / 2 - 6, right = CX - 8 + numW / 2 + 3 + unitW + 6;
    canvas.fillRoundRect(left, gy - 26, right - left, 50, 8, TFT_RED);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextSize(1.12f);
  }
  canvas.setTextColor(over ? TFT_WHITE : TFT_YELLOW);
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.drawString(buf, CX - 8, gy);
  canvas.setTextSize(1);
  canvas.setFont(&fonts::FreeSansBold12pt7b);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.drawString("G", CX - 8 + numW / 2 + 3, gy + 6);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setFont(&fonts::Font0);

  // 軌跡（新しいほど明るく大きく）
  for (int i = trailCount - 1; i >= 0; i--) {
    int idx = (trailHead - 1 - i + TRAIL) % TRAIL;
    uint8_t v = 255 - i * 200 / TRAIL;
    canvas.fillCircle(trailX[idx], trailY[idx], i < TRAIL / 4 ? 4 : 3, canvas.color565(0, v * 3 / 4, v));
  }

  // 現在値のボール（外周でクリップ）
  float px = s.fx / RANGE_G, py = s.fy / RANGE_G;
  float m = std::sqrt(px * px + py * py);
  if (m > 1.0f) { px /= m; py /= m; }
  int dx = CX + (int)(px * R), dy = CY - (int)(py * R);
  thickLine(CX, CY, dx, dy, 2.0f, TFT_ORANGE);
  canvas.fillCircle(dx, dy, 13, TFT_WHITE);
  canvas.fillCircle(dx, dy, 10, TFT_RED);

  trailX[trailHead] = dx;
  trailY[trailHead] = dy;
  trailHead = (trailHead + 1) % TRAIL;
  if (trailCount < TRAIL) trailCount++;

  // 右側の情報カード: ピーク、前後、左右
  constexpr int KX = 240, KW = 76;
  snprintf(buf, sizeof(buf), "%.1f", s.peakG);
  drawCard(KX, 6, KW, 66, "PEAK G", buf, TFT_RED, &fonts::FreeSansBold24pt7b, s.peakG > 1.0f);

  snprintf(buf, sizeof(buf), "%.2f", std::fabs(s.fy));
  drawCard(KX, 80, KW, 56, "F / B", buf, TFT_WHITE, &fonts::FreeSansBold12pt7b);
  if (std::fabs(s.fy) >= 0.02f) drawArrow(KX + 14, 80 + 38, s.fy > 0 ? 0 : 1, TFT_ORANGE);

  snprintf(buf, sizeof(buf), "%.2f", std::fabs(s.fx));
  drawCard(KX, 144, KW, 56, "L / R", buf, TFT_WHITE, &fonts::FreeSansBold12pt7b);
  if (std::fabs(s.fx) >= 0.02f) drawArrow(KX + 14, 144 + 38, s.fx > 0 ? 3 : 2, TFT_ORANGE);

  if (!calLoaded) {
    canvas.setTextColor(TFT_ORANGE);
    canvas.setCursor(KX, 210);
    canvas.print("NO CAL (B)");
  }
  drawMainButtons();
  drawZeroStatus(s);

  pushCanvas();
}

// 車のイラストの頂点をローカル座標（x 右, y 上）から画面座標へ回転・平行移動する
struct Pt {
  int x, y;
};
static Pt rotPt(int cx, int cy, float x, float y, float c, float s) {
  return {cx + (int)std::lround(x * c - y * s), cy - (int)std::lround(x * s + y * c)};
}
static void fillQuad(int cx, int cy, float c, float s, const float (&q)[8], uint32_t color) {
  Pt p0 = rotPt(cx, cy, q[0], q[1], c, s), p1 = rotPt(cx, cy, q[2], q[3], c, s);
  Pt p2 = rotPt(cx, cy, q[4], q[5], c, s), p3 = rotPt(cx, cy, q[6], q[7], c, s);
  canvas.fillTriangle(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, color);
  canvas.fillTriangle(p0.x, p0.y, p2.x, p2.y, p3.x, p3.y, color);
}

// 姿勢ゲージ 1 枚分。side=true は横から見た車（前が右）、false は後ろから見た車。
// visDeg は画面上の回転角（反時計回りが正）で、強調倍率をかけた値を渡す
static void drawAttGauge(int ox, const char* title, const char* hint, float angle, float visDeg, float mn, float mx,
                         bool side) {
  const int cx = ox + 80, cy = 100, r = 68;

  canvas.setTextSize(2);
  canvas.setTextColor(TFT_WHITE);
  canvas.setCursor(ox + 6, 4);
  canvas.print(title);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setCursor(ox + 6, 22);
  canvas.print(hint);

  // 目盛り（1° ごと、5° ごとに長く太く）。左右両端に、強調倍率をかけた角度で並べる
  thickLine(cx - r, cy, cx + r, cy, 1.0f, canvas.color565(90, 90, 90));
  for (int k = -(int)ATT_RANGE_DEG; k <= (int)ATT_RANGE_DEG; k++) {
    float va = k * VIS_GAIN * DEG_TO_RAD;
    float c = std::cos(va), s = std::sin(va);
    bool major = (k % 5 == 0);
    int tl = major ? 12 : 6;
    float w = major ? 1.6f : 1.0f;
    uint32_t col = (k == 0) ? TFT_WHITE : (major ? TFT_WHITE : TFT_LIGHTGREY);
    thickLine(cx + r * c, cy - r * s, cx + (r - tl) * c, cy - (r - tl) * s, w, col);
    thickLine(cx - r * c, cy + r * s, cx - (r - tl) * c, cy + (r - tl) * s, w, col);
    if (k != 0 && major) {
      canvas.setTextColor(TFT_WHITE);
      canvas.setCursor(cx + (int)((r + 3) * c) - (k > 0 ? 0 : 0), cy - (int)((r + 3) * s) - 3);
      canvas.printf("%d", std::abs(k));
    }
  }

  float v = std::max(-40.0f, std::min(40.0f, visDeg)) * DEG_TO_RAD;
  float c = std::cos(v), s = std::sin(v);


  if (side) {
    // 横から見た車（前が右）
    const float body[8] = {-46, -6, 46, -6, 44, 10, -46, 10};
    const float cabin[8] = {-26, 10, 22, 10, 10, 24, -18, 24};
    const float glass[8] = {-21, 12, 17, 12, 8, 21, -15, 21};
    fillQuad(cx, cy, c, s, body, TFT_SKYBLUE);
    fillQuad(cx, cy, c, s, cabin, TFT_SKYBLUE);
    fillQuad(cx, cy, c, s, glass, TFT_NAVY);
    Pt w1 = rotPt(cx, cy, -28, -8, c, s), w2 = rotPt(cx, cy, 28, -8, c, s);
    canvas.fillCircle(w1.x, w1.y, 10, TFT_WHITE);
    canvas.fillCircle(w2.x, w2.y, 10, TFT_WHITE);
    canvas.fillCircle(w1.x, w1.y, 8, TFT_DARKGREY);
    canvas.fillCircle(w2.x, w2.y, 8, TFT_DARKGREY);
    canvas.fillCircle(w1.x, w1.y, 3, TFT_LIGHTGREY);
    canvas.fillCircle(w2.x, w2.y, 3, TFT_LIGHTGREY);
  } else {
    // 後ろから見た車（車の右が画面の右）
    const float tireL[8] = {-37, -17, -21, -17, -21, -2, -37, -2};
    const float tireR[8] = {21, -17, 37, -17, 37, -2, 21, -2};
    const float body[8] = {-39, -6, 39, -6, 39, 12, -39, 12};
    const float cabin[8] = {-28, 12, 28, 12, 20, 28, -20, 28};
    const float glass[8] = {-23, 14, 23, 14, 17, 25, -17, 25};
    fillQuad(cx, cy, c, s, tireL, TFT_WHITE);
    fillQuad(cx, cy, c, s, tireR, TFT_WHITE);
    fillQuad(cx, cy, c, s, body, TFT_SKYBLUE);
    fillQuad(cx, cy, c, s, cabin, TFT_SKYBLUE);
    fillQuad(cx, cy, c, s, glass, TFT_NAVY);
  }

  // 車体の基準線＝針（目盛りまで伸ばす）。白い目盛りと区別できるよう赤にし、車のイラストの上に重ねて線全体を見せる
  thickLine(cx - r * c, cy + r * s, cx + r * c, cy - r * s, 2.5f, TFT_RED);

  // 数値（度）。大きく表示し、桁が多くてパネル幅に入らないときだけ縮める
  char buf[16];
  snprintf(buf, sizeof(buf), "%+.1f", angle);
  canvas.setFont(&fonts::FreeSansBold24pt7b);
  canvas.setTextSize(1);
  float size = std::min(1.5f, 152.0f / canvas.textWidth(buf));
  canvas.setTextSize(size);
  canvas.setTextColor(TFT_YELLOW);
  canvas.setTextDatum(textdatum_t::bottom_center);
  canvas.drawString(buf, ox + 80, 227);
  canvas.setTextSize(1);
  canvas.setFont(&fonts::Font0);

  // 最小・最大（右上）
  canvas.setTextColor(TFT_RED);
  canvas.setTextDatum(textdatum_t::top_right);
  snprintf(buf, sizeof(buf), "max %+.1f", mx);
  canvas.drawString(buf, ox + 156, 4);
  snprintf(buf, sizeof(buf), "min %+.1f", mn);
  canvas.drawString(buf, ox + 156, 14);
  canvas.setTextDatum(textdatum_t::top_left);
}

static void drawAttitude(const Snapshot& s) {
  canvas.fillScreen(TFT_BLACK);
  // ピッチ: ノーズアップが正 → 前（右）が上がるよう反時計回り
  drawAttGauge(0, "PITCH", "deg, nose up +", s.pitch, s.pitch * VIS_GAIN, s.pitchMin, s.pitchMax, true);
  // ロール: 右側が下がるのが正 → 右が下がるよう時計回り
  drawAttGauge(160, "ROLL", "deg, right down +", s.roll, -s.roll * VIS_GAIN, s.rollMin, s.rollMax, false);
  canvas.drawFastVLine(160, 0, 226, TFT_DARKGREY);

  canvas.setTextSize(1);
  canvas.setTextColor(TFT_GREEN);
  canvas.setCursor(4, 156);
  if (s.stationary) canvas.print("STILL");
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setCursor(280, 156);
  canvas.printf("car x%.0f", VIS_GAIN);
  drawMainButtons();
  drawZeroStatus(s);

  pushCanvas();
}

// 推移グラフ 1 枚分（y0..y0+h）。data は古い順に n 個。
// 縦軸は ±minScale（unipolar なら 0..minScale）を基本に、超えたら step 刻みで広げる。グリッドも step ごと。
// fmt は軸ラベル・現在値の書式。unipolar のグラフは線の下を暗く塗る
static void drawTrendGraph(int y0, int h, const char* title, const char* hint, const float* data, int n,
                           uint32_t color, float minScale, float step, const char* fmt, bool unipolar = false,
                           uint32_t fillColor = 0) {
  constexpr int X0 = 36, X1 = 316;
  float maxAbs = 0;
  for (int i = 0; i < n; i++) maxAbs = std::max(maxAbs, std::fabs(data[i]));
  float scale = std::max(minScale, std::ceil(maxAbs / step) * step);
  int cy = y0 + h / 2;
  int base = y0 + h - 3;
  auto yOf = [&](float v) {
    if (unipolar) return base - (int)std::lround(std::max(0.0f, v) / scale * (h - 6));
    return cy - (int)std::lround(v / scale * (h / 2 - 2));
  };

  // 線の下の塗りつぶし（unipolar のみ）
  if (unipolar && n > 1) {
    for (int i = 0; i < n; i++) {
      int x = X1 - (X1 - X0) * (n - 1 - i) / (HIST - 1);
      int y = yOf(data[i]);
      if (y < base) canvas.drawFastVLine(x, y, base - y, fillColor);
    }
  }

  // 枠とグリッド（step ごと）、時間グリッド（2 秒ごと）
  uint32_t gridCol = canvas.color565(70, 70, 70);
  for (int t = 0; t <= 10; t += 2) {
    int x = X1 - (X1 - X0) * t / 10;
    canvas.drawFastVLine(x, y0, h, gridCol);
  }
  canvas.setTextSize(1);
  int steps = (int)std::lround(scale / step);
  // ラベルが詰まりすぎる（12px 未満）ときは 1 つおきに出す
  float pxPerStep = std::fabs((float)(yOf(0) - yOf(step)));
  int labelEvery = pxPerStep < 12 ? 2 : 1;
  for (int k = unipolar ? 0 : -steps; k <= steps; k++) {
    float g = k * step;
    int y = yOf(g);
    bool zero = (k == 0);
    canvas.drawFastHLine(X0, y, X1 - X0, zero ? TFT_LIGHTGREY : gridCol);
    if (zero) canvas.drawFastHLine(X0, y + 1, X1 - X0, TFT_LIGHTGREY);
    if (k % labelEvery != 0) continue;
    canvas.setTextColor(TFT_LIGHTGREY);
    canvas.setTextDatum(textdatum_t::middle_right);
    char lb[12];
    snprintf(lb, sizeof(lb), unipolar ? "%.2f" : fmt, g);
    canvas.drawString(zero ? "0" : lb, X0 - 3, y);
  }
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.drawRect(X0, y0, X1 - X0 + 1, h, TFT_DARKGREY);

  // 折れ線（新しいものが右端）。2 ピクセルの太さで描く
  for (int i = 1; i < n; i++) {
    int xa = X1 - (X1 - X0) * (n - i) / (HIST - 1);
    int xb = X1 - (X1 - X0) * (n - 1 - i) / (HIST - 1);
    int ya = yOf(data[i - 1]), yb = yOf(data[i]);
    canvas.drawLine(xa, ya, xb, yb, color);
    canvas.drawLine(xa, ya + 1, xb, yb + 1, color);
  }

  // タイトルと現在値
  canvas.setTextColor(color);
  canvas.setTextSize(2);
  canvas.setCursor(X0 + 4, y0 + 3);
  canvas.print(title);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setCursor(X0 + 4, y0 + 20);
  canvas.print(hint);
  if (n > 0) {
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(color);
    canvas.setTextDatum(textdatum_t::top_right);
    char buf[16];
    snprintf(buf, sizeof(buf), unipolar ? "%.2f" : fmt, data[n - 1]);
    canvas.drawString(buf, X1 - 4, y0 + 3);
    canvas.setTextDatum(textdatum_t::top_left);
    canvas.setFont(&fonts::Font0);
  }
}

// 履歴リングバッファから古い順に 2 系列を取り出す
static int copyHistory(const float* srcA, const float* srcB, float* a, float* b) {
  portENTER_CRITICAL(&mux);
  int n = histCount;
  for (int i = 0; i < n; i++) {
    int idx = (histHead - n + i + HIST) % HIST;
    a[i] = srcA[idx];
    b[i] = srcB[idx];
  }
  portEXIT_CRITICAL(&mux);
  return n;
}

static void drawTrendScreen(const Snapshot& s, const char* titleA, const char* hintA, const float* a,
                            const char* titleB, const char* hintB, const float* b, int n, float minScale,
                            float step, const char* fmt) {
  canvas.fillScreen(TFT_BLACK);
  drawTrendGraph(2, 108, titleA, hintA, a, n, TFT_YELLOW, minScale, step, fmt);
  drawTrendGraph(114, 108, titleB, hintB, b, n, TFT_CYAN, minScale, step, fmt);
  drawMainButtons();
  drawZeroStatus(s);
  pushCanvas();
}

static float trendA[HIST], trendB[HIST];

static void drawTrend(const Snapshot& s) {
  int n = copyHistory(histP, histR, trendA, trendB);
  drawTrendScreen(s, "PITCH", "deg, nose up +", trendA, "ROLL", "deg, right down +", trendB, n, ATT_RANGE_DEG, 5.0f,
                  "%+.1f");
}

static float trendG[HIST];

// G の推移: 上に合成 G（0〜1.5G）、下に前後・左右
static void drawGTrend(const Snapshot& s) {
  portENTER_CRITICAL(&mux);
  int n = histCount;
  for (int i = 0; i < n; i++) {
    int idx = (histHead - n + i + HIST) % HIST;
    trendG[i] = histG[idx];
    trendA[i] = histFB[idx];
    trendB[i] = histLR[idx];
  }
  portEXIT_CRITICAL(&mux);

  canvas.fillScreen(TFT_BLACK);
  drawTrendGraph(2, 98, "G", "combined", trendG, n, TFT_ORANGE, RANGE_G, 0.25f, "%.2f", true,
                 canvas.color565(90, 45, 0));
  drawTrendGraph(103, 61, "F/B", "ball up +", trendA, n, TFT_YELLOW, 0.5f, 0.25f, "%+.2f");
  drawTrendGraph(167, 61, "L/R", "ball right +", trendB, n, TFT_CYAN, 0.5f, 0.25f, "%+.2f");
  drawMainButtons();
  drawZeroStatus(s);
  pushCanvas();
}

// デバッグ画面: 生値・補正値・推定の内部状態をまとめて表示する
static float dbgImuHz = 0, dbgFps = 0;
static int dbgY = 0;
static void dbgLine(uint32_t col, const char* fmt, ...) {
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  canvas.setTextColor(col);
  canvas.setCursor(4, dbgY);
  canvas.print(buf);
  dbgY += 11;
}

static void drawDebug(const Snapshot& s) {
  canvas.fillScreen(TFT_BLACK);
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  dbgY = 2;
  dbgLine(TFT_YELLOW, "DEBUG");
  dbgLine(TFT_CYAN, "acc raw  %+7.3f %+7.3f %+7.3f  |%5.3f|", s.raw.x, s.raw.y, s.raw.z, len(s.raw));
  dbgLine(TFT_CYAN, "acc cal  %+7.3f %+7.3f %+7.3f  |%5.3f|", s.acc.x, s.acc.y, s.acc.z, len(s.acc));
  dbgLine(TFT_CYAN, "gyro     %+7.2f %+7.2f %+7.2f dps", s.gyro.x, s.gyro.y, s.gyro.z);
  dbgLine(TFT_CYAN, "bias     %+7.2f %+7.2f %+7.2f dps", s.bias.x, s.bias.y, s.bias.z);
  dbgY += 4;
  dbgLine(TFT_WHITE, "G frame  x(L/R)%+6.3f y(F/B)%+6.3f z%+6.3f", s.fx, s.fy, s.fz);
  dbgLine(TFT_WHITE, "pitch    gyro %+7.2f  acc %+7.2f  rate %+7.2f", s.pitch, s.accPitch, s.rateP);
  dbgLine(TFT_WHITE, "roll     gyro %+7.2f  acc %+7.2f  rate %+7.2f", s.roll, s.accRoll, s.rateR);
  dbgLine(TFT_WHITE, "sd acc %.4f g  gyro %.3f dps  %s", s.accSd, s.gyroSd, s.stationary ? "STILL" : "moving");
  dbgY += 4;
  dbgLine(TFT_LIGHTGREY, "g0    %+6.3f %+6.3f %+6.3f", g0.x, g0.y, g0.z);
  dbgLine(TFT_LIGHTGREY, "fwd   %+6.3f %+6.3f %+6.3f", fwd.x, fwd.y, fwd.z);
  dbgLine(TFT_LIGHTGREY, "right %+6.3f %+6.3f %+6.3f", right.x, right.y, right.z);
  dbgLine(calLoaded ? TFT_LIGHTGREY : TFT_ORANGE, "cal %s off %+.3f %+.3f %+.3f", calLoaded ? "OK" : "NONE", calOffset[0],
       calOffset[1], calOffset[2]);
  dbgLine(TFT_LIGHTGREY, "        scl %.3f %.3f %.3f", calScale[0], calScale[1], calScale[2]);
  dbgY += 4;
  {
    portENTER_CRITICAL(&mux);
    uint32_t dropped = logDropped;
    int fill = (logHead - logTail + LOG_BUF) % LOG_BUF;
    portEXIT_CRITICAL(&mux);
    const char* st[] = {"init", "no card", "ready", "REC", "ERROR", "saving", "safe"};
    dbgLine(sdState == SD_ERROR ? TFT_RED : TFT_LIGHTGREY, "sd %s %s lines %lu drop %lu", st[sdState], logPath,
            (unsigned long)logLines, (unsigned long)dropped);
    dbgLine(TFT_LIGHTGREY, "   buf %d/%d (max %d)  max write %lums", fill, LOG_BUF, logMaxFill,
            (unsigned long)logMaxWriteMs);
  }
  dbgLine(TFT_GREEN, "imu %.0f Hz  fps %.0f  heap %u  canvas %dbit", dbgImuHz, dbgFps, ESP.getFreeHeap(),
       (int)canvas.getColorDepth());
  drawMainButtons();
  drawZeroStatus(s);
  pushCanvas();
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.BtnB.setHoldThresh(CAL_HOLD_MS);  // B ボタンは 3 秒長押しでキャリブレーション
  Serial.begin(115200);

  // 太線のアンチエイリアスがきれいに出るよう 16bit で確保し、メモリが足りなければ 8bit にする
  canvas.setColorDepth(16);
  if (!canvas.createSprite(M5.Display.width(), M5.Display.height())) {
    canvas.setColorDepth(8);
    canvas.createSprite(M5.Display.width(), M5.Display.height());
  }
  Serial.printf("canvas %dbit, free heap %u\n", canvas.getColorDepth(), ESP.getFreeHeap());

  loadCal();
  spiMutex = xSemaphoreCreateMutex();
  lastMicros = micros();
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 3, nullptr, 0);
  // SD 書き込みはコア 0 の低優先度タスク。SD が止まっても IMU タスク（優先度 3）が割り込んで動き続ける
  xTaskCreatePinnedToCore(sdTask, "sd", 8192, nullptr, 1, nullptr, 0);
}

static uint32_t bPressStart = 0;

void loop() {
  // 操作は 1 文字のコマンドで扱う。シリアルからの入力（リモートでの動作確認用）と本体ボタンが同じコマンドになる
  int key = 0;
  if (Serial.available()) key = Serial.read();

  // 本体ボタン（GPIO なので IMU タスクの I2C とは競合しない）
  M5.update();
  // A=記録の開始/停止, B=短押し(離したとき)でゼロ点・3 秒長押しでキャリブレーション, C=モード切替
  // キャリブレーション中は B=測定, C=中止
  calHoldProgress = 0;
  if (calMode) {
    if (M5.BtnB.wasPressed()) key = ' ';
    if (M5.BtnC.wasPressed()) key = 'c';
  } else {
    if (M5.BtnA.wasPressed()) key = 'l';
    if (M5.BtnB.wasHold()) {
      key = 'c';
    } else if (M5.BtnB.wasClicked()) {
      key = ' ';
    } else if (M5.BtnB.isPressed() && M5.BtnB.pressedFor(500)) {
      calHoldProgress = std::min(1.0f, (float)(millis() - bPressStart) / CAL_HOLD_MS);
    }
    if (M5.BtnB.wasPressed()) bPressStart = millis();
    if (M5.BtnC.wasPressed()) key = 'm';
  }

  Snapshot s = snapshot();

  if (calMode) {
    if (key == ' ') {
      calMsg = "Capturing...";
      calMsgColor = TFT_YELLOW;
      drawCal(s);
      captureCalFace();
    } else if (key == 'c' || key == 'C') {
      calMode = false;
      Serial.println("cal canceled");
    }
    if (calMode) drawCal(snapshot());
    delay(20);
    return;
  }

  if (key == ' ') {
    zeroRequest = true;
    trailCount = 0;
  } else if (key == 'c' || key == 'C') {
    startCal();
    return;
  } else if (key == 'm' || key == 'M') {
    mode = (mode + 1) % MODE_COUNT;
    trailCount = 0;
    Serial.printf("mode -> %d\n", mode);
  } else if (key == 'l' || key == 'L') {
    logToggleRequest = true;
  } else if (key == 'r' || key == 'R') {
    peakResetRequest = true;
  }

  // 検証用ログ（200ms ごと）。imuHz は IMU タスクの実際のサンプリング周波数
  static uint32_t lastLog = 0, lastSamples = 0, frames = 0;
  static float fps = 0;
  frames++;
  uint32_t nowMs = millis();
  if (nowMs - lastLog >= 200) {
    float hz = (s.samples - lastSamples) * 1000.0f / (nowMs - lastLog);
    lastLog = nowMs;
    lastSamples = s.samples;
    fps = frames * 1000.0f / 200;
    frames = 0;
    dbgImuHz = hz;
    dbgFps = fps;
    Serial.printf("|cal|=%5.3f pitch=%+6.2f roll=%+6.2f acc(p,r)=%+6.2f %+6.2f rate(p,r)=%+7.2f %+7.2f "
                  "bias=(%+.2f %+.2f %+.2f) sd(a,w)=%.3f %.2f %s imuHz=%.0f fps=%.0f\n",
                  len(s.acc), s.pitch, s.roll, s.accPitch, s.accRoll, s.rateP, s.rateR, s.bias.x, s.bias.y,
                  s.bias.z, s.accSd, s.gyroSd, s.stationary ? "STILL" : "", hz, fps);
  }

  uint32_t t0 = micros();
  switch (mode) {
    case MODE_ATTITUDE: drawAttitude(s); break;
    case MODE_TREND: drawTrend(s); break;
    case MODE_GMETER: drawMeter(s); break;
    case MODE_GTREND: drawGTrend(s); break;
    default: drawDebug(s); break;
  }
  static uint32_t lastDrawLog = 0;
  if (millis() - lastDrawLog > 1000) {
    lastDrawLog = millis();
    Serial.printf("draw mode=%d %luus\n", mode, (unsigned long)(micros() - t0));
  }
  delay(5);
}
