#include "Joystick.h"

//----------------------------------------------------------
// ロータリーエンコーダ
class RotaryEncoder_
{
public:
  // 作成
  RotaryEncoder_(
    uint8_t switchApin,   // スイッチAのピン番号
    uint8_t switchBpin,   // スイッチBのピン番号
    int initValue=512,    // ダイヤル位置の初期値
    int minValue=0,       // ダイヤル位置の最小値
    int maxValue=1023,    // ダイヤル位置の最大値
    int deltaValue=64) :  // ダイヤル位置の増減値
    m_value(initValue),
    m_switchApin(switchApin),
    m_switchBpin(switchBpin),
    m_minValue(minValue),
    m_maxValue(maxValue),
    m_deltaValue(deltaValue)
  {
    // スイッチA/Bピンを、プルアップのデジタル入力に設定
    pinMode(switchApin, INPUT_PULLUP);
    pinMode(switchBpin, INPUT_PULLUP);
  }

  // 回転検出
  // dt : 前回呼び出しからの経過時間(マイクロ秒)
  // 可能な限り高頻度で呼出すこと。
  // dt はスイッチのチャタリング対策で参照する。
  void update(const uint16_t dt);

  // ダイヤル位置の取得
  int getValue() const
  {
    return m_value;
  }

  // 時計回り(CW)トリガーの参照
  bool getCwTrigger()
  {
    bool r = m_cwTrigger;
    m_cwTrigger = false;
    return r;
  }
  
  // 反時計回り(CCW)トリガーの参照
  bool getCcwTrigger()
  {
    bool r = m_ccwTrigger;
    m_ccwTrigger = false;
    return r;
  }

private:
  // 検出結果
  int     m_value;              // ダイヤル位置(0 - 1023)
  bool    m_cwTrigger = false;  // 時計回り(CW)トリガー
  bool    m_ccwTrigger = false; // 反時計回り(CCW)トリガー

  // パラメータ
  uint8_t m_switchApin; // スイッチAのピン番号
  uint8_t m_switchBpin; // スイッチBのピン番号
  int     m_minValue;   // ダイヤル位置の最小値
  int     m_maxValue;   // ダイヤル位置の最大値
  int     m_deltaValue; // ダイヤル位置の増減値

  // 回転方向の検出
  enum ERotateDir { RotateNon, CW, CCW }; // 回転方向
  ERotateDir m_fetchedRotDir = RotateNon; // 回転開始方向

  // チャタリング防止
  bool     m_A0 = true;             // スイッチAの直前の値
  bool     m_B0 = false;            // スイッチBの直前の値
  bool     m_holdB = false;         // スイッチBの状態(チャタリング防止済み)
  uint16_t m_chatEquivalTimeA = 0;  // スイッチAで同じ状態が続いているタイマー(減算)
  uint16_t m_chatEquivalTimeB = 0;  // スイッチBで同じ状態が続いているタイマー(減算)
  const uint16_t m_chatUpThreshold = 250; // チャタリングが終わったと判定する時間(ミリ秒)
};

// 0クリップされる減算
inline uint16_t sat_sub(uint16_t x, uint16_t y)
{
  return (x < y)? 0: (x - y);
}

// 回転検出
void RotaryEncoder_::update(const uint16_t dt)
{
  // インクリメンタルタイプのロータリーエンコーダのスイッチの読み込み
  const bool A = (digitalRead(m_switchApin) != LOW);
  const bool B = (digitalRead(m_switchBpin) != LOW);

  // スイッチAのエッジ検出
  enum EEdgeDir { EdgeNon, DownEdge, UpEdge };// エッジ方向
  EEdgeDir edgeDir = EdgeNon;
  // チャタリング防止判定付きのエッジ検出
  if((m_chatEquivalTimeA == 0) && (m_A0 != A))
  {
    edgeDir = (A? UpEdge: DownEdge);
  }
  // チャタリング防止の不感タイムを設定/カウントダウン
  if(m_A0 != A) m_chatEquivalTimeA = m_chatUpThreshold;
  else          m_chatEquivalTimeA = sat_sub(m_chatEquivalTimeA, dt);
  m_A0 = A;

  // スイッチBの状態保持
  // チャタリング防止判定付き
  if(m_chatEquivalTimeB == 0)
  {
    m_holdB = B;
  }
  // チャタリング防止の不感タイムを設定/カウントダウン
  if(m_B0 != B) m_chatEquivalTimeB = m_chatUpThreshold;
  else          m_chatEquivalTimeB = sat_sub(m_chatEquivalTimeB, dt);
  m_B0 = B;

  // クリック安定点からの回転開始の検出(スイッチAのダウンエッジ)
  if(edgeDir == DownEdge)
  {
    m_fetchedRotDir = (m_holdB? CW: CCW);
  }
  // 次のクリック安定点への到着の検出(スイッチAのアップエッジ)
  else if(edgeDir == UpEdge)
  {
    // 時計回り(CW)検出
    if((m_fetchedRotDir == CW) && !m_holdB)
    {
      // ダイヤル位置を変更
      m_value += m_deltaValue;
      // 時計回りトリガーをセット
      m_cwTrigger = true;
      // 回転方向をリセット
      m_fetchedRotDir = RotateNon;
    }
    // 半時計回り(CCW)検出
    else if((m_fetchedRotDir == CCW) && m_holdB)
    {
      // ダイヤル位置を変更
      m_value -= m_deltaValue;
      // 反時計回りトリガーをセット
      m_ccwTrigger = true;
      // 回転方向をリセット
      m_fetchedRotDir = RotateNon;
    }

    // ダイヤル位置を範囲内にクリッピング
    m_value = constrain(m_value, m_minValue, m_maxValue);
  }
}

//----------------------------------------------------------
// サンプル
const uint8_t buttonPin[] = { 8, 9 }; // ボタンピン番号
const uint8_t analogPin = 18;         // ボリュームピン番号

Joystick_ Joystick(
    JOYSTICK_DEFAULT_REPORT_ID, // hidReportId
    JOYSTICK_TYPE_JOYSTICK,     // joystickType
    10,     // ボタン x4
    0,     // hatSwitchCount
    true,  // X軸 : ボリューム値
    true,  // Y軸 : ロータリーエンコーダーのダイヤル値
    false, // includeZAxis
    false, // includeRxAxis
    false, // includeRyAxis
    false, // includeRzAxis
    false, // includeRudder
    false, // includeThrottle
    false, // includeAccelerator
    false, // includeBrake
    false);// includeSteering

// ピン7,6 に接続されたロータリーエンコーダを初期化
RotaryEncoder_ rotaryEncoder(7, 6);

//----------------------------------------------------------
void setup()
{
  pinMode(analogPin, INPUT);
  
  for(uint8_t i = 0; i < 2; i++)
  {
    pinMode(buttonPin[i], INPUT_PULLUP);
  }

  Joystick.begin(false);
}

//----------------------------------------------------------
// 経過時間をマイクロ秒で取得
uint16_t elapsed_micros()
{
  // マイクロ秒精度の経過時間の計算
  const uint32_t time = micros();
  static uint32_t time0 = time;
  uint16_t dt = (time - time0);
  // タイマーのオーバーフロー対策
  if(time < time0){
    dt = (UINT32_MAX - time0) + 1 + time;
  }
  time0 = time;
  return dt;
}

//----------------------------------------------------------
void loop()
{
  // 経過時間の取得(マイクロ秒)
  uint16_t dt = elapsed_micros();

  // ロータリーエンコーダーの回転検出は全力で
  rotaryEncoder.update(dt);

  // ゲームコントローラの状態更新は秒間50回(20[ms])程度で。
  static int16_t nextSendTime = 0;
  if(nextSendTime == 0)
  {
    // ロータリーエンコーダのダイヤル値 -> Y軸
    Joystick.setYAxis(rotaryEncoder.getValue());
    // ロータリーエンコーダの時計/反時計回りトリガーをボタンに設定(オプション)
    // PC側で確認しやすいように、トリガーを0.1秒のボタン押し込みに変換する
    static int8_t cwTriggerCnt = 0;
    static int8_t ccwTriggerCnt = 0;
    if(rotaryEncoder.getCwTrigger()) cwTriggerCnt = 5;
    if(rotaryEncoder.getCcwTrigger()) ccwTriggerCnt = 5;
    Joystick.setButton(2, (0 < cwTriggerCnt? 1: 0));
    Joystick.setButton(3, (0 < ccwTriggerCnt? 1: 0));
    cwTriggerCnt = max(0, cwTriggerCnt-1);
    ccwTriggerCnt = max(0, ccwTriggerCnt-1);

    // アナログ入力 -> X軸
    int analogVal = analogRead(analogPin);
    Joystick.setXAxis(analogVal);

    // ボタン
    for(uint8_t i = 0; i < 2; i++)
    {
      uint8_t buttonState = digitalRead(buttonPin[i]);
      uint8_t buttonVal = (buttonState == HIGH)? 1: 0;
      Joystick.setButton(i, buttonVal);
    }
    Joystick.sendState();

    // 次の20[ms]後に
    nextSendTime = 20*1000;
  }
  // 経過時間のカウントダウン
  nextSendTime = sat_sub(nextSendTime, dt);
}
